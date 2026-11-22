// Banco 4: IMU (basado en Prueba_IMU.ino). Además de los datos crudos calcula la inclinación y la
// superficie (suelo, transición, pared, techo) con GravityFilter y SurfaceClassifier, como el firmware.

// ESP32 DevKit V1 · I2C: SDA = GPIO21, SCL = GPIO22 · MPU-6050 en 0x68 (AD0 a GND)
// No necesita librerías externas, solo Wire.
// Monitor serie: 115200 baudios, fin de línea "Nueva línea". Escribe AYUDA.

#include <Arduino.h>
#include <Wire.h>

#include "GravityFilter.h"
#include "SurfaceClassifier.h"
#include "config.h"

const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint8_t MPU = 0x68;

// Pines de drivers y ESC: se fijan en bajo para que nada se mueva durante la prueba
const int PINES_APAGADOS[] = {17, 19, 18, 23, 4, 16, 25, 26};

// Registros del MPU-6050
const uint8_t REG_SMPLRT_DIV   = 0x19;
const uint8_t REG_CONFIG       = 0x1A;
const uint8_t REG_GYRO_CONFIG  = 0x1B;
const uint8_t REG_ACCEL_CONFIG = 0x1C;
const uint8_t REG_ACCEL_XOUT_H = 0x3B;
const uint8_t REG_PWR_MGMT_1   = 0x6B;
const uint8_t REG_WHO_AM_I     = 0x75;

const int   A_RANGO[] = {2, 4, 8, 16};             // ±g
const float A_SENS[]  = {16384, 8192, 4096, 2048}; // LSB/g
const int   G_RANGO[] = {250, 500, 1000, 2000};    // ±°/s
const float G_SENS[]  = {131, 65.5, 32.8, 16.4};   // LSB/(°/s)
const int   DLPF_HZ[] = {260, 184, 94, 44, 21, 10, 5};

int aIdx = 1;   // ±4 g
int gIdx = 1;   // ±500 °/s
int dlpf = 3;   // 44 Hz

const float ALFA = 0.98;  // filtro complementario
const unsigned long PERIODO_US = 5000;  // muestreo a 200 Hz

int16_t raw[7];
float ax, ay, az, gx, gy, gz, temp;
float aOff[3] = {0, 0, 0}, gOff[3] = {0, 0, 0};
float roll = 0, pitch = 0, yaw = 0;
bool angInit = false;

enum Modo { SUP, DATOS, ANG, PLOT_ANG, PLOT_A, PLOT_G, RAW, NADA };
Modo modo = SUP;

GravityFilter gravedad(imu::kAccelWeight);
SurfaceClassifier superficie(imu::kBands);
bool gravedadInit = false;
float hzImpresion = 10;

bool imuOk = false;
int fallosSeguidos = 0;
unsigned long errores = 0;
unsigned long tMuestra = 0, tImpresion = 0, tReintento = 0;
String buffer;

// ---------- I2C ----------

bool escribir(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool leer(uint8_t reg, uint8_t* buf, uint8_t n) {
  Wire.beginTransmission(MPU);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom(MPU, n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void escanear() {
  Serial.println("Escaneando bus I2C...");
  int n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      const char* nombre = (a == 0x68) ? " MPU-6050"
                         : (a == 0x69) ? " MPU-6050 con AD0 en alto (revisa AD0)"
                         : (a == 0x29) ? " VL53L0X"
                         : "";
      Serial.printf("  0x%02X%s\n", a, nombre);
      n++;
    }
  }
  if (n == 0) Serial.println("  Nada responde: revisa SDA (GPIO21), SCL (GPIO22), 3,3 V y GND");
}

// ---------- MPU-6050 ----------

bool configurarImu() {
  uint8_t who = 0;
  if (!leer(REG_WHO_AM_I, &who, 1)) {
    Serial.println("El MPU-6050 no responde en 0x68");
    return false;
  }
  Serial.printf("WHO_AM_I = 0x%02X%s\n", who,
                who == 0x68 ? " (MPU-6050 OK)" : " (no es 0x68: puede ser un clon, se intenta igual)");

  bool ok = escribir(REG_PWR_MGMT_1, 0x80);  // reset
  delay(100);
  ok &= escribir(REG_PWR_MGMT_1, 0x01);      // despierta, reloj del giroscopio X
  delay(10);
  ok &= escribir(REG_SMPLRT_DIV, 4);         // 1 kHz / (1 + 4) = 200 Hz
  ok &= escribir(REG_CONFIG, dlpf);
  ok &= escribir(REG_GYRO_CONFIG, gIdx << 3);
  ok &= escribir(REG_ACCEL_CONFIG, aIdx << 3);
  if (!ok) Serial.println("Error al escribir la configuración");
  else Serial.printf("Configurado: ±%d g, ±%d °/s, filtro %d Hz\n",
                     A_RANGO[aIdx], G_RANGO[gIdx], DLPF_HZ[dlpf]);
  angInit = false;
  gravedadInit = false;
  return ok;
}

bool leerImu() {
  uint8_t b[14];
  if (!leer(REG_ACCEL_XOUT_H, b, 14)) { errores++; return false; }
  for (int i = 0; i < 7; i++) raw[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
  ax = raw[0] / A_SENS[aIdx] - aOff[0];
  ay = raw[1] / A_SENS[aIdx] - aOff[1];
  az = raw[2] / A_SENS[aIdx] - aOff[2];
  temp = raw[3] / 340.0f + 36.53f;
  gx = raw[4] / G_SENS[gIdx] - gOff[0];
  gy = raw[5] / G_SENS[gIdx] - gOff[1];
  gz = raw[6] / G_SENS[gIdx] - gOff[2];
  return true;
}

void actualizarAngulos(float dt) {
  float rollAcc  = atan2f(ay, az) * RAD_TO_DEG;
  float pitchAcc = atan2f(-ax, sqrtf(ay * ay + az * az)) * RAD_TO_DEG;
  if (!angInit) {
    roll = rollAcc;
    pitch = pitchAcc;
    angInit = true;
  } else {
    roll  = ALFA * (roll  + gx * dt) + (1 - ALFA) * rollAcc;
    pitch = ALFA * (pitch + gy * dt) + (1 - ALFA) * pitchAcc;
  }
  yaw += gz * dt;  // solo giroscopio: deriva con el tiempo

  const ImuSample s{ax, ay, az, gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD};
  if (!gravedadInit) {
    gravedad.reset(s);
    gravedadInit = true;
  } else {
    gravedad.update(s, dt);
  }
  superficie.update(gravedad.tiltDeg(), millis());
}

// conAcel = true: también corrige el acelerómetro (robot plano, Z hacia arriba)
void calibrar(bool conAcel) {
  Serial.println(conAcel ? "Calibrando giroscopio y acelerometro: robot quieto y plano, Z hacia arriba..."
                         : "Calibrando giroscopio: robot quieto (cualquier posicion)...");
  delay(1000);
  float aPrev[3], gPrev[3];
  for (int i = 0; i < 3; i++) {
    aPrev[i] = aOff[i];
    gPrev[i] = gOff[i];
    gOff[i] = 0;
    if (conAcel) aOff[i] = 0;
  }

  const int N = 600;
  double s[6] = {0}, s2[3] = {0};
  int ok = 0;
  for (int i = 0; i < N; i++) {
    if (leerImu()) {
      float v[6] = {ax, ay, az, gx, gy, gz};
      for (int k = 0; k < 6; k++) s[k] += v[k];
      for (int k = 0; k < 3; k++) s2[k] += v[3 + k] * v[3 + k];
      ok++;
    }
    delay(3);
  }
  if (ok < N / 2) {
    Serial.println("Demasiados errores de lectura, calibracion cancelada");
    for (int i = 0; i < 3; i++) { aOff[i] = aPrev[i]; gOff[i] = gPrev[i]; }
    return;
  }

  float ruidoMax = 0;
  for (int k = 0; k < 3; k++) {
    gOff[k] = s[3 + k] / ok;
    float var = s2[k] / ok - gOff[k] * gOff[k];
    ruidoMax = max(ruidoMax, sqrtf(max(var, 0.0f)));
  }
  if (conAcel) {
    aOff[0] = s[0] / ok;
    aOff[1] = s[1] / ok;
    aOff[2] = s[2] / ok - 1.0f;
  }

  Serial.printf("Offset giro [°/s]: x %+.2f  y %+.2f  z %+.2f\n", gOff[0], gOff[1], gOff[2]);
  if (conAcel) Serial.printf("Offset acel [g]:   x %+.3f  y %+.3f  z %+.3f\n", aOff[0], aOff[1], aOff[2]);
  if (ruidoMax > 1.0f) Serial.println("Aviso: el giroscopio varió bastante. ¿Se movió el robot? Repite la calibración.");
  angInit = false;
  gravedadInit = false;
  yaw = 0;
}

// Media y desviación estándar durante unos segundos (útil para ver vibración de hélices o motores)
void vibracion(float seg) {
  seg = constrain(seg, 0.5f, 30.0f);
  Serial.printf("Midiendo %.1f s...\n", seg);
  double s[6] = {0}, s2[6] = {0};
  int n = 0;
  float aMax = 0;
  unsigned long fin = millis() + (unsigned long)(seg * 1000);
  while ((long)(millis() - fin) < 0) {
    if (leerImu()) {
      float v[6] = {ax, ay, az, gx, gy, gz};
      for (int k = 0; k < 6; k++) { s[k] += v[k]; s2[k] += v[k] * v[k]; }
      aMax = max(aMax, sqrtf(ax * ax + ay * ay + az * az));
      n++;
    }
    delay(5);
  }
  if (n == 0) { Serial.println("Sin lecturas"); return; }
  const char* eje[] = {"ax", "ay", "az", "gx", "gy", "gz"};
  Serial.printf("%d muestras\n", n);
  for (int k = 0; k < 6; k++) {
    float m = s[k] / n;
    float d = sqrtf(max((float)(s2[k] / n - m * m), 0.0f));
    Serial.printf("  %s  media %+8.3f  desv %7.3f %s\n", eje[k], m, d, k < 3 ? "g" : "°/s");
  }
  Serial.printf("  |a| maximo: %.2f g\n", aMax);
}

// ---------- Salida ----------

void imprimir() {
  switch (modo) {
    case SUP:
      Serial.printf("Inclinacion %6.1f  superficie %-10s  |a| %.3f g\n", gravedad.tiltDeg(),
                    toString(superficie.current()), sqrtf(ax * ax + ay * ay + az * az));
      break;
    case DATOS:
      Serial.printf("A[g] %+6.3f %+6.3f %+6.3f | G[°/s] %+7.2f %+7.2f %+7.2f | T %.1f C\n",
                    ax, ay, az, gx, gy, gz, temp);
      break;
    case ANG:
      Serial.printf("Roll %+7.2f  Pitch %+7.2f  Yaw %+8.2f  |a| %.3f g\n",
                    roll, pitch, yaw, sqrtf(ax * ax + ay * ay + az * az));
      break;
    case PLOT_ANG:
      Serial.printf("roll:%.2f,pitch:%.2f,yaw:%.2f\n", roll, pitch, yaw);
      break;
    case PLOT_A:
      Serial.printf("ax:%.3f,ay:%.3f,az:%.3f\n", ax, ay, az);
      break;
    case PLOT_G:
      Serial.printf("gx:%.2f,gy:%.2f,gz:%.2f\n", gx, gy, gz);
      break;
    case RAW:
      Serial.printf("raw  a %6d %6d %6d | t %6d | g %6d %6d %6d\n",
                    raw[0], raw[1], raw[2], raw[3], raw[4], raw[5], raw[6]);
      break;
    default:
      break;
  }
}

void ayuda() {
  Serial.println();
  Serial.println("=== bench_imu · Risky ===");
  Serial.println("  SUP          inclinacion y superficie, como el firmware (modo inicial)");
  Serial.println("  DATOS        acelerometro [g], giroscopio [°/s] y temperatura");
  Serial.println("  ANG          roll, pitch (filtro complementario) y yaw (solo giro, deriva)");
  Serial.println("  PLOT         roll/pitch/yaw para el Serial Plotter (PLOT A o PLOT G para acel/giro)");
  Serial.println("  RAW          valores crudos de los registros");
  Serial.println("  STOP         deja de imprimir (cualquier modo lo reanuda)");
  Serial.println("  HZ 20        frecuencia de impresion (1..100)");
  Serial.println("  CAL          calibra el giroscopio (robot quieto)");
  Serial.println("  CAL A        calibra giroscopio y acelerometro (robot quieto, plano, Z arriba)");
  Serial.println("  CERO         reinicia los angulos y el yaw");
  Serial.println("  VIB 3        media y desviacion durante 3 s (vibracion)");
  Serial.println("  RANGO A 4    rango del acelerometro: 2, 4, 8 o 16 g");
  Serial.println("  RANGO G 500  rango del giroscopio: 250, 500, 1000 o 2000 °/s");
  Serial.println("  FILTRO 3     filtro interno 0..6 (260, 184, 94, 44, 21, 10, 5 Hz)");
  Serial.println("  SCAN         escanea el bus I2C (deberian salir 0x68 y 0x29)");
  Serial.println("  INFO         configuracion y conteo de errores");
  Serial.println("  AYUDA        esta lista");
  Serial.println();
}

int buscar(const int* lista, int n, int valor) {
  for (int i = 0; i < n; i++) if (lista[i] == valor) return i;
  return -1;
}

void procesar(String linea) {
  linea.trim();
  linea.toUpperCase();
  if (linea.length() == 0) return;

  char cmd[12] = {0}, t1[12] = {0}, t2[12] = {0};
  int n = sscanf(linea.c_str(), "%11s %11s %11s", cmd, t1, t2);
  String c = cmd;

  if (c == "SUP") modo = SUP;
  else if (c == "DATOS") modo = DATOS;
  else if (c == "ANG") modo = ANG;
  else if (c == "PLOT") modo = (t1[0] == 'A') ? PLOT_A : (t1[0] == 'G') ? PLOT_G : PLOT_ANG;
  else if (c == "RAW") modo = RAW;
  else if (c == "STOP" || c == "S") modo = NADA;
  else if (c == "HZ") {
    if (n < 2) { Serial.printf("Impresion a %.0f Hz\n", hzImpresion); return; }
    hzImpresion = constrain((float)atof(t1), 1.0f, 100.0f);
    Serial.printf("Impresion a %.0f Hz\n", hzImpresion);
  }
  else if (c == "CAL") calibrar(t1[0] == 'A');
  else if (c == "CERO") { angInit = false; yaw = 0; Serial.println("Angulos reiniciados"); }
  else if (c == "VIB") vibracion(n >= 2 ? atof(t1) : 3);
  else if (c == "RANGO") {
    int v = atoi(t2);
    if (t1[0] == 'A') {
      int i = buscar(A_RANGO, 4, v);
      if (i < 0) { Serial.println("Usa RANGO A 2, 4, 8 o 16"); return; }
      aIdx = i;
    } else if (t1[0] == 'G') {
      int i = buscar(G_RANGO, 4, v);
      if (i < 0) { Serial.println("Usa RANGO G 250, 500, 1000 o 2000"); return; }
      gIdx = i;
    } else { Serial.println("Usa RANGO A <g> o RANGO G <°/s>"); return; }
    imuOk = configurarImu();
  }
  else if (c == "FILTRO") {
    int v = atoi(t1);
    if (n < 2 || v < 0 || v > 6) { Serial.println("Usa FILTRO 0..6"); return; }
    dlpf = v;
    imuOk = configurarImu();
  }
  else if (c == "SCAN") escanear();
  else if (c == "INFO") {
    Serial.printf("IMU %s · ±%d g · ±%d °/s · filtro %d Hz · errores I2C: %lu\n",
                  imuOk ? "OK" : "SIN CONEXION", A_RANGO[aIdx], G_RANGO[gIdx], DLPF_HZ[dlpf], errores);
  }
  else if (c == "AYUDA" || c == "H" || c == "?") ayuda();
  else { Serial.println("Comando no reconocido. Escribe AYUDA."); return; }

  tImpresion = millis();
}

void leerSerie() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\n' || ch == '\r') {
      if (buffer.length()) {
        String linea = buffer;
        buffer = "";
        procesar(linea);
      }
    } else if (buffer.length() < 40) {
      buffer += ch;
    }
  }
}

void setup() {
  for (int p : PINES_APAGADOS) {
    pinMode(p, OUTPUT);
    digitalWrite(p, LOW);
  }

  Serial.begin(115200);
  delay(300);
  Wire.begin(PIN_SDA, PIN_SCL, 100000);

  ayuda();
  escanear();
  imuOk = configurarImu();
  if (imuOk) calibrar(false);
  tMuestra = micros();
}

void loop() {
  leerSerie();

  unsigned long ahora = micros();
  if (ahora - tMuestra >= PERIODO_US) {
    float dt = (ahora - tMuestra) / 1e6f;
    tMuestra = ahora;
    if (imuOk) {
      if (leerImu()) {
        fallosSeguidos = 0;
        actualizarAngulos(dt);
      } else if (++fallosSeguidos > 20) {
        imuOk = false;
        Serial.println("Se perdio la comunicacion con el IMU, reintentando...");
      }
    }
  }

  if (!imuOk && millis() - tReintento > 2000) {
    tReintento = millis();
    imuOk = configurarImu();
  }

  if (imuOk && modo != NADA && millis() - tImpresion >= 1000 / hzImpresion) {
    tImpresion = millis();
    imprimir();
  }
}