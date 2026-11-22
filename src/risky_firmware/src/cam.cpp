// ESP32-CAM: video MJPEG por HTTP. El nodo camera_bridge del PC lo publica en ROS 2.
//   http://<IP>/              página de prueba con video y control del flash
//   http://<IP>:81/stream     MJPEG
//   http://<IP>/capture       una foto
//   http://<IP>/led?level=0..255
#include <Arduino.h>
#include <ESPmDNS.h>
#include <WiFi.h>
#include <esp_camera.h>
#include <esp_http_server.h>
#include <lwip/sockets.h>

#include <initializer_list>

#include "secrets.h"
#include "util.h"

namespace {

constexpr char kHostname[] = "risky-cam";
constexpr int kFlashPin = 4;
// Canal 7 usa el timer 3; el XCLK de la cámara ocupa el canal 0 y el timer 0.
constexpr uint8_t kFlashLedc = 7;

constexpr uint32_t kStatsPeriodMs = 2000;

constexpr char kStreamType[] = "multipart/x-mixed-replace;boundary=frame";
constexpr char kPartHeader[] = "\r\n--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// Delimitador "html" porque el onchange contiene )" y cerraría un raw string normal.
constexpr char kIndexHtml[] = R"html(<!doctype html><meta name=viewport content="width=device-width">
<title>Risky cam</title><body style="margin:0;background:#111;color:#eee;font-family:sans-serif">
<img id=v style="width:100%"><p style="padding:8px">Flash <input type=range max=255 value=0 onchange="fetch('/led?level='+this.value)"></p>
<script>v.src=`http://${location.hostname}:81/stream`</script>)html";

// Pines fijos de la placa AI-Thinker.
camera_config_t cameraConfig() {
  camera_config_t c{};
  c.pin_pwdn = 32;
  c.pin_reset = -1;
  c.pin_xclk = 0;
  c.pin_sccb_sda = 26;
  c.pin_sccb_scl = 27;
  c.pin_d7 = 35;
  c.pin_d6 = 34;
  c.pin_d5 = 39;
  c.pin_d4 = 36;
  c.pin_d3 = 21;
  c.pin_d2 = 19;
  c.pin_d1 = 18;
  c.pin_d0 = 5;
  c.pin_vsync = 25;
  c.pin_href = 23;
  c.pin_pclk = 22;
  c.xclk_freq_hz = 20000000;
  c.ledc_timer = LEDC_TIMER_0;
  c.ledc_channel = LEDC_CHANNEL_0;
  c.pixel_format = PIXFORMAT_JPEG;
  c.frame_size = FRAMESIZE_VGA;
  c.jpeg_quality = 12;
  c.fb_count = 2;
  c.fb_location = CAMERA_FB_IN_PSRAM;
  c.grab_mode = CAMERA_GRAB_LATEST;
  return c;
}

bool queryValue(httpd_req_t* req, const char* key, char* value, size_t size) {
  char query[48];
  return httpd_req_get_url_query_str(req, query, sizeof query) == ESP_OK &&
         httpd_query_key_value(query, key, value, size) == ESP_OK;
}

esp_err_t indexHandler(httpd_req_t* req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t captureHandler(httpd_req_t* req) {
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) return httpd_resp_send_500(req);
  httpd_resp_set_type(req, "image/jpeg");
  esp_err_t err = httpd_resp_send(req, reinterpret_cast<const char*>(fb->buf), fb->len);
  esp_camera_fb_return(fb);
  return err;
}

esp_err_t ledHandler(httpd_req_t* req) {
  char value[8];
  if (!queryValue(req, "level", value, sizeof value)) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "uso: /led?level=0..255");
    return ESP_FAIL;
  }
  ledcWrite(kFlashLedc, constrain(atoi(value), 0, 255));
  return httpd_resp_send(req, nullptr, 0);
}

// Bloquea su servidor mientras el cliente mire; por eso va en un puerto aparte.
esp_err_t streamHandler(httpd_req_t* req) {
  // Sin esto, Nagle retiene el encabezado corto de cada cuadro esperando el ACK anterior.
  int noDelay = 1;
  setsockopt(httpd_req_to_sockfd(req), IPPROTO_TCP, TCP_NODELAY, &noDelay, sizeof noDelay);

  httpd_resp_set_type(req, kStreamType);
  char header[96];
  uint32_t frames = 0;
  uint32_t bytes = 0;
  uint32_t waitMs = 0;  // esperando a la cámara
  uint32_t sendMs = 0;  // esperando a la red
  uint32_t statsStart = millis();
  while (true) {
    uint32_t t0 = millis();
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) return ESP_FAIL;
    uint32_t t1 = millis();
    int n = snprintf(header, sizeof header, kPartHeader, static_cast<unsigned>(fb->len));
    esp_err_t err = httpd_resp_send_chunk(req, header, n);
    if (err == ESP_OK) {
      err = httpd_resp_send_chunk(req, reinterpret_cast<const char*>(fb->buf), fb->len);
    }
    bytes += fb->len;
    esp_camera_fb_return(fb);
    if (err != ESP_OK) return err;  // el cliente cerró la conexión

    ++frames;
    waitMs += t1 - t0;
    sendMs += millis() - t1;
    uint32_t elapsed = millis() - statsStart;
    if (elapsed >= kStatsPeriodMs) {
      Serial.printf("%.1f fps  %u KB/cuadro  %.0f KB/s  camara %u ms  red %u ms\n", frames * 1000.f / elapsed,
                    static_cast<unsigned>(bytes / frames / 1024), bytes / 1.024f / elapsed,
                    static_cast<unsigned>(waitMs / frames), static_cast<unsigned>(sendMs / frames));
      frames = bytes = waitMs = sendMs = 0;
      statsStart = millis();
    }
  }
}

httpd_uri_t route(const char* path, esp_err_t (*handler)(httpd_req_t*)) {
  httpd_uri_t r{};
  r.uri = path;
  r.method = HTTP_GET;
  r.handler = handler;
  return r;
}

httpd_handle_t startServer(uint16_t port, std::initializer_list<httpd_uri_t> uris) {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = port;
  config.ctrl_port = port;  // puerto UDP interno; dos servidores no pueden compartirlo
  httpd_handle_t server = nullptr;
  if (httpd_start(&server, &config) != ESP_OK) halt("No se pudo iniciar el servidor HTTP.");
  for (const httpd_uri_t& uri : uris) httpd_register_uri_handler(server, &uri);
  return server;
}

}  // namespace

void setup() {
  Serial.begin(115200);

  ledcSetup(kFlashLedc, 5000, 8);
  ledcAttachPin(kFlashPin, kFlashLedc);
  ledcWrite(kFlashLedc, 0);

  if (!psramFound()) halt("Sin PSRAM: la placa no es una AI-Thinker completa.");
  camera_config_t config = cameraConfig();
  if (esp_camera_init(&config) != ESP_OK) halt("La cámara no responde: revise el flex del sensor.");
  Serial.println("Cámara lista.");

  WiFi.setHostname(kHostname);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);  // el ahorro de energía del WiFi corta el video a tirones
  WiFi.begin(secrets::kWifiSsid, secrets::kWifiPassword);
  Serial.printf("Conectando a la red %s", secrets::kWifiSsid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print('.');
  }
  Serial.printf("\nWiFi conectado: %s  (señal %d dBm)\n", WiFi.localIP().toString().c_str(), WiFi.RSSI());

  if (MDNS.begin(kHostname)) MDNS.addService("http", "tcp", 80);

  startServer(80, {route("/", indexHandler), route("/capture", captureHandler), route("/led", ledHandler)});
  startServer(81, {route("/stream", streamHandler)});

  Serial.printf("Abra http://%s/ o http://%s.local/\n", WiFi.localIP().toString().c_str(), kHostname);
}

void loop() {
  static bool connected = true;
  if (WiFi.isConnected() != connected) {
    connected = !connected;
    if (connected) {
      Serial.printf("WiFi reconectado: %s\n", WiFi.localIP().toString().c_str());
    } else {
      Serial.println("WiFi perdido.");
    }
  }
  delay(500);
}
