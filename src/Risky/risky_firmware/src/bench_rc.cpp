// Banco 1: receptor FS-R6B. Muestra el ancho de cada canal y los extremos vistos para calibrar config.h.
#include <Arduino.h>

#include "RcInput.h"
#include "config.h"
#include "util.h"

namespace {
struct Channel {
  const char* name;
  uint8_t pin;
  RcChannel input;
  uint16_t minSeen = UINT16_MAX;
  uint16_t maxSeen = 0;
};

Channel channels[] = {
    {"CH1", pin::kRcCh1, {}},
    {"CH2", pin::kRcCh2, {}},
    {"CH5", pin::kRcCh5, {}},
    {"CH6", pin::kRcCh6, {}},
};
Periodic report(100);
}  // namespace

void setup() {
  Serial.begin(115200);
  for (Channel& ch : channels) ch.input.begin(ch.pin);
  Serial.println("bench_rc: ancho actual [mínimo-máximo vistos] en us; ---- = sin señal");
}

void loop() {
  if (!report.due(millis())) return;
  for (Channel& ch : channels) {
    uint16_t us;
    if (ch.input.read(us, rc::kTimeoutUs)) {
      ch.minSeen = min(ch.minSeen, us);
      ch.maxSeen = max(ch.maxSeen, us);
      Serial.printf("%s %4u [%4u-%4u]   ", ch.name, us, ch.minSeen, ch.maxSeen);
    } else {
      Serial.printf("%s ----               ", ch.name);
    }
  }
  Serial.println();
}
