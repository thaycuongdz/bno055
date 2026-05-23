#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>

#define RX_PIN 16
#define TX_PIN 17
#define UART_BAUD 115200
#define WIFI_CHANNEL 6
#define RESPONSE_BASE_US 1200
#define RESPONSE_SLOT_US 2500

// Fill with master ESP32 MAC.
uint8_t masterMacAddress[] = {0xEC, 0x64, 0xC9, 0x90, 0xEC, 0x34};

// Change this per slave: 1, 2, 3, or 4.
#define MY_SENDER_ID 2

#pragma pack(push, 1)
struct TriggerMessage {
  uint32_t seq;
  uint64_t master_ts_us;
};

struct SensorData {
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  uint64_t pico_ts_us;
  uint64_t slave_local_ts_us;
  uint32_t master_seq;
  uint64_t master_ts_us;
  uint32_t pico_sample_seq;
  uint32_t slave_send_seq;
  uint32_t trigger_to_send_us;
  uint8_t sender_id;
};
#pragma pack(pop)

struct PicoSample {
  float q_w;
  float q_x;
  float q_y;
  float q_z;
  uint64_t pico_ts_us;
  uint32_t sample_seq;
};

struct QueuedTrigger {
  uint32_t seq;
  uint64_t master_ts_us;
  uint64_t slave_rx_ts_us;
};

const uint8_t MAX_PENDING_TRIGGERS = 8;

QueuedTrigger triggerQueue[MAX_PENDING_TRIGGERS];
volatile uint8_t triggerHead = 0;
volatile uint8_t triggerTail = 0;
volatile uint32_t droppedTriggers = 0;
volatile uint32_t triggerRxCount = 0;
portMUX_TYPE triggerMux = portMUX_INITIALIZER_UNLOCKED;

PicoSample latestSample = {};
bool hasSample = false;
uint32_t slaveSendSeq = 0;
uint32_t picoSampleCount = 0;
uint32_t noSampleTriggerCount = 0;
volatile uint32_t sendOkCount = 0;
volatile uint32_t sendFailCount = 0;

char uartLine[96];
size_t uartLineLen = 0;
uint64_t lastStatusUs = 0;

void onDataSent(const esp_now_send_info_t *info, esp_now_send_status_t status) {
  if (status != ESP_NOW_SEND_SUCCESS) {
    sendFailCount++;
  } else {
    sendOkCount++;
  }
}

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(TriggerMessage)) return;

  const TriggerMessage *t = reinterpret_cast<const TriggerMessage *>(data);
  uint8_t nextHead = (triggerHead + 1) % MAX_PENDING_TRIGGERS;

  portENTER_CRITICAL_ISR(&triggerMux);
  if (nextHead == triggerTail) {
    triggerTail = (triggerTail + 1) % MAX_PENDING_TRIGGERS;
    droppedTriggers++;
  }

  triggerQueue[triggerHead].seq = t->seq;
  triggerQueue[triggerHead].master_ts_us = t->master_ts_us;
  triggerQueue[triggerHead].slave_rx_ts_us = esp_timer_get_time();
  triggerHead = nextHead;
  triggerRxCount++;
  portEXIT_CRITICAL_ISR(&triggerMux);
}

bool popTrigger(QueuedTrigger *out) {
  bool ok = false;

  portENTER_CRITICAL(&triggerMux);
  if (triggerTail != triggerHead) {
    out->seq = triggerQueue[triggerTail].seq;
    out->master_ts_us = triggerQueue[triggerTail].master_ts_us;
    out->slave_rx_ts_us = triggerQueue[triggerTail].slave_rx_ts_us;
    triggerTail = (triggerTail + 1) % MAX_PENDING_TRIGGERS;
    ok = true;
  }
  portEXIT_CRITICAL(&triggerMux);

  return ok;
}

void parsePicoLine(const char *line) {
  float w, x, y, z;
  unsigned long long picoTsUs = 0;
  unsigned long picoSampleSeq = 0;

  int n = sscanf(line, "%f,%f,%f,%f,%llu,%lu",
                 &w, &x, &y, &z, &picoTsUs, &picoSampleSeq);

  if (n >= 5) {
    latestSample.q_w = w;
    latestSample.q_x = x;
    latestSample.q_y = y;
    latestSample.q_z = z;
    latestSample.pico_ts_us = picoTsUs;
    latestSample.sample_seq = (n == 6) ? (uint32_t)picoSampleSeq : 0;
    hasSample = true;
    picoSampleCount++;
  }
}

void readPicoUart() {
  while (Serial1.available()) {
    char c = (char)Serial1.read();
    if (c == '\r') continue;

    if (c == '\n') {
      uartLine[uartLineLen] = '\0';
      if (uartLineLen > 0) parsePicoLine(uartLine);
      uartLineLen = 0;
      continue;
    }

    if (uartLineLen < sizeof(uartLine) - 1) {
      uartLine[uartLineLen++] = c;
    } else {
      uartLineLen = 0;
    }
  }
}

void sendOneSampleForTrigger(const QueuedTrigger &trig) {
  if (!hasSample) {
    noSampleTriggerCount++;
    return;
  }

  PicoSample smp = latestSample;
  uint8_t slotIndex = (MY_SENDER_ID > 0) ? (MY_SENDER_ID - 1) : 0;
  uint64_t dueUs = trig.slave_rx_ts_us + RESPONSE_BASE_US +
                   ((uint64_t)slotIndex * RESPONSE_SLOT_US);

  while ((int64_t)(esp_timer_get_time() - dueUs) < 0) {
    readPicoUart();
  }

  uint64_t sendTsUs = esp_timer_get_time();

  SensorData pkt;
  pkt.q_w = smp.q_w;
  pkt.q_x = smp.q_x;
  pkt.q_y = smp.q_y;
  pkt.q_z = smp.q_z;
  pkt.pico_ts_us = smp.pico_ts_us;
  pkt.slave_local_ts_us = sendTsUs;
  pkt.master_seq = trig.seq;
  pkt.master_ts_us = trig.master_ts_us;
  pkt.pico_sample_seq = smp.sample_seq;
  pkt.slave_send_seq = slaveSendSeq++;
  pkt.trigger_to_send_us = (uint32_t)(sendTsUs - trig.slave_rx_ts_us);
  pkt.sender_id = MY_SENDER_ID;

  esp_err_t result = esp_now_send(masterMacAddress, (uint8_t *)&pkt, sizeof(pkt));
  if (result != ESP_OK) sendFailCount++;
}

void printStatus() {
  uint64_t nowUs = esp_timer_get_time();
  if (nowUs - lastStatusUs < 2000000ULL) return;
  lastStatusUs = nowUs;

  Serial.printf("SLAVE_STATUS,id=%u,trig=%lu,pico=%lu,send_ok=%lu,send_fail=%lu,no_sample=%lu,drop_trig=%lu\n",
                MY_SENDER_ID,
                (unsigned long)triggerRxCount,
                (unsigned long)picoSampleCount,
                (unsigned long)sendOkCount,
                (unsigned long)sendFailCount,
                (unsigned long)noSampleTriggerCount,
                (unsigned long)droppedTriggers);
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(UART_BAUD, SERIAL_8N1, RX_PIN, TX_PIN);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP_NOW_INIT_FAILED");
    return;
  }

  esp_now_register_recv_cb(onDataReceive);
  esp_now_register_send_cb(onDataSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, masterMacAddress, 6);
  peerInfo.channel = WIFI_CHANNEL;
  peerInfo.encrypt = false;

  if (!esp_now_is_peer_exist(masterMacAddress)) {
    esp_now_add_peer(&peerInfo);
  }

  Serial.printf("ESP32_SLAVE_READY,id=%u,channel=%u,uart=%u\n",
                MY_SENDER_ID, WIFI_CHANNEL, UART_BAUD);
  Serial.printf("SLAVE_MAC,%s\n", WiFi.macAddress().c_str());
  Serial.printf("SLAVE_TARGET_MASTER,%02X:%02X:%02X:%02X:%02X:%02X\n",
                masterMacAddress[0], masterMacAddress[1], masterMacAddress[2],
                masterMacAddress[3], masterMacAddress[4], masterMacAddress[5]);
}

void loop() {
  readPicoUart();

  QueuedTrigger trig;
  while (popTrigger(&trig)) {
    sendOneSampleForTrigger(trig);
  }

  printStatus();
}
