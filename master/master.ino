#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_timer.h>
#include <stdint.h>

#define WIFI_CHANNEL 6
#define IMU_COUNT 4
#define USB_SERIAL_BAUD 921600

uint8_t BROADCAST_MAC[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t IMU1[] = {0xF8, 0xB3, 0xB7, 0x51, 0xB9, 0x94};
uint8_t IMU2[] = {0x08, 0xF9, 0xE0, 0x88, 0x5D, 0x14};
uint8_t IMU3[] = {0xF8, 0xB3, 0xB7, 0x52, 0x7B, 0xDC};
uint8_t IMU4[] = {0xF8, 0xB3, 0xB7, 0x50, 0xDE, 0xC8};
uint8_t EMG_SLAVE[] = {0x8C, 0x4F, 0x00, 0x3C, 0x78, 0xC9};

#pragma pack(push, 1)
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

struct EmgData {
  float emg_clean;
  uint32_t timestamp_us;
  uint8_t sender_id;
};

struct TriggerMessage {
  uint32_t seq;
  uint64_t master_ts_us;
};
#pragma pack(pop)

struct QueuedSensorData {
  SensorData sample;
  uint64_t master_rx_ts_us;
};

const uint8_t RX_QUEUE_SIZE = 64;
QueuedSensorData rxQueue[RX_QUEUE_SIZE];
volatile uint8_t rxHead = 0;
volatile uint8_t rxTail = 0;
volatile uint32_t rxOverflowCount = 0;
portMUX_TYPE rxMux = portMUX_INITIALIZER_UNLOCKED;

const uint32_t IMU_PERIOD_US = 20000;       // 50 Hz
const uint32_t EMG_PERIOD_US = 10000;       // 100 Hz
const bool ENABLE_EMG_TRIGGER = false;
const bool ENABLE_HEARTBEAT = true;

uint64_t lastImuRoundUs = 0;
uint64_t lastEmgTrigUs = 0;
uint64_t lastHeartbeatUs = 0;
uint32_t imuSeq = 0;
volatile uint32_t rxPacketCount = 0;
uint32_t triggerSendFailCount = 0;

void addPeer(const uint8_t *macAddr) {
  if (!esp_now_is_peer_exist(macAddr)) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, macAddr, 6);
    peerInfo.channel = WIFI_CHANNEL;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
  }
}

void onDataReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len != (int)sizeof(SensorData)) return;

  const SensorData *s = reinterpret_cast<const SensorData *>(data);
  uint8_t nextHead = (rxHead + 1) % RX_QUEUE_SIZE;

  portENTER_CRITICAL_ISR(&rxMux);
  if (nextHead == rxTail) {
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    rxOverflowCount++;
  }

  rxQueue[rxHead].sample = *s;
  rxQueue[rxHead].master_rx_ts_us = esp_timer_get_time();
  rxHead = nextHead;
  rxPacketCount++;
  portEXIT_CRITICAL_ISR(&rxMux);
}

bool popSensorData(QueuedSensorData *out) {
  bool ok = false;

  portENTER_CRITICAL(&rxMux);
  if (rxTail != rxHead) {
    out->sample = rxQueue[rxTail].sample;
    out->master_rx_ts_us = rxQueue[rxTail].master_rx_ts_us;
    rxTail = (rxTail + 1) % RX_QUEUE_SIZE;
    ok = true;
  }
  portEXIT_CRITICAL(&rxMux);

  return ok;
}

void printSensorData(const QueuedSensorData &pkt) {
  const SensorData &s = pkt.sample;
  uint32_t airUs = (uint32_t)(pkt.master_rx_ts_us - s.master_ts_us);

  Serial.printf("IMU,%u,%lu,%.4f,%.4f,%.4f,%.4f,%llu,%llu,%llu,%lu,%lu,%lu,%lu\n",
                s.sender_id,
                (unsigned long)s.master_seq,
                s.q_w, s.q_x, s.q_y, s.q_z,
                (unsigned long long)s.master_ts_us,
                (unsigned long long)s.slave_local_ts_us,
                (unsigned long long)s.pico_ts_us,
                (unsigned long)s.pico_sample_seq,
                (unsigned long)s.slave_send_seq,
                (unsigned long)s.trigger_to_send_us,
                (unsigned long)airUs);
}

void sendImuTrigger(uint64_t nowUs) {
  TriggerMessage trig;
  trig.seq = imuSeq++;
  trig.master_ts_us = nowUs;
  esp_err_t result = esp_now_send(BROADCAST_MAC, (uint8_t *)&trig, sizeof(trig));
  if (result != ESP_OK) triggerSendFailCount++;
}

void setup() {
  Serial.begin(USB_SERIAL_BAUD);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);
  esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP_NOW_INIT_FAILED");
    while (true) delay(1000);
  }

  esp_now_register_recv_cb(onDataReceive);

  addPeer(BROADCAST_MAC);
  addPeer(IMU1);
  addPeer(IMU2);
  addPeer(IMU3);
  addPeer(IMU4);
  addPeer(EMG_SLAVE);

  Serial.printf("MASTER_READY,channel=%u,serial=%u\n", WIFI_CHANNEL, USB_SERIAL_BAUD);
  Serial.printf("MASTER_MAC,%s\n", WiFi.macAddress().c_str());
  Serial.println("type,id,master_seq,q_w,q_x,q_y,q_z,master_ts_us,slave_ts_us,pico_ts_us,pico_sample_seq,slave_send_seq,trigger_to_send_us,air_us");
}

void loop() {
  uint64_t nowUs = esp_timer_get_time();

  if (lastImuRoundUs == 0) {
    sendImuTrigger(nowUs);
    lastImuRoundUs = nowUs;
  } else if (nowUs - lastImuRoundUs >= IMU_PERIOD_US) {
    sendImuTrigger(nowUs);
    lastImuRoundUs += IMU_PERIOD_US;
    if (nowUs - lastImuRoundUs >= IMU_PERIOD_US) {
      lastImuRoundUs = nowUs;
    }
  }

  if (ENABLE_EMG_TRIGGER && nowUs - lastEmgTrigUs >= EMG_PERIOD_US) {
    lastEmgTrigUs = nowUs;
    uint8_t simpleTrig = 0x01;
    esp_now_send(EMG_SLAVE, &simpleTrig, 1);
  }

  if (ENABLE_HEARTBEAT && nowUs - lastHeartbeatUs >= 2000000ULL) {
    lastHeartbeatUs = nowUs;
    Serial.printf("MASTER_STATUS,trig=%lu,trig_fail=%lu,rx=%lu,rx_overflow=%lu\n",
                  (unsigned long)imuSeq,
                  (unsigned long)triggerSendFailCount,
                  (unsigned long)rxPacketCount,
                  (unsigned long)rxOverflowCount);
  }

  QueuedSensorData pkt;
  while (popSensorData(&pkt)) {
    printSensorData(pkt);
  }
}
