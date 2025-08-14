/********* Survivor Detection System (ESP32) - No Cooldown *********
 * Sensors: PIR (GPIO 27), MPU6050 (I2C 21/22), INMP441 I2S mic (25/26/33), GPS (UART2: RX2=16, TX2=17)
 * Action: On 2-of-3 confirmations within a time window, send Telegram message + location
 *******************************************************************/

#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClientSecure.h>

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

#include <driver/i2s.h>
#include <TinyGPSPlus.h>

// ========= USER CONFIG =========
const char* WIFI_SSID     = "realme 10";
const char* WIFI_PASSWORD = "niki556666";

const String BOT_TOKEN = "8171742879:AAH_VGpG_87YM0GTa7mD5NOb6q5v9KnyQrU";
const String CHAT_ID   = "5949148312";

// ========= PINS =========
#define PIR_PIN         27

// I2C for MPU6050
#define I2C_SDA         21
#define I2C_SCL         22

// I2S for INMP441
#define I2S_WS          25   // LRCL / WS
#define I2S_SCK         26   // BCLK
#define I2S_SD          33   // DOUT from mic

// GPS UART2
#define GPS_RX          16   // ESP32 RX2  <- GPS TX
#define GPS_TX          17   // ESP32 TX2  -> GPS RX

// ========= DETECTION TUNABLES =========
// Time window within which 2 of 3 cues must occur (ms)
#define CONFIRM_WINDOW_MS   10000UL

// PIR: needs just a HIGH
// MPU6050: accel magnitude (m/s^2) spike
#define VIB_THRESHOLD        3.5f      // try 2.0–6.0; depends on mounting

// INMP441 audio: RMS threshold
#define MIC_SAMPLE_RATE      16000
#define MIC_READ_SAMPLES     2048      // power-of-two
#define MIC_RMS_THRESHOLD    150000.0  // adjust after Serial tuning

// GPS fix wait time
#define GPS_FIX_TIMEOUT_MS   15000UL

// ========= GLOBALS =========
Adafruit_MPU6050 mpu;
TinyGPSPlus gps;
HardwareSerial SerialGPS(1);

bool pirFlag = false;
bool vibFlag = false;
bool voiceFlag = false;

unsigned long firstCueTime = 0;

float lastLat = 0.0, lastLng = 0.0;
bool haveLastFix = false;

// ========= I2S SETUP =========
void i2s_install() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = MIC_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    .dma_buf_len = 512,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

double readMicRMS() {
  const size_t bytesToRead = MIC_READ_SAMPLES * sizeof(int32_t);
  static int32_t samples[MIC_READ_SAMPLES];

  size_t bytesRead = 0;
  esp_err_t res = i2s_read(I2S_NUM_0, (void*)samples, bytesToRead, &bytesRead, 50 / portTICK_PERIOD_MS);
  if (res != ESP_OK || bytesRead == 0) return 0.0;

  double acc = 0.0;
  int n = bytesRead / sizeof(int32_t);
  for (int i = 0; i < n; i++) {
    int32_t s = samples[i] >> 8;
    acc += (double)s * (double)s;
  }
  if (n == 0) return 0.0;
  return sqrt(acc / n);
}

// ========= WIFI & TELEGRAM =========
void connectWiFi() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("WiFi: connecting");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 15000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println();
  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("WiFi OK: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("WiFi failed (continuing; will retry before sending).");
  }
}

bool telegramGET(const String& url) {
  HTTPClient http;
  WiFiClientSecure client;
  client.setInsecure();
  if (!http.begin(client, url)) {
    Serial.println("HTTP begin failed");
    return false;
  }
  int code = http.GET();
  Serial.printf("Telegram HTTP %d\n", code);
  http.end();
  return (code > 0 && code < 400);
}

bool sendTelegramLocation(double lat, double lon) {
  String url = "https://api.telegram.org/bot" + BOT_TOKEN +
               "/sendLocation?chat_id=" + CHAT_ID +
               "&latitude=" + String(lat, 6) +
               "&longitude=" + String(lon, 6);
  return telegramGET(url);
}

bool sendTelegramMessage(const String& text) {
  String url = "https://api.telegram.org/bot" + BOT_TOKEN +
               "/sendMessage?chat_id=" + CHAT_ID +
               "&text=" + urlencode(text);
  return telegramGET(url);
}

String urlencode(const String &s) {
  String out;
  char buf[5];
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
      out += c;
    } else if (c == ' ') {
      out += "%20";
    } else {
      snprintf(buf, sizeof(buf), "%%%02X", (unsigned char)c);
      out += buf;
    }
  }
  return out;
}

// ========= GPS =========
bool getGPSFix(double &lat, double &lng) {
  unsigned long start = millis();
  bool gotFix = false;
  while (millis() - start < GPS_FIX_TIMEOUT_MS) {
    while (SerialGPS.available()) {
      char c = (char)SerialGPS.read();
      gps.encode(c);
    }
    if (gps.location.isUpdated() && gps.location.isValid()) {
      lat = gps.location.lat();
      lng = gps.location.lng();
      gotFix = true;
      break;
    }
  }
  return gotFix;
}

// ========= SETUP =========
void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(PIR_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!mpu.begin()) {
    Serial.println("MPU6050 NOT FOUND");
  } else {
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    Serial.println("MPU6050 OK");
  }

  i2s_install();

  SerialGPS.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS serial started");

  connectWiFi();
}

// ========= HELPERS =========
bool checkPIR() {
  return digitalRead(PIR_PIN) == HIGH;
}

bool checkVibration() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  float mag = sqrtf(a.acceleration.x * a.acceleration.x +
                    a.acceleration.y * a.acceleration.y +
                    a.acceleration.z * a.acceleration.z);
  float dynamic = fabs(mag - 9.81f);
  return (dynamic > VIB_THRESHOLD);
}

bool checkVoice() {
  double rms = readMicRMS();
  return (rms > MIC_RMS_THRESHOLD);
}

void resetCueWindow() {
  pirFlag = vibFlag = voiceFlag = false;
  firstCueTime = 0;
}

int countCues() {
  return (pirFlag ? 1 : 0) + (vibFlag ? 1 : 0) + (voiceFlag ? 1 : 0);
}

// ========= MAIN LOOP =========
void loop() {
  while (SerialGPS.available()) {
    gps.encode(SerialGPS.read());
    if (gps.location.isValid()) {
      haveLastFix = true;
      lastLat = gps.location.lat();
      lastLng = gps.location.lng();
    }
  }

  bool pir = checkPIR();
  bool vib = checkVibration();
  bool vio = checkVoice();

  if (pir || vib || vio) {
    if (firstCueTime == 0) firstCueTime = millis();
    if (pir)  pirFlag  = true;
    if (vib)  vibFlag  = true;
    if (vio)  voiceFlag = true;
  }

  if (firstCueTime != 0) {
    if (millis() - firstCueTime > CONFIRM_WINDOW_MS) {
      resetCueWindow();
    } else {
      if (countCues() >= 2) {
        Serial.println(">>> HUMAN DETECTED (2-of-3) <<<");

        if (WiFi.status() != WL_CONNECTED) connectWiFi();

        double lat = 0.0, lng = 0.0;
        bool fresh = getGPSFix(lat, lng);
        if (!fresh && haveLastFix) {
          lat = lastLat; lng = lastLng;
        }

        String summary = "ALERT: Possible survivor detected.\n"
                         "Cues: PIR=" + String(pirFlag) + ", VIB=" + String(vibFlag) + ", VOICE=" + String(voiceFlag);
        sendTelegramMessage(summary);

        if (fresh || haveLastFix) {
          sendTelegramLocation(lat, lng);
          sendTelegramMessage(String("Location: ") + String(lat, 6) + ", " + String(lng, 6));
        } else {
          sendTelegramMessage("Location unavailable (no GPS fix yet).");
        }

        resetCueWindow();
      }
    }
  }

  delay(30);
}
