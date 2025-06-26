#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "driver/i2s.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

// WiFi credentials
const char* ssid = "Kawcher";
const char* password = "01970916952";

// Server info
const char* serverHost = "192.168.0.105";  // IP or domain of your server
const int serverPort = 80;
const char* serverPath = "/upload";

const int buttonPin = 15;  // Changed from 0 to 4 (adjust to your wiring)
const int RECORD_SECONDS = 7;
const int SAMPLE_RATE = 16000;
const int I2S_READ_LEN = 1024;

#define I2S_WS   42
#define I2S_SD   2
#define I2S_SCK  41
#define I2S_PORT I2S_NUM_0

std::vector<uint8_t> audioData;

// WAV header struct for 16-bit mono PCM WAV
struct WAVHeader {
  char riff[4] = {'R', 'I', 'F', 'F'};
  uint32_t chunkSize;
  char wave[4] = {'W', 'A', 'V', 'E'};
  char fmt[4] = {'f', 'm', 't', ' '};
  uint32_t subChunk1Size = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels = 1;
  uint32_t sampleRate = SAMPLE_RATE;
  uint32_t byteRate;
  uint16_t blockAlign;
  uint16_t bitsPerSample = 16;
  char subChunk2ID[4] = {'d','a','t','a'};
  uint32_t subChunk2Size;
};

void fillWavHeader(WAVHeader &header, uint32_t dataSize) {
  header.subChunk2Size = dataSize;
  header.chunkSize = 36 + dataSize;
  header.byteRate = header.sampleRate * header.numChannels * header.bitsPerSample / 8;
  header.blockAlign = header.numChannels * header.bitsPerSample / 8;
}

void connectWiFi() {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.print("Connecting to WiFi");
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
  }
}

void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 1024,
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

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
  i2s_zero_dma_buffer(I2S_PORT);
}

void recordAudio() {
  Serial.println("Recording audio...");
  size_t bytes_read;
  uint8_t buffer[I2S_READ_LEN];
  unsigned long start = millis();
  audioData.clear();

  while (millis() - start < RECORD_SECONDS * 1000) {
    esp_err_t result = i2s_read(I2S_PORT, &buffer, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    if (result == ESP_OK && bytes_read > 0) {
      audioData.insert(audioData.end(), buffer, buffer + bytes_read);
    } else {
      Serial.printf("I2S read failed or returned zero bytes. err=%d\n", result);
    }
  }

  Serial.printf("Recording complete. Collected %d bytes.\n", audioData.size());
}

camera_fb_t* captureImage() {
  Serial.println("Capturing image...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
  } else {
    Serial.printf("Image captured: %d bytes\n", fb->len);
  }
  return fb;
}

void sendToServer(camera_fb_t* fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected!");
    return;
  }

  WiFiClient client;
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("Connection to server failed");
    return;
  }

  String boundary = "----ESP32FormBoundary";
  String contentType = "multipart/form-data; boundary=" + boundary;

  String startRequest = "--" + boundary + "\r\n"
                        "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
                        "Content-Type: image/jpeg\r\n\r\n";

  String midRequest = "";
  uint8_t wavHeaderBuffer[44];
  if (!audioData.empty()) {
    WAVHeader header;
    fillWavHeader(header, audioData.size());
    memcpy(wavHeaderBuffer, &header, sizeof(WAVHeader));

    midRequest = "\r\n--" + boundary + "\r\n"
                 "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n"
                 "Content-Type: audio/wav\r\n\r\n";
  }

  String endRequest = "\r\n--" + boundary + "--\r\n";

  int totalLen = startRequest.length() + fb->len + midRequest.length() + (audioData.empty() ? 0 : (44 + audioData.size())) + endRequest.length();

  // HTTP headers
  String httpRequest = String("POST ") + serverPath + " HTTP/1.1\r\n" +
                       "Host: " + serverHost + "\r\n" +
                       "Content-Type: " + contentType + "\r\n" +
                       "Content-Length: " + String(totalLen) + "\r\n" +
                       "Connection: close\r\n\r\n";

  Serial.println("Sending HTTP request...");
  client.print(httpRequest);
  client.print(startRequest);
  client.write(fb->buf, fb->len);

  if (!audioData.empty()) {
    client.print(midRequest);
    client.write(wavHeaderBuffer, 44);         // WAV header
    client.write(audioData.data(), audioData.size());  // PCM data
  }

  client.print(endRequest);

  Serial.println("Data sent, waiting for server response...");

  unsigned long timeout = millis();
  while (client.connected() && millis() - timeout < 5000) {
    while (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
      timeout = millis();
    }
  }

  client.stop();
  Serial.println("Connection closed.");
}

void setup() {
  Serial.begin(115200);
  pinMode(buttonPin, INPUT_PULLUP);

  Serial.println("Initializing camera...");
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_LATEST;

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1) delay(1000);
  }

  connectWiFi();
  setupI2SMic();
  Serial.println("Setup complete.");
}

void loop() {
  static bool lastButtonState = HIGH;
  static unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 50;

  int reading = digitalRead(buttonPin);

  if (reading != lastButtonState) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading == LOW && lastButtonState == HIGH) {
      Serial.println("Button pressed!");

      connectWiFi();
      camera_fb_t* fb = captureImage();
      if (fb) {
        recordAudio();
        sendToServer(fb);
        esp_camera_fb_return(fb);
      }
    }
  }

  lastButtonState = reading;
}

