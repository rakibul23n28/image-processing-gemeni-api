#include <WiFi.h>
#include "esp_camera.h"
#include "driver/i2s.h"

// WiFi credentials
const char* ssid = "Kawcher";
const char* password = "01970916952";

// Server details
const char* server_host = "192.168.0.105";
const uint16_t server_port = 5000;
const char* server_path = "/upload";

// I2S pins for INMP441 mic
#define I2S_WS 17
#define I2S_SD 16
#define I2S_SCK 18

#define SAMPLE_RATE 8000
#define RECORD_TIME_SECONDS 2
#define AUDIO_BUFFER_SIZE (SAMPLE_RATE * RECORD_TIME_SECONDS)

// Static audio buffer to avoid malloc
static int16_t audioBuffer[AUDIO_BUFFER_SIZE];

// ESP32-S3-CAM pins (adjust if your board differs)
#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      21
#define SIOD_GPIO_NUM      26
#define SIOC_GPIO_NUM      27

#define Y9_GPIO_NUM        35
#define Y8_GPIO_NUM        34
#define Y7_GPIO_NUM        39
#define Y6_GPIO_NUM        36
#define Y5_GPIO_NUM        19
#define Y4_GPIO_NUM        18
#define Y3_GPIO_NUM         5
#define Y2_GPIO_NUM         4
#define VSYNC_GPIO_NUM     25
#define HREF_GPIO_NUM      23
#define PCLK_GPIO_NUM      22

void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
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

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupCamera() {
  camera_config_t config = {};
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
  config.frame_size = FRAMESIZE_CIF;  // 320x240 for less memory
  config.jpeg_quality = 25;            // lower quality for less memory
  config.fb_count = 1;                 // only one framebuffer (no PSRAM)

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    while (true) delay(1000);
  }
}

void sendMultipart(WiFiClient& client, uint8_t* audioData, size_t audioLen, uint8_t* imageData, size_t imageLen) {
  String boundary = "----ESP32Boundary";
  String contentType = "multipart/form-data; boundary=" + boundary;

  String bodyStart =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n"
    "Content-Type: audio/wav\r\n\r\n";

  String bodyMiddle =
    "\r\n--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";

  String bodyEnd = "\r\n--" + boundary + "--\r\n";

  int contentLength = bodyStart.length() + audioLen + bodyMiddle.length() + imageLen + bodyEnd.length();

  if (!client.connect(server_host, server_port)) {
    Serial.println("Connection to server failed");
    return;
  }

  // Send HTTP request headers
  client.printf("POST %s HTTP/1.1\r\n", server_path);
  client.printf("Host: %s\r\n", server_host);
  client.printf("Content-Type: %s\r\n", contentType.c_str());
  client.printf("Content-Length: %d\r\n", contentLength);
  client.printf("Connection: close\r\n\r\n");

  // Send multipart body
  client.print(bodyStart);
  client.write(audioData, audioLen);
  client.print(bodyMiddle);
  client.write(imageData, imageLen);
  client.print(bodyEnd);

  // Read server response
  while (client.connected() || client.available()) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      Serial.println(line);
    }
  }

  client.stop();
}

void writeWavHeader(uint8_t* buffer, size_t dataSize) {
  memcpy(buffer, "RIFF", 4);
  uint32_t chunkSize = 36 + dataSize;
  memcpy(buffer + 4, &chunkSize, 4);
  memcpy(buffer + 8, "WAVE", 4);
  memcpy(buffer + 12, "fmt ", 4);

  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1; // PCM
  uint16_t numChannels = 1;
  uint32_t sampleRate = SAMPLE_RATE;
  uint32_t byteRate = sampleRate * numChannels * 16 / 8;
  uint16_t blockAlign = numChannels * 16 / 8;
  uint16_t bitsPerSample = 16;

  memcpy(buffer + 16, &subchunk1Size, 4);
  memcpy(buffer + 20, &audioFormat, 2);
  memcpy(buffer + 22, &numChannels, 2);
  memcpy(buffer + 24, &sampleRate, 4);
  memcpy(buffer + 28, &byteRate, 4);
  memcpy(buffer + 32, &blockAlign, 2);
  memcpy(buffer + 34, &bitsPerSample, 2);
  memcpy(buffer + 36, "data", 4);
  memcpy(buffer + 40, &dataSize, 4);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");
  Serial.println(WiFi.localIP());

  setupI2SMic();
  setupCamera();
}

void loop() {
  Serial.println("Capturing image...");
camera_fb_t* fb = esp_camera_fb_get();
if (!fb) {
  Serial.println("Camera capture failed");
  delay(1000);
  return;
}
Serial.printf("Image captured: %zu bytes\n", fb->len);

Serial.println("Recording audio...");
size_t bytesRead = 0;
size_t index = 0;
while (index < AUDIO_BUFFER_SIZE) {
  size_t toRead = (AUDIO_BUFFER_SIZE - index) * sizeof(int16_t);
  if (i2s_read(I2S_NUM_0, (void*)(audioBuffer + index), toRead, &bytesRead, portMAX_DELAY) == ESP_OK) {
    index += bytesRead / sizeof(int16_t);
  }
}
Serial.println("Audio recording done.");

// Prepare WAV buffer
const size_t wavSize = 44 + AUDIO_BUFFER_SIZE * sizeof(int16_t);
uint8_t* wavBuffer = (uint8_t*)malloc(wavSize);
if (!wavBuffer) {
  Serial.println("Failed to allocate WAV buffer");
  esp_camera_fb_return(fb);
  delay(1000);
  return;
}
Serial.printf("Allocated WAV buffer: %d bytes\n", wavSize);

writeWavHeader(wavBuffer, AUDIO_BUFFER_SIZE * sizeof(int16_t));
memcpy(wavBuffer + 44, (uint8_t*)audioBuffer, AUDIO_BUFFER_SIZE * sizeof(int16_t));

Serial.println("Sending multipart POST request...");
WiFiClient client;
sendMultipart(client, wavBuffer, wavSize, fb->buf, fb->len);

free(wavBuffer);
esp_camera_fb_return(fb);
delay(10000); // Wait before next capture
}
