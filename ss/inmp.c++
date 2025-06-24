#include <WiFi.h>
#include <HTTPClient.h>
#include "driver/i2s.h"

// WiFi credentials
const char* ssid = "Kawcher";
const char* password = "01970916952";

// Flask server (adjust to your PC's IP address)
#define SERVER_URL "http://192.168.0.105:5000/upload"

// INMP441 I2S pins
#define I2S_WS 17
#define I2S_SD 16
#define I2S_SCK 18

// Audio config
#define SAMPLE_RATE 8000
#define RECORD_TIME_SECONDS 5
#define AUDIO_BUFFER_SIZE (SAMPLE_RATE * RECORD_TIME_SECONDS)

int16_t* audioBuffer = nullptr;

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

void writeWavHeader(uint8_t* buffer, size_t dataSize) {
  memcpy(buffer, "RIFF", 4);
  uint32_t chunkSize = 36 + dataSize;
  memcpy(buffer + 4, &chunkSize, 4);
  memcpy(buffer + 8, "WAVE", 4);
  memcpy(buffer + 12, "fmt ", 4);

  uint32_t subchunk1Size = 16;
  uint16_t audioFormat = 1;
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

void sendWavToServer() {
  size_t wavSize = 44 + AUDIO_BUFFER_SIZE * sizeof(int16_t);
  uint8_t* wavData = (uint8_t*)malloc(wavSize);
  if (!wavData) {
    Serial.println("Failed to allocate memory for WAV data!");
    return;
  }

  writeWavHeader(wavData, AUDIO_BUFFER_SIZE * sizeof(int16_t));
  memcpy(wavData + 44, (uint8_t*)audioBuffer, AUDIO_BUFFER_SIZE * sizeof(int16_t));

  WiFiClient client;
  HTTPClient http;

  http.begin(client, SERVER_URL);
  http.setTimeout(10000);
  http.addHeader("Content-Type", "audio/wav");

  int httpResponseCode = http.POST(wavData, wavSize);
  if (httpResponseCode > 0) {
    String response = http.getString();
    Serial.println("Server response:");
    Serial.println(response);
  } else {
    Serial.printf("POST failed: %s\n", http.errorToString(httpResponseCode).c_str());
  }

  http.end();
  free(wavData);
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

  audioBuffer = (int16_t*)malloc(AUDIO_BUFFER_SIZE * sizeof(int16_t));
  if (!audioBuffer) {
    Serial.println("Failed to allocate audio buffer!");
    while (true) delay(1000);
  }

  setupI2SMic();
}

void loop() {
  Serial.println("Recording...");

  size_t bytesRead = 0;
  size_t index = 0;

  while (index < AUDIO_BUFFER_SIZE) {
    size_t toRead = (AUDIO_BUFFER_SIZE - index) * sizeof(int16_t);
    if (i2s_read(I2S_NUM_0, (void*)(audioBuffer + index), toRead, &bytesRead, portMAX_DELAY) == ESP_OK) {
      index += bytesRead / sizeof(int16_t);
    }
  }

  Serial.println("Recording done.");

  // Print first 10 samples to check signal
  for (int i = 0; i < 10; i++) {
    Serial.printf("Sample[%d] = %d\n", i, audioBuffer[i]);
  }

  // Check if input is mostly silent
  int silentCount = 0;
  for (int i = 0; i < AUDIO_BUFFER_SIZE; i++) {
    if (abs(audioBuffer[i]) < 100) silentCount++;
  }
  if ((float)silentCount / AUDIO_BUFFER_SIZE > 0.9) {
    Serial.println("⚠️ Warning: Mic input is mostly silent.");
  } else {
    Serial.println("✅ Mic seems to be working.");
  }

  sendWavToServer();
  delay(5000); // wait before next recording
}
