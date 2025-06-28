#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "driver/i2s.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

#define BUTTON_PIN 21

const char* ssid = "Kawcher";
const char* password = "01970916952";

const char* serverHost = "192.168.0.104";
const int serverPort = 80;
const char* serverPath = "/upload";

#define SAMPLE_RATE     16000
#define RECORD_SECONDS  5
#define I2S_READ_LEN    1024

#define I2S_WS          20
#define I2S_SD          2
#define I2S_SCK         41
#define I2S_PORT        I2S_NUM_0

#define I2S_OUT_DOUT    35
#define I2S_OUT_BCLK    36
#define I2S_OUT_LRC     37
#define I2S_PORT_OUT    I2S_NUM_1

#define MAX_AUDIO_SIZE  (SAMPLE_RATE * 2 * RECORD_SECONDS) // 2 bytes per sample (16-bit)
uint8_t audioBuffer[MAX_AUDIO_SIZE];
size_t audioLen = 0;

uint8_t* responseAudio = nullptr;
size_t responseAudioLen = 0;

struct WAVHeader {
  char riff[4] = {'R', 'I', 'F', 'F'};
  uint32_t chunkSize;
  char wave[4] = {'W', 'A', 'V', 'E'};
  char fmt[4] = {'f', 'm', 't', ' '};
  uint32_t subChunk1Size = 16;
  uint16_t audioFormat = 1;
  uint16_t numChannels = 1; // Assuming mono audio for mic and playback
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
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected. IP: " + WiFi.localIP().toString());
}

void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Assuming mono input from mic
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

  esp_err_t err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK) {
      Serial.printf("Failed to install I2S Mic driver: %d\n", err);
      while(1) delay(1000);
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK) {
      Serial.printf("Failed to set I2S Mic pins: %d\n", err);
      while(1) delay(1000);
  }
  i2s_zero_dma_buffer(I2S_PORT);
  Serial.println("I2S Mic driver installed.");
}

void setupI2SOutput() {
  i2s_config_t i2s_config_out = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Assuming pyttsx3 output is mono
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8, // Increased buffer count
    .dma_buf_len = 1024, // Increased buffer length
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_out = {
    .bck_io_num = I2S_OUT_BCLK,
    .ws_io_num = I2S_OUT_LRC,
    .data_out_num = I2S_OUT_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  esp_err_t err = i2s_driver_install(I2S_PORT_OUT, &i2s_config_out, 0, NULL);
  if (err != ESP_OK) {
      Serial.printf("Failed to install I2S Output driver: %d\n", err);
      // Don't halt, try to continue or handle gracefully
  }
  err = i2s_set_pin(I2S_PORT_OUT, &pin_config_out);
  if (err != ESP_OK) {
      Serial.printf("Failed to set I2S Output pins: %d\n", err);
      // Don't halt
  }
  i2s_zero_dma_buffer(I2S_PORT_OUT);
  Serial.println("I2S Output driver installed.");
}

void recordAudio() {
  Serial.println("Recording audio...");
  uint8_t buffer[I2S_READ_LEN];
  size_t bytes_read;
  unsigned long start = millis();
  audioLen = 0;

  // Clear the audioBuffer before recording to ensure no old data remains
  memset(audioBuffer, 0, sizeof(audioBuffer)); 

  while (millis() - start < RECORD_SECONDS * 1000 && audioLen + I2S_READ_LEN <= MAX_AUDIO_SIZE) {
    esp_err_t result = i2s_read(I2S_PORT, &buffer, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    if (result == ESP_OK && bytes_read > 0) {
      memcpy(audioBuffer + audioLen, buffer, bytes_read);
      audioLen += bytes_read;
    } else if (result != ESP_OK) {
        Serial.printf("i2s_read error: %d\n", result);
    }
    yield(); // Yield to allow other tasks to run and prevent watchdog
  }

  Serial.printf("Recording complete. Bytes recorded: %u\n", audioLen);
}

camera_fb_t* captureImage() {
  Serial.println("Capturing image...");
  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) Serial.println("Camera capture failed");
  else Serial.printf("Image captured: %d bytes\n", fb->len);
  return fb;
}

void sendToServer(camera_fb_t* fb) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected! Attempting to reconnect...");
    connectWiFi(); // Try to reconnect
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Failed to reconnect to WiFi. Aborting send.");
        if (fb) esp_camera_fb_return(fb); // Free camera buffer if not sent
        return;
    }
  }

  WiFiClient client;
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("Connection to server failed");
    if (fb) esp_camera_fb_return(fb); // Free camera buffer if not sent
    return;
  }

  String boundary = "----ESP32FormBoundary";
  String contentType = "multipart/form-data; boundary=" + boundary;

  String startRequest = "--" + boundary + "\r\n"
                        "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
                        "Content-Type: image/jpeg\r\n\r\n";

  String midRequest = "\r\n--" + boundary + "\r\n"
                      "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n"
                      "Content-Type: audio/wav\r\n\r\n";

  String endRequest = "\r\n--" + boundary + "--\r\n";

  WAVHeader wavHeader;
  fillWavHeader(wavHeader, audioLen);
  uint8_t headerBytes[sizeof(WAVHeader)]; // Use sizeof(WAVHeader)
  memcpy(headerBytes, &wavHeader, sizeof(WAVHeader));

  // Total HTTP body length
  int totalLen = startRequest.length() + fb->len +
                 midRequest.length() + sizeof(headerBytes) + audioLen +
                 endRequest.length();

  String httpRequest = String("POST ") + serverPath + " HTTP/1.1\r\n" +
                       "Host: " + serverHost + "\r\n" +
                       "Content-Type: " + contentType + "\r\n" +
                       "Content-Length: " + String(totalLen) + "\r\n" +
                       "Connection: close\r\n\r\n";

  // === Send the request ===
  client.print(httpRequest);
  client.print(startRequest);
  client.write(fb->buf, fb->len);
  client.print(midRequest);
  client.write(headerBytes, sizeof(headerBytes));
  client.write(audioBuffer, audioLen);
  client.print(endRequest);

  Serial.println("Data sent. Waiting for response...");

  // Free camera buffer immediately after sending
  esp_camera_fb_return(fb);
  
  // === Read and print status line ===
  String statusLine = client.readStringUntil('\n');
  Serial.println("Status line: " + statusLine);

  // === Read headers ===
  String headerLine;
  while (true) {
    headerLine = client.readStringUntil('\n');
    if (headerLine == "\r" || headerLine.length() == 0) break;
    Serial.println("Header: " + headerLine);
  }

  // === Read response body ===
  responseAudioLen = 0;
  size_t capacity = 8192; // Initial capacity
  if (responseAudio) {
    free(responseAudio);
    responseAudio = nullptr;
  }
  responseAudio = (uint8_t*)malloc(capacity);
  if (!responseAudio) {
    Serial.println("Failed to allocate memory for response audio (initial)");
    responseAudioLen = 0; // Ensure length is zero on failure
    return;
  }

  unsigned long lastReceive = millis();
  const unsigned long timeout = 10000; // Increased timeout for response reception

  while (client.connected() || client.available()) {
    while (client.available()) {
      if (responseAudioLen >= capacity) {
        capacity *= 2; // Double capacity
        Serial.printf("Reallocating response audio buffer to %u bytes\n", capacity);
        uint8_t* newBuffer = (uint8_t*)realloc(responseAudio, capacity);
        if (!newBuffer) {
          Serial.println("Failed to reallocate buffer for response audio");
          free(responseAudio);
          responseAudio = nullptr;
          responseAudioLen = 0;
          client.stop(); // Stop client to prevent further attempts
          return;
        }
        responseAudio = newBuffer;
      }
      int c = client.read();
      if (c < 0) break; // No more data
      responseAudio[responseAudioLen++] = (uint8_t)c;
      lastReceive = millis();
    }
    if (millis() - lastReceive > timeout) {
      Serial.println("Response read timeout - no more data for a while.");
      break;
    }
    yield(); // Yield to prevent watchdog during long receive
  }

  client.stop();
  Serial.printf("Audio response received. Size: %u bytes\n", responseAudioLen);

  // === Playback response in chunks ===
  if (responseAudio && responseAudioLen > 0) {
    Serial.printf("Free PSRAM before playback: %u bytes\n", ESP.getFreePsram());
    setupI2SOutput();

    size_t written = 0;
    const size_t chunkSize = 2048; // Increased chunk size for potentially better performance
    Serial.println("Starting audio playback...");
    for (size_t i = 0; i < responseAudioLen; i += written) { // Iterate by written bytes
      size_t toWrite = min(chunkSize, responseAudioLen - i);
      esp_err_t err = i2s_write(I2S_PORT_OUT, responseAudio + i, toWrite, &written, portMAX_DELAY);
      if (err != ESP_OK) {
        Serial.printf("i2s_write error at offset %u: %d\n", i, err);
        break;
      }
      if (written == 0) { // If no bytes were written, something is wrong, prevent infinite loop
          Serial.println("i2s_write wrote 0 bytes, stopping playback.");
          break;
      }
      yield(); // Crucial for watchdog
      // Small delay might be helpful, but too much will cause gaps
      // delay(1); // Consider reducing or removing if it causes gaps
    }

    Serial.println("Playback loop finished.");
    i2s_driver_uninstall(I2S_PORT_OUT);
    Serial.println("I2S Output driver uninstalled.");
  } else {
    Serial.println("No audio response or response is empty to play.");
  }

  // Free audio buffer
  if (responseAudio) {
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
    Serial.println("Response audio buffer freed.");
  }
  // Clear mic audio buffer after sending
  memset(audioBuffer, 0, audioLen);
  audioLen = 0;
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP);

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

  Serial.printf("Free heap: %u bytes\n", ESP.getFreeHeap());
  Serial.printf("Free PSRAM: %u bytes\n", ESP.getFreePsram());
  Serial.println("Setup complete.");
}

void loop() {
  static unsigned long lastButtonPress = 0;
  static bool buttonPressed = false;
  unsigned long now = millis();

  // Check for button press (active LOW with INPUT_PULLUP)
  if (digitalRead(BUTTON_PIN) == LOW && (now - lastButtonPress > 500)) {
    buttonPressed = true;
    lastButtonPress = now;
  }

  if (buttonPressed) {
    Serial.println("Button trigger starting...");

    camera_fb_t* fb = captureImage();
    if (fb) {
      recordAudio();
      sendToServer(fb);
    } else {
      Serial.println("No camera frame buffer to send.");
    }

    Serial.println("Capture finished.");
    buttonPressed = false;
  }

  delay(50); // Small delay for stability and watchdog
}
