#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "driver/i2s.h"

#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

#define BUTTON_PIN 21

//const char* ssid = "UIU-STUDENT";
//const char* password = "12345678";
// const char* ssid = "Redmi 10C";
// const char* password = "zeff1952";
 const char* ssid = "Kawcher";
 const char* password = "01970916952";

 const char* serverHost = "192.168.0.104";
//const char* serverHost = "10.15.50.199";
const int serverPort = 80;
const char* serverPath = "/upload";

#define SAMPLE_RATE     16000
#define RECORD_SECONDS  5
#define I2S_READ_LEN    1024

#define I2S_WS          20
#define I2S_SD          2
#define I2S_SCK         41
#define I2S_PORT        I2S_NUM_0


#define I2S_OUT_DOUT 13 // Data Out (DIN)
#define I2S_OUT_BCLK 14 // Bit Clock
#define I2S_OUT_LRC  15 // Left-Right Clock

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


// ---------------- WIFI CONNECT --------------------
void connectWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  unsigned long startAttemptTime = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000) {
    Serial.print(".");
    delay(500);
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nWiFi connected!" : "\nFailed to connect.");
}

bool ensureWiFiConnected() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
    return WiFi.status() == WL_CONNECTED;
  }
  return true;
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
//
//#define I2S_OUT_DOUT 13 // Data Out (DIN)
//#define I2S_OUT_BCLK 14 // Bit Clock
//#define I2S_OUT_LRC  15 // Left-Right Clock
//
//#define I2S_PORT_OUT    I2S_NUM_1

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

void cleanupBuffers(camera_fb_t* fb) {
  if (fb) {
    esp_camera_fb_return(fb);
  }

  if (responseAudio) {
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
  }

  memset(audioBuffer, 0, audioLen);
  audioLen = 0;
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

// ---------------- MULTIPART REQUEST --------------------
void sendMultipartRequest(WiFiClient &client, camera_fb_t* fb) {
  String boundary = "----ESP32FormBoundary";
  String contentType = "multipart/form-data; boundary=" + boundary;

  String partImageHeader = "--" + boundary + "\r\n" +
                           "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n" +
                           "Content-Type: image/jpeg\r\n\r\n";

  String partAudioHeader = "\r\n--" + boundary + "\r\n" +
                           "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n" +
                           "Content-Type: audio/wav\r\n\r\n";

  String closingBoundary = "\r\n--" + boundary + "--\r\n";

  // Build WAV header for raw PCM data
  WAVHeader wavHeader;
  fillWavHeader(wavHeader, audioLen);
  uint8_t headerBytes[sizeof(WAVHeader)];
  memcpy(headerBytes, &wavHeader, sizeof(WAVHeader));

  // Calculate total content length
  int totalLen = partImageHeader.length() + fb->len +
                 partAudioHeader.length() + sizeof(headerBytes) + audioLen +
                 closingBoundary.length();

  String httpRequest = String("POST ") + serverPath + " HTTP/1.1\r\n" +
                       "Host: " + serverHost + "\r\n" +
                       "Content-Type: " + contentType + "\r\n" +
                       "Content-Length: " + String(totalLen) + "\r\n" +
                       "Connection: close\r\n\r\n";

  // Send request header
  client.print(httpRequest);

  // Send image part
  client.print(partImageHeader);
  client.write(fb->buf, fb->len);
  client.print("\r\n");  // <-- important! Ends image data

  // Send audio part
  client.print(partAudioHeader);
  client.write(headerBytes, sizeof(headerBytes));
  client.write(audioBuffer, audioLen);

  // Send closing boundary
  client.print(closingBoundary);

  Serial.println("Request sent. Waiting for response...");
}


// ---------------- SERVER RESPONSE --------------------
void readServerResponse(WiFiClient &client) {
  Serial.println("Reading server response...");

  // Step 1: Read and print status line
  String statusLine = client.readStringUntil('\n');
  Serial.println("Status: " + statusLine);

  // Step 2: Skip HTTP headers by detecting "\r\n\r\n"
  String headers = "";
  while (client.connected()) {
    char c = client.read();
    if (c < 0) break;
    headers += c;
    if (headers.endsWith("\r\n\r\n")) {
      Serial.println("[INFO] End of HTTP headers.");
      break;
    }
  }

  // Step 3: Prepare response audio buffer
  if (responseAudio) {
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
  }

  size_t capacity = 8192;
  responseAudio = (uint8_t*)malloc(capacity);
  if (!responseAudio) {
    Serial.println("[ERROR] Memory allocation failed for response audio.");
    return;
  }

  // Step 4: Read binary audio body
  unsigned long lastReceive = millis();
  const unsigned long timeout = 10000;

  while (client.connected() || client.available()) {
    while (client.available()) {
      if (responseAudioLen >= capacity) {
        capacity *= 2;
        uint8_t* newBuffer = (uint8_t*)realloc(responseAudio, capacity);
        if (!newBuffer) {
          Serial.println("[ERROR] Realloc failed for response audio.");
          free(responseAudio);
          responseAudio = nullptr;
          responseAudioLen = 0;
          return;
        }
        responseAudio = newBuffer;
      }

      int c = client.read();
      if (c < 0) break;
      responseAudio[responseAudioLen++] = (uint8_t)c;
      lastReceive = millis();
    }

    if (millis() - lastReceive > timeout) {
      Serial.println("[ERROR] Read timeout.");
      break;
    }

    yield();  // avoid watchdog reset
  }

  client.stop();
  Serial.printf("Received %u bytes of audio\n", responseAudioLen);
}



// ---------------- PLAY RESPONSE --------------------
void playWavResponse() {
  Serial.printf("Total responseAudioLen = %u bytes\n", responseAudioLen);

  // Check if response is at least large enough to contain a WAV header
  if (responseAudioLen <= 44) {
    Serial.println("[ERROR] Response too short to contain WAV header.");
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
    return;
  }

  // Print first 16 bytes of the response
  Serial.println("First 16 bytes of responseAudio:");
  for (int i = 0; i < 16 && i < responseAudioLen; i++) {
    Serial.printf("%02X ", responseAudio[i]);
  }
  Serial.println();

  // Check for "RIFF" magic header
  if (!(responseAudio[0] == 'R' && responseAudio[1] == 'I' &&
        responseAudio[2] == 'F' && responseAudio[3] == 'F')) {
    Serial.print("[ERROR] WAV header missing or invalid: Expected 'RIFF' but got: ");
    for (int i = 0; i < 4 && i < responseAudioLen; i++) {
      Serial.print((char)responseAudio[i]);
    }
    Serial.println();
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
    return;
  }

  Serial.println("WAV header is valid. Beginning playback...");

  setupI2SOutput();

  const size_t wavHeaderSize = 44;
const size_t chunkSize = 2048;

size_t offset = wavHeaderSize;

while (offset < responseAudioLen) {
  size_t toWrite = min(chunkSize, responseAudioLen - offset);
  size_t written = 0;
  esp_err_t err = i2s_write(I2S_PORT_OUT, responseAudio + offset, toWrite, &written, portMAX_DELAY);
  if (err != ESP_OK || written == 0) {
    Serial.printf("[ERROR] i2s_write failed at offset %u, err: %d\n", offset, err);
    break;
  }
  offset += written;
  yield();
}

  Serial.println("Playback complete. Cleaning up...");

  i2s_driver_uninstall(I2S_PORT_OUT);
  free(responseAudio);
  responseAudio = nullptr;
  responseAudioLen = 0;
  memset(audioBuffer, 0, audioLen);
  audioLen = 0;
}

// ---------------- MASTER FUNCTION --------------------
void sendToServer(camera_fb_t* fb) {
  if (!ensureWiFiConnected()) {
    cleanupBuffers(fb);
    return;
  }

  WiFiClient client;
  if (!client.connect(serverHost, serverPort)) {
    Serial.println("Connection to server failed.");
    cleanupBuffers(fb);
    return;
  }

  sendMultipartRequest(client, fb);

  // Return image buffer here after sending request (not earlier)
  esp_camera_fb_return(fb);
  fb = nullptr;  // Mark as returned

  readServerResponse(client);
  client.stop();

  playWavResponse();

  // After playback, cleanup audio buffer
  if (responseAudio) {
    free(responseAudio);
    responseAudio = nullptr;
    responseAudioLen = 0;
  }

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

  if (digitalRead(BUTTON_PIN) == LOW && (now - lastButtonPress > 500)) {
    buttonPressed = true;
    lastButtonPress = now;
  }

  if (buttonPressed) {
    Serial.println("Button trigger starting...");

    // Clean buffers before new capture
    cleanupBuffers(nullptr);

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

  delay(50);
}
