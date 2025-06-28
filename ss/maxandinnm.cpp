#include "esp_camera.h"
#include <WiFi.h>
#include <WiFiClient.h>
#include "driver/i2s.h"

// Include camera pins specific to ESP32S3-EYE.
// Assuming the camera module you are using with your ESP32-S3-WROOM-1
// has these standard ESP32S3-EYE pin assignments.
#define CAMERA_MODEL_ESP32S3_EYE
#include "camera_pins.h"

// Button pin - GPIO21 is generally available on ESP32-S3-WROOM-1
// and avoids conflicts with camera pins.
#define BUTTON_PIN 21

// WiFi credentials
const char* ssid = "Kawcher";
const char* password = "01970916952";

// Server info
const char* serverHost = "192.168.0.104";  // IP or domain of your server
const int serverPort = 80;
const char* serverPath = "/upload";

const int RECORD_SECONDS = 7;
const int SAMPLE_RATE = 16000;
const int I2S_READ_LEN = 1024;

// I2S Mic pins (these are standard for the onboard PDM mic on ESP32-S3-EYE board)
// If you are using a different mic or connecting it externally, confirm these pins.
#define I2S_MIC_WS   20    // Word Select (LRC)
#define I2S_MIC_SD   2     // Serial Data (DIN)
#define I2S_MIC_SCK  41    // Bit Clock (BCLK)
#define I2S_PORT_MIC I2S_NUM_0 // I2S port for microphone input

// I2S Output pins - CHANGED for ESP32-S3-WROOM-1 to use generally available GPIOs.
// These pins (GPIO35, GPIO36, GPIO37) are chosen to avoid conflicts with common
// camera pinouts and typical ESP32-S3-WROOM-1 development board configurations.
// Connect your external I2S DAC to these pins.
#define I2S_OUT_DOUT      35   // Data Out (changed from 26)
#define I2S_OUT_BCLK      36   // Bit Clock (changed from 33)
#define I2S_OUT_LRC       37   // Left/Right Clock (Word Select) (changed from 34)
#define I2S_PORT_OUT      I2S_NUM_1 // I2S port for audio output

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

// Fills the WAV header with correct sizes based on audio data size.
void fillWavHeader(WAVHeader &header, uint32_t dataSize) {
  header.subChunk2Size = dataSize;
  header.chunkSize = 36 + dataSize;
  header.byteRate = header.sampleRate * header.numChannels * header.bitsPerSample / 8;
  header.blockAlign = header.numChannels * header.bitsPerSample / 8;
}

// Connects to the configured WiFi network.
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

// Configures I2S for audio output using the newly assigned pins (GPIO35, 36, 37).
void setupI2SOutput() {
  i2s_config_t i2s_config_out = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX), // ESP32 as master, Transmit mode
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT, // Stereo output format (typical for DACs)
    .communication_format = I2S_COMM_FORMAT_I2S_MSB, // I2S MSB format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
    .dma_buf_count = 4, // Number of DMA buffers
    .dma_buf_len = 1024, // Size of each DMA buffer
    .use_apll = false, // Use APLL (Audio PLL)
    .tx_desc_auto_clear = true, // Auto clear descriptor on transmit
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_out = {
    .bck_io_num = I2S_OUT_BCLK, // New BCLK pin (GPIO36)
    .ws_io_num = I2S_OUT_LRC,   // New LRC pin (GPIO37)
    .data_out_num = I2S_OUT_DOUT, // New DOUT pin (GPIO35)
    .data_in_num = I2S_PIN_NO_CHANGE // Not used for transmit
  };

  // Install the I2S driver for the output port
  i2s_driver_install(I2S_PORT_OUT, &i2s_config_out, 0, NULL);
  // Set the I2S pins
  i2s_set_pin(I2S_PORT_OUT, &pin_config_out);
  // Zero the DMA buffer
  i2s_zero_dma_buffer(I2S_PORT_OUT);
  Serial.println("I2S Output driver installed on new pins.");
}

// Configures I2S for microphone input using standard ESP32S3-EYE mic pins.
void setupI2SMic() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX), // ESP32 as master, Receive mode
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT, // Only read from the left channel (mono mic)
    .communication_format = I2S_COMM_FORMAT_I2S_MSB, // I2S MSB format
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
    .dma_buf_count = 4, // Number of DMA buffers
    .dma_buf_len = 1024, // Size of each DMA buffer
    .use_apll = false, // Do not use APLL
    .tx_desc_auto_clear = false, // Don't auto clear descriptor (receive mode)
    .fixed_mclk = 0
  };

  // CORRECTED: Reordered data_out_num and data_in_num to match i2s_pin_config_t declaration order.
  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_MIC_SCK, // Mic BCLK pin (GPIO41)
    .ws_io_num = I2S_MIC_WS,   // Mic WS pin (GPIO20)
    .data_out_num = I2S_PIN_NO_CHANGE, // Not used for receive, but must be in order
    .data_in_num = I2S_MIC_SD // Mic DIN pin (GPIO2)
  };

  // Install the I2S driver for the microphone port
  i2s_driver_install(I2S_PORT_MIC, &i2s_config, 0, NULL);
  // Set the I2S pins
  i2s_set_pin(I2S_PORT_MIC, &pin_config);
  // Zero the DMA buffer
  i2s_zero_dma_buffer(I2S_PORT_MIC);
  Serial.println("I2S Mic driver installed.");
}

// Records audio from the microphone.
void recordAudio() {
  Serial.println("Recording audio...");
  size_t bytes_read;
  uint8_t buffer[I2S_READ_LEN];
  unsigned long start = millis();
  audioData.clear(); // Clear previous audio data to prepare for new recording

  while (millis() - start < RECORD_SECONDS * 1000) {
    // Read data from I2S microphone
    esp_err_t result = i2s_read(I2S_PORT_MIC, &buffer, I2S_READ_LEN, &bytes_read, portMAX_DELAY);
    if (result == ESP_OK && bytes_read > 0) {
      // Append read data to audioData vector
      audioData.insert(audioData.end(), buffer, buffer + bytes_read);
    } else {
      Serial.printf("I2S read failed or returned zero bytes. err=%d\n", result);
    }
    delay(10); // ADDED: Small delay to prevent watchdog timer reset
  }

  Serial.printf("Recording complete. Collected %d bytes.\n", audioData.size());
}

// Captures an image from the camera.
camera_fb_t* captureImage() {
  Serial.println("Capturing image...");
  camera_fb_t* fb = esp_camera_fb_get(); // Get a frame buffer from the camera
  if (!fb) {
    Serial.println("Camera capture failed");
  } else {
    Serial.printf("Image captured: %d bytes\n", fb->len);
  }
  return fb;
}

// Sends captured image and audio data to the server and handles audio response.
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

  String boundary = "----ESP32FormBoundary"; // Multipart form data boundary for HTTP POST
  String contentType = "multipart/form-data; boundary=" + boundary;

  // Prepare the start of the multipart request for the image
  String startRequest = "--" + boundary + "\r\n"
                        "Content-Disposition: form-data; name=\"image\"; filename=\"image.jpg\"\r\n"
                        "Content-Type: image/jpeg\r\n\r\n";

  String midRequest = "";
  uint8_t wavHeader[44]; // Buffer for WAV header
  if (!audioData.empty()) {
    WAVHeader header;
    fillWavHeader(header, audioData.size()); // Fill WAV header with audio data info
    memcpy(wavHeader, &header, sizeof(WAVHeader)); // Copy filled header to byte buffer

    // Prepare the middle part of the request for audio if available
    midRequest = "\r\n--" + boundary + "\r\n"
                 "Content-Disposition: form-data; name=\"audio\"; filename=\"audio.wav\"\r\n"
                 "Content-Type: audio/wav\r\n\r\n";
  }

  // Prepare the end of the multipart request
  String endRequest = "\r\n--" + boundary + "--\r\n";

  // Calculate total length of the request body
  int totalLen = startRequest.length() + fb->len +
                 midRequest.length() +
                 (audioData.empty() ? 0 : (sizeof(WAVHeader) + audioData.size())) + // Add WAV header size
                 endRequest.length();

  // Construct HTTP POST request headers
  String httpRequest = String("POST ") + serverPath + " HTTP/1.1\r\n" +
                       "Host: " + serverHost + "\r\n" +
                       "Content-Type: " + contentType + "\r\n" +
                       "Content-Length: " + String(totalLen) + "\r\n" +
                       "Connection: close\r\n\r\n";

  // Send HTTP headers
  client.print(httpRequest);
  // Send image part
  client.print(startRequest);
  client.write(fb->buf, fb->len);

  // Send audio part if available
  if (!audioData.empty()) {
    client.print(midRequest);
    client.write(wavHeader, sizeof(wavHeader)); // Send WAV header bytes
    client.write(audioData.data(), audioData.size()); // Send raw audio data
  }

  // Send end of multipart request
  client.print(endRequest);

  Serial.println("Request sent. Waiting for response...");

  // Read server response headers
  while (client.connected()) {
    String line = client.readStringUntil('\n');
    Serial.println("Header: " + line);
    if (line == "\r" || line.length() == 0) break; // End of headers is an empty line
  }
  
  delay(100); // Give server time to start sending data

  // Play WAV audio response directly from the server stream
  uint8_t buffer[512];
  size_t bytes_written;
  int skipBytes = 44; // Assuming the server response also starts with a 44-byte WAV header

  while (client.connected()) {
    while (client.available()) {
      int len = client.read(buffer, sizeof(buffer));
      if (len > 0) {
        int toWrite = len;
        int offset = 0;

        // Skip header bytes from the incoming stream on the first few reads
        if (skipBytes > 0) {
          if (len <= skipBytes) {
            skipBytes -= len;
            continue; // Skip the entire buffer if it's still part of the header
          } else {
            offset = skipBytes; // Start writing after the header portion
            toWrite = len - skipBytes;
            skipBytes = 0; // Header fully skipped
          }
        }
        // Write audio data to I2S output
        i2s_write(I2S_PORT_OUT, buffer + offset, toWrite, &bytes_written, portMAX_DELAY);
      }
    }
    delay(10); // Small delay to prevent tight loop and allow other tasks to run
  }

  client.stop(); // Close the connection to the server
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT_PULLUP); // Initialize the BUTTON_PIN (GPIO21)

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
  config.frame_size = FRAMESIZE_QVGA; // Reduced frame size for faster processing/upload
  config.jpeg_quality = 10;
  config.fb_count = 1;
  config.fb_location = CAMERA_FB_IN_PSRAM; // Store frame buffer in PSRAM
  config.grab_mode = CAMERA_GRAB_LATEST; // Grab the latest frame

  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed");
    while (1) delay(1000); // Halt if camera initialization fails
  }

  connectWiFi(); // Connect to WiFi network
  setupI2SMic();    // Setup I2S for microphone input
  setupI2SOutput(); // Setup I2S for audio output with new, compatible pins
  Serial.println("Setup complete.");
}

void loop() {
  static unsigned long lastAutoTrigger = 0;
  const unsigned long autoTriggerInterval = 60000; // 1 minute interval for auto trigger

  unsigned long now = millis();

  // Debug print every 5 seconds to confirm loop is running and timers are progressing
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 5000) {
    Serial.printf("Millis: %lu, lastAutoTrigger: %lu\n", now, lastAutoTrigger);
    lastDebug = now;
  }

  // Check for auto trigger interval to perform capture and send
  if (now - lastAutoTrigger >= autoTriggerInterval) {
    Serial.println("Auto trigger (1 minute) starting...");

    camera_fb_t* fb = captureImage(); // Capture image
    if (fb) {
      recordAudio(); // Record audio
      sendToServer(fb); // Send data to server and play response
      esp_camera_fb_return(fb); // Return frame buffer to camera to free memory
    } else {
      Serial.println("Failed to capture image");
    }

    Serial.println("Auto capture finished.");
    lastAutoTrigger = now; // Reset auto trigger timer for the next interval
  }
}
