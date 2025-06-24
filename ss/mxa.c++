#include <Arduino.h>
#include "driver/i2s.h"
#include <math.h>

#define I2S_TX_DOUT 14  // Speaker DIN
#define I2S_TX_BCLK 12
#define I2S_TX_LRC  13

#define SAMPLE_RATE     16000
#define TONE_HZ         440
#define AMPLITUDE       5000
#define DURATION_SEC    2

void setupI2SSpeaker() {
  i2s_config_t tx_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = 256,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config_tx = {
    .bck_io_num = I2S_TX_BCLK,
    .ws_io_num = I2S_TX_LRC,
    .data_out_num = I2S_TX_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE  // Not used for TX
  };

  i2s_driver_install(I2S_NUM_1, &tx_config, 0, NULL);
  i2s_set_pin(I2S_NUM_1, &pin_config_tx);
  i2s_zero_dma_buffer(I2S_NUM_1);
}

void playSineWave() {
  int samples = SAMPLE_RATE / TONE_HZ;
  int16_t* buffer = new int16_t[samples];
  for (int i = 0; i < samples; i++) {
    buffer[i] = AMPLITUDE * sinf(2.0 * PI * i / samples);
  }

  size_t bytes_written;
  for (int i = 0; i < SAMPLE_RATE * DURATION_SEC / samples; i++) {
    i2s_write(I2S_NUM_1, buffer, samples * sizeof(int16_t), &bytes_written, portMAX_DELAY);
  }

  delete[] buffer;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("Initializing MAX98357A I2S speaker...");
  setupI2SSpeaker();
  delay(500);
  Serial.println("Playing 440Hz test tone...");
  playSineWave();
  Serial.println("Done.");
}

void loop() {
  delay(5000); // wait 5 seconds before repeating
}
