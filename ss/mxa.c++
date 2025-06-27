#include <Arduino.h>
#include <driver/i2s.h>

#define I2S_BCLK 18
#define I2S_LRC  4
#define I2S_DOUT 5

#define SAMPLE_RATE     44100
#define SINE_FREQ       440    // Frequency of sine wave in Hz
#define AMPLITUDE       10000  // Max 32767 for int16
#define BUFFER_LEN      512

// I2S configuration
void setupI2S() {
  const i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_TX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // Mono to left
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 64,
    .use_apll = false,
    .tx_desc_auto_clear = true,
    .fixed_mclk = 0
  };

  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRC,
    .data_out_num = I2S_DOUT,
    .data_in_num = I2S_PIN_NO_CHANGE
  };

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

// Generate and send sine wave
void playSineTone() {
  static float phase = 0.0;
  static const float increment = 2.0 * PI * SINE_FREQ / SAMPLE_RATE;

  int16_t buffer[BUFFER_LEN];

  for (int i = 0; i < BUFFER_LEN; i++) {
    buffer[i] = (int16_t)(AMPLITUDE * sin(phase));
    phase += increment;
    if (phase >= 2.0 * PI) phase -= 2.0 * PI;
  }

  size_t bytes_written;
  i2s_write(I2S_NUM_0, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
}

void setup() {
  Serial.begin(115200);
  setupI2S();
  Serial.println("Playing sine wave on MAX98357A...");
}

void loop() {
  playSineTone();
}
