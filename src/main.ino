#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int BLOCK_SIZE = 1024;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  Serial.println("Configuring I2S...");
  esp_err_t err;

  // The I2S config as per the example
  const i2s_config_t i2s_config = {
      .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
      .sample_rate = 48000,                              // 48KHz
      .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT,      // could only get it to work with 32bits
      .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,       // although the SEL config should be left, it seems to transmit on right
      .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
      .dma_buf_count = 4,                       // number of buffers
      .dma_buf_len = BLOCK_SIZE                 // samples per buffer
  };

  // The pin config as per the setup
  const i2s_pin_config_t pin_config = {
      .bck_io_num = 14,   // BCKL
      .ws_io_num = 15,    // LRCL
      .data_out_num = -1, // not used (only for speakers)
      .data_in_num = 32   // DOUT
  };

  // Configuring the I2S driver and pins.
  // This function must be called before any I2S driver read/write operations.
  err = i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  if (err != ESP_OK)
  {
    Serial.printf("Failed installing driver: %d\n", err);
    while (true)
      ;
  }
  err = i2s_set_pin(I2S_PORT, &pin_config);
  if (err != ESP_OK)
  {
    Serial.printf("Failed setting pin: %d\n", err);
    while (true)
      ;
  }
  Serial.println("I2S driver installed.");
}

int data_read(uint32_t *data_value)
{
  size_t num_bytes_read = 0;
  i2s_read(I2S_PORT, (void *)data_value, BLOCK_SIZE * sizeof(uint32_t), &num_bytes_read, portMAX_DELAY);
  return num_bytes_read;
}

void   loop(){
  // put your main code here, to run repeatedly:

  // Read multiple samples at once and calculate the sound pressure
  uint32_t samples[BLOCK_SIZE];
  int num_read = data_read(samples);

  Serial.print("number read: ");
  Serial.println(num_read);
  for (int i = 0; i < BLOCK_SIZE ; i++)
  {
    samples[i] = (samples[i] & 0x00FFFFFF);
    Serial.println(samples[i]);
  }
  delay(10000);
}