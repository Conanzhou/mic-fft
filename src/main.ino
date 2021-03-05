#include <Arduino.h>
#include <driver/i2s.h>
#include "arduinoFFT.h"

arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

const i2s_port_t I2S_PORT = I2S_NUM_0;
const int SAMPLE_SIZE = 1024;
const int SAMPLE_Freq = 48000; //Hz, 48KHz
unsigned int sampling_period_us;
unsigned long microseconds;

/*
These are the input and output vectors
Input vectors receive computed results from FFT
*/
double vReal[SAMPLE_SIZE];
double vImag[SAMPLE_SIZE];

#define SCL_INDEX 0x00
#define SCL_TIME 0x01
#define SCL_FREQUENCY 0x02
#define SCL_PLOT 0x03

void setup()
{
    // put your setup code here, to run once:

    sampling_period_us = round(1000000 * (1.0 / SAMPLE_Freq));
    Serial.begin(115200);
    Serial.println("Configuring I2S...");
    esp_err_t err;

    // The I2S config as per the example
    const i2s_config_t i2s_config = {
        .mode = i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX), // Receive, not transfer
        .sample_rate = SAMPLE_Freq,
        .bits_per_sample = I2S_BITS_PER_SAMPLE_24BIT, // could only get it to work with 32bits
        .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,  // although the SEL config should be left, it seems to transmit on right
        .communication_format = i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
        .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1, // Interrupt level 1
        .dma_buf_count = 8,                       // number of buffers
        .dma_buf_len = SAMPLE_SIZE                // samples per buffer
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
    i2s_read(I2S_PORT, (void *)data_value, SAMPLE_SIZE * sizeof(uint32_t), &num_bytes_read, portMAX_DELAY);
    return num_bytes_read;
}

void PrintVector(double *vData, uint16_t bufferSize, uint8_t scaleType)
{
    for (uint16_t i = 0; i < bufferSize; i++)
    {
        double abscissa;
        /* Print abscissa value */
        switch (scaleType)
        {
        case SCL_INDEX:
            abscissa = (i * 1.0);
            break;
        case SCL_TIME:
            abscissa = ((i * 1.0) / SAMPLE_Freq);
            break;
        case SCL_FREQUENCY:
            abscissa = ((i * 1.0 * SAMPLE_Freq) / SAMPLE_SIZE);
            break;
        }
        Serial.print(abscissa, 6);
        if (scaleType == SCL_FREQUENCY)
            Serial.print("Hz");
        Serial.print(" ");
        Serial.println(vData[i], 4);
    }
    Serial.println();
}
void loop()
{
    // put your main code here, to run repeatedly:

    // Read multiple samples at once and calculate the sound pressure
    uint32_t samples[SAMPLE_SIZE];
    int num_read = data_read(samples);

    //  Serial.print("number read: ");
    //  Serial.println(num_read);

    // microseconds = micros();
    for (int i = 0; i < SAMPLE_SIZE; i++)
    {
        vReal[i] = (samples[i] & 0x00FFFFFF);
        vImag[i] = 0;
        //Serial.println(vReal[i]);
        // microseconds += sampling_period_us;
    }

    //    delay(10000);

    /* Print the results of the sampling according to time */
    //    Serial.println("Data:");
    //    PrintVector(vReal, SAMPLE_SIZE, SCL_TIME);
    //    delay(5000);
    FFT.Windowing(vReal, SAMPLE_SIZE, FFT_WIN_TYP_HAMMING, FFT_FORWARD); /* Weigh data */
    //    Serial.println("Weighed data:");
    //    PrintVector(vReal, SAMPLE_SIZE, SCL_TIME);
    //    delay(2000);
    FFT.Compute(vReal, vImag, SAMPLE_SIZE, FFT_FORWARD);                 /* Compute FFT */
    //    Serial.println("Computed Real values:");
    //    PrintVector(vReal, SAMPLE_SIZE, SCL_INDEX);
    //    delay(2000);
    //    Serial.println("Computed Imaginary values:");
    //    PrintVector(vImag, SAMPLE_SIZE, SCL_INDEX);
    //    delay(2000);
    FFT.ComplexToMagnitude(vReal, vImag, SAMPLE_SIZE);                   /* Compute magnitudes */
    // Serial.println("Computed magnitudes:");
    PrintVector(vReal, (SAMPLE_SIZE >> 1), SCL_FREQUENCY);
    //    delay(2000);
    //    double x = FFT.MajorPeak(vReal, SAMPLE_SIZE, SAMPLE_Freq);
    //    Serial.println(x, 6); //Print out what frequency is the most dominant.

    delay(2000); /* Repeat after delay */
}