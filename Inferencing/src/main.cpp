/**
 * Wio Terminal lungs Sound Classification Demo
 * 
 * Perform continuous Lungs sound classification using Edge Impulse library.
 * 
 * Author: Pratyush Mallick
 * Date: April 02, 2021
 * 
 * Connect analog microphone to pin 13 on the back of the Wio Terminal.
 * Pin 13 is also labelled BCM272 on the SAMD51
 * 
 * This works with the following MAX4466 Electret Microphone:
 * https://robu.in/product/max4466-electret-microphone-amplifier-with-adjustable-gain-module/
 * 
 * You will need to install the Edge Impulse .zip file as a library. This
 * sketch works with the following .zip library: 
 * https://github.com/ShawnHymel/ei-keyword-spotting/tree/master/embedded-demos/arduino/arduino-nano-33-ble-sense
 * Download the .zip file and install as Arduino library. 
 * 
 * Major part of the code has been derived from:
 * 1) https://github.com/AIWintermuteAI/Seeed_Arduino_Sketchbook/tree/master/examples/WioTerminal_TinyML_2_Audio_Scene_Recognition
 * 2) https://github.com/ShawnHymel/ei-keyword-spotting/blob/master/embedded-demos/arduino/wio-terminal/wio-terminal.ino
 * 
 * 
 * Copyright (c) 2021 EdgeImpulse Inc.
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "Free_Fonts.h"
#include "cet_80_60.h"

// If your target is limited in memory remove this macro to save 10K RAM
//#define EIDSP_QUANTIZE_FILTERBANK   1

/**
 * Define the number of slices per model window. E.g. a model window of 1000 ms
 * with slices per model window set to 4. Results in a slice size of 250 ms.
 * For more info: https://docs.edgeimpulse.com/docs/continuous-audio-sampling
 */
#define EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW   3

/* Includes ---------------------------------------------------------------- */
// Change this to match the Edge Impulse library that you downloaded
#include <digital_stethoscope_inference.h>

//Global Objects and Variables
TFT_eSPI tft = TFT_eSPI(); /* TFT instance */
volatile bool key_a = false;
volatile uint8_t count = 0;

/** Audio buffers, pointers and selectors */
typedef struct {
    int16_t *buffer;
    uint8_t buf_ready;
    uint32_t buf_count;
    uint32_t n_samples;
} inference_t;

static inference_t inference;
//Sampling period in microseconds
unsigned int sampling_period_us = round(600000 * (1.0 / 16000));
// Set this to true to see e.g. features generated from the raw signal
static bool debug_nn = false; 

/**
 * @brief      Printf function uses vsnprintf and output using Arduino Serial
 *
 * @param[in]  format     Variable argument list
 */
void ei_printf(const char *format, ...) {
    static char print_buf[1024] = { 0 };

    va_list args;
    va_start(args, format);
    int r = vsnprintf(print_buf, sizeof(print_buf), format, args);
    va_end(args);

    if (r > 0) {
        Serial.write(print_buf);
    }
}


/**
 * @brief      Init inferencing struct and setup/start PDM
 *
 * @param[in]  n_samples  The n samples
 *
 * @return     { description_of_the_return_value }
 */
static bool microphone_inference_start(uint32_t n_samples)
{
    inference.buffer = (int16_t *)malloc(n_samples * sizeof(int16_t));

    if(inference.buffer == NULL) {
        return false;
    }

    inference.buf_count  = 0;
    inference.n_samples  = n_samples;
    inference.buf_ready  = 0;
    //pinMode(WIO_MIC, INPUT);

    return true;
}

/**
 * @brief      Wait on new data
 *
 * @return     True when finished
 */
static bool microphone_inference_record(void)
{
    inference.buf_ready = 0;
    inference.buf_count = 0;

    if (inference.buf_ready == 0) {
        for(int i = 0; i < EI_CLASSIFIER_RAW_SAMPLE_COUNT; i++) {
            inference.buffer[inference.buf_count++] = map(analogRead(BCM27), 0, 65536, -32768, 32767);
            delayMicroseconds(sampling_period_us);
            
            if(inference.buf_count >= inference.n_samples) {
                inference.buf_count = 0;
                inference.buf_ready = 1;
                break;
            }
        }
    }

    return true;
}

/**
 * Get raw audio signal data
 */
static int microphone_audio_signal_get_data(size_t offset, size_t length, float *out_ptr)
{
    numpy::int16_to_float(&inference.buffer[offset], out_ptr, length);

    return 0;
}

/**
 * @brief      Stop PDM and release buffers
 */
static void microphone_inference_end(void)
{
    free(inference.buffer);
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_MICROPHONE
#error "Invalid model for current sensor."
#endif

//Interrupt Service Routine for Button A
void WIO_KEY_A_Handler(void)
{
   key_a = !(key_a);

   if(++count ==1)
   {
    tft.setFreeFont(FSSB9);
    tft.drawString("Press A to stop",160,20);
   }
}

//Funtion Display to the Instuctions 
static void no_flic(void)
{
    tft.fillRect(120,20,200,180,TFT_BLACK);
    tft.setFreeFont(FSSB12);
    tft.drawString("Press Button",160,60);
    tft.fillCircle(240,110,25,TFT_DARKGREEN);
    tft.drawChar(0x0041,232,119);
    tft.drawString("To start",190,150);
    tft.drawString("Analysis",189,180);
}

/**
 * @brief      Arduino setup function
 */
void setup()
{
    Serial.begin(115200);
    analogReadResolution(16);
    analogReference(AR_INTERNAL2V23);
    pinMode(BCM27, INPUT_PULLDOWN);

    // Setting for buttons for navigation
    pinMode(WIO_KEY_A, INPUT_PULLUP);
    attachInterrupt(WIO_KEY_A,WIO_KEY_A_Handler,FALLING);

    Serial.println("Edge Impulse Inferencing Demo");

    // summary of inferencing settings (from model_metadata.h)
    ei_printf("Inferencing settings:\n");
    ei_printf("\tInterval: %.2f ms.\n", (float)EI_CLASSIFIER_INTERVAL_MS);
    ei_printf("\tFrame size: %d\n", EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE);
    ei_printf("\tSample length: %d ms.\n", EI_CLASSIFIER_RAW_SAMPLE_COUNT / 16);
    ei_printf("\tNo. of classes: %d\n", sizeof(ei_classifier_inferencing_categories) / sizeof(ei_classifier_inferencing_categories[0]));
    
    // Initialize classifier
    run_classifier_init();

    if (microphone_inference_start(EI_CLASSIFIER_RAW_SAMPLE_COUNT) == false) {
        ei_printf("ERR: Failed to setup audio sampling\r\n");
        return;
    }

    // TFT initialization and opening screen
    tft.init();
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setSwapBytes(true);
    // Logo of my college
    tft.pushImage(15, 0, 100, 138, cet);
    // Logo of edge impulse 
    tft.pushImage(20, 160, 100, 71, EI);
    tft.setFreeFont(FSSB12);
    tft.drawString("Digital",195,100);
    tft.drawString("Stethoscope",160,128);

    // Stop and admire the digital stethoscope
    delay(2000);

    no_flic();
}

/**
 * @brief      Arduino main function. Runs the inferencing loop.
 */
void loop()
{
    // Start the recording and inferencing only when
    // button A is pressed
    if(key_a)
    {
        // Save the maximum label
        uint8_t max = 0;

        tft.fillRect(120,60,200,150,TFT_BLACK);
        ei_printf("Recording...\n");
        tft.setFreeFont(FSSB12);
        tft.drawString("Recording",160,100);

        // Wait until buffer is full
        bool m = microphone_inference_record();
        if (!m) {
        ei_printf("ERR: Failed to record audio...\n");
        return;
        }

        tft.fillRect(140,100,160,60,TFT_BLACK);
        ei_printf("Recording done\n");
        tft.drawString("Analyzing",160,100);

        // Do classification (i.e. the inference part)
        signal_t signal;
        signal.total_length = EI_CLASSIFIER_RAW_SAMPLE_COUNT;
        signal.get_data = &microphone_audio_signal_get_data;
        ei_impulse_result_t result = { 0 };

        EI_IMPULSE_ERROR r = run_classifier(&signal, &result, debug_nn);
        if (r != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", r);
        return;
        }

        // Print the predictions
        ei_printf("Predictions ");
        ei_printf("(DSP: %d ms., Classification: %d ms., Anomaly: %d ms.)",
        result.timing.dsp, result.timing.classification, result.timing.anomaly);
        ei_printf(": \n");
        for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {
        ei_printf("    %s: %.5f\n", result.classification[ix].label, result.classification[ix].value);
        
        if(result.classification[ix].value > 0.9)
            max = ix;
        }
        
        tft.fillRect(160,100,120,60,TFT_BLACK);
        tft.drawString("Results:",160,100);
        tft.setFreeFont(FSSB24);
        tft.drawString(result.classification[max].label,160,140);
        delay(1000);

        #if EI_CLASSIFIER_HAS_ANOMALY == 1
        ei_printf("    anomaly score: %.3f\n", result.anomaly);
        #endif
    }
    else
    {
        // To avoid flicker print the Instructions only once
        if(count == 2)
        {
            no_flic();
            count = 0;
        }
    }

}