/*
M5StickC Audio Spectrum Display, Oscilloscope, and Tuner

Copyright 2020 KIRA Ryouta

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

/* ESP8266/32 Audio Spectrum Analyser on an SSD1306/SH1106 Display
 * The MIT License (MIT) Copyright (c) 2017 by David Bird. 
 * The formulation and display of an AUdio Spectrum using an ESp8266 or ESP32 and SSD1306 or SH1106 OLED Display using a Fast Fourier Transform
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files 
 * (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, 
 * publish, distribute, but not to use it commercially for profit making or to sub-license and/or to sell copies of the Software or to 
 * permit persons to whom the Software is furnished to do so, subject to the following conditions:  
 * The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software. 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES 
 * OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE 
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN 
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE. 
 * See more at http://dsbird.org.uk 
*/

// M5StickC Audio Spectrum             : 2019.06.01 : macsbug
//  https://macsbug.wordpress.com/2019/06/01/
// Audio Spectrum Display with M5STACK : 2017.12.31 : macsbug
//  https://macsbug.wordpress.com/2017/12/31/audio-spectrum-display-with-m5stack/
// https://github.com/tobozo/ESP32-8-Octave-Audio-Spectrum-Display/tree/wrover-kit
// https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display
// https://github.com/kosme/arduinoFFT
 
 
// David Bird (https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display)
// tobozo (https://github.com/tobozo/ESP32-Audio-Spectrum-Waveform-Display)
// macsbug (https://macsbug.wordpress.com/)
// KIRA Ryouta (https://github.com/KKQ-KKQ/m5stickc-audiospectrum)


#pragma GCC optimize ("O3")
#include <arduinoFFT.h>
#include <M5StickC.h>
#include <driver/i2s.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include "dywapitchtrack.h"

#define PIN_CLK  0
#define PIN_DATA 34
#define SAMPLES 1024 // Must be a power of 2
#define READ_LEN (2 * SAMPLES)
#define TFT_WIDTH 160
#define TFT_HEIGHT 80
#define BANDS 8
#define BANDS_WIDTH ( TFT_WIDTH / BANDS )
#define BANDS_PADDING 8
#define BAR_WIDTH ( BANDS_WIDTH - BANDS_PADDING )
#define ATTENUATION 2.5
#define NOISE_FLOOR 316.227766017 // pow(10, ATTENUATION)
#define MAGNIFY 23
#define OSC_NOISEFLOOR 100
#define OSC_SAMPLES DYWAPT_SAMPLESIZE
#define OSC_SKIPCOUNT (OSC_SAMPLES/SAMPLES)
#define OSC_EXTRASKIP 1

//#define MAXBUFSIZE (2*SAMPLES)
#define MAXBUFSIZE OSC_SAMPLES

arduinoFFT FFT = arduinoFFT();  

struct eqBand {
  const char *freqname;
  int peak;
  int lastpeak;
  uint16_t lastval;
};

enum : uint8_t {
  ModeSpectrumBars,
  ModeOscilloscope,
  ModeTuner,
  ModeCount,
};

static uint8_t runmode = 0;

static bool semaphore = false;
static bool needinit = true;

static eqBand audiospectrum[BANDS] = {
  // freqname,peak,lastpeak,lastval,
  { ".1k", 0 },
  { ".2k", 0 },
  { ".5k", 0 },
  { " 1k", 0 },
  { " 2k", 0 },
  { " 4k", 0 },
  { " 8k", 0 },
  { "16k", 0 }
};
 
static double vTemp[2][MAXBUFSIZE];
static uint8_t curbuf = 0;
static uint16_t colormap[TFT_HEIGHT];//color palette for the band meter(pre-fill in setup)

static dywapitchtracker pitchTracker;

static int bufposcount = 0;

static const char *notestr[12] = {
  "C ", "C#", "D ", "D#", "E ", "F ", "F#", "G ", "G#", "A ", "A#", "B "
};

static uint16_t oscbuf[2][OSC_SAMPLES];
static int skipcount = 0;

void i2sInit(){
   i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_PDM),
    .sample_rate =  44100,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT, //is fixed at 12bit,stereo,MSB
    .channel_format = I2S_CHANNEL_FMT_ALL_RIGHT,
    .communication_format = I2S_COMM_FORMAT_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 2,
    .dma_buf_len = 128,
   };
   i2s_pin_config_t pin_config;
   pin_config.bck_io_num   = I2S_PIN_NO_CHANGE;
   pin_config.ws_io_num    = PIN_CLK;
   pin_config.data_out_num = I2S_PIN_NO_CHANGE;
   pin_config.data_in_num  = PIN_DATA; 
   i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
   i2s_set_pin(I2S_NUM_0, &pin_config);
   i2s_set_clk(I2S_NUM_0, 44100,I2S_BITS_PER_SAMPLE_16BIT,I2S_CHANNEL_MONO);
}
 
void setup() {
  M5.begin();
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextSize(1);

  i2sInit();
 
  for(uint8_t i=0;i<TFT_HEIGHT;i++) {
    //colormap[i] = M5.Lcd.color565(TFT_HEIGHT-i*.5,i*1.1,0); //RGB
    //colormap[i] = M5.Lcd.color565(TFT_HEIGHT-i*4.4,i*2.5,0);//RGB:rev macsbug
    double r = TFT_HEIGHT - i;
    double g = i;
    double mag = (r > g)? 255./r : 255./g;
    r *= mag;
    g *= mag;
    colormap[i] = M5.Lcd.color565((uint8_t)r,(uint8_t)g,0); // Modified by KKQ-KKQ
  }
  xTaskCreatePinnedToCore(looptask,"calctask",32768,NULL,1,NULL,1);
}

void initMode() {
  M5.Lcd.fillRect(0, 0,
                  TFT_WIDTH, TFT_HEIGHT, BLACK);
  switch (runmode) {
    case ModeSpectrumBars:
      M5.Lcd.setTextSize(1);
      M5.Lcd.setTextColor(LIGHTGREY);
      for (byte band = 0; band < BANDS; band++) {
        M5.Lcd.setCursor(BANDS_WIDTH*band + 2, 0);
        M5.Lcd.print(audiospectrum[band].freqname);
      }
      break;

    case ModeOscilloscope:
      {
        M5.Lcd.setTextColor(GREEN);
        dywapitch_inittracking(&pitchTracker);
      }
      break;

    case ModeTuner:
      M5.Lcd.setTextSize(1);
      break;
  }
}

void showSpectrumBars(){
  double *vTemp_ = vTemp[curbuf^1];

  FFT.Windowing(vTemp_, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vTemp_, vTemp_ + SAMPLES, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vTemp_, vTemp_ + SAMPLES, SAMPLES);

  double values[BANDS] = {};
  for (int i = 2; i < (SAMPLES/2); i++){ 
    // Don't use sample 0 and only first SAMPLES/2 are usable. 
    // Each array element represents a frequency and its value the amplitude.
    if (vTemp_[i] > NOISE_FLOOR) {
      byte bandNum = getBand(i);
      if(bandNum != 8 && vTemp_[i] > values[bandNum]) {
        values[bandNum] = vTemp_[i];
      }
    }
  }
  for (byte band = 0; band < BANDS; band++) {
    displayBand(band, (log10(values[band]) - ATTENUATION) * MAGNIFY);
  }
  //long vnow = millis();
  for (byte band = 0; band < BANDS; band++) {
    if (audiospectrum[band].peak > 0) {
      audiospectrum[band].peak -= 2;
    }
    if(audiospectrum[band].peak <= 0) {
      audiospectrum[band].peak = 0;
    }
    // only draw if peak changed
    if(audiospectrum[band].lastpeak != audiospectrum[band].peak) {
      // delete last peak
      uint16_t hpos = BANDS_WIDTH*band + (BANDS_PADDING/2);
      M5.Lcd.drawFastHLine(hpos,TFT_HEIGHT-audiospectrum[band].lastpeak,BAR_WIDTH,BLACK);
      audiospectrum[band].lastpeak = audiospectrum[band].peak;
      uint16_t ypos = TFT_HEIGHT - audiospectrum[band].peak;
      M5.Lcd.drawFastHLine(hpos, ypos,
                           BAR_WIDTH, colormap[ypos]);
    }
  } 
}

void displayBand(int band, int dsize){
  uint16_t hpos = BANDS_WIDTH*band + (BANDS_PADDING/2);
  if (dsize < 0) dsize = 0;
#define dmax 200
  if(dsize>TFT_HEIGHT-10) {
    dsize = TFT_HEIGHT-10; // leave some hspace for text
  }
  if(dsize < audiospectrum[band].lastval) {
    // lower value, delete some lines
    M5.Lcd.fillRect(hpos, TFT_HEIGHT-audiospectrum[band].lastval,
                    BAR_WIDTH, audiospectrum[band].lastval - dsize,BLACK);
  }
  if (dsize > dmax) dsize = dmax;
  for (int s = 0; s <= dsize; s=s+4){
    uint16_t ypos = TFT_HEIGHT - s;
    M5.Lcd.drawFastHLine(hpos, ypos, BAR_WIDTH, colormap[ypos]);
  }
  if (dsize > audiospectrum[band].peak){audiospectrum[band].peak = dsize;}
  audiospectrum[band].lastval = dsize;
}

byte getBand(int i) {
  if (i >= 2   && i < 4  ) return 0;  // 125Hz
  if (i >= 4   && i < 8  ) return 1;  // 250Hz
  if (i >= 8   && i < 16 ) return 2;  // 500Hz
  if (i >= 16  && i < 32 ) return 3;  // 1000Hz
  if (i >= 32  && i < 64 ) return 4;  // 2000Hz
  if (i >= 64  && i < 128) return 5;  // 4000Hz
  if (i >= 128 && i < 256) return 6;  // 8000Hz
  if (i >= 256 && i < 512) return 7;  // 16000Hz
  return 8;
}

double calcNumSamples(double f) {
  if (f == 0.0) return SAMPLES/2;
  double s = (double)(44100 * 2) / f;
  if (s > (double)OSC_SAMPLES) {
    do {
      s *= 0.5;
    } while (s > (double)OSC_SAMPLES);
  }
  return s;
}

void showFreq(double freq) {
  if (freq > 0) {
    char strbuf[16];
    sprintf(strbuf, "%8.2fHz", freq);
    M5.Lcd.drawString(strbuf, 0, 0, 1);
    double fnote = log2(freq)*12 - 36.376316562;
    int note = fnote + 0.5;
    if (note >= 0) {
      M5.Lcd.setCursor(TFT_WIDTH/2 - 4, 0);
      M5.Lcd.print(notestr[note % 12]);
      M5.Lcd.print(note / 12 - 1);
      double cent = (fnote - note) * 100;
      sprintf(strbuf, "%.1fcents", cent);
      M5.Lcd.drawRightString(strbuf, TFT_WIDTH, 0, 1);
    }
  }
}

void showOscilloscope()
{
  uint16_t i,j;
  uint16_t *oscbuf_ = oscbuf[curbuf^1];
  double *vTemp_ = vTemp[curbuf ^ 1];
  double freq = dywapitch_computepitch(&pitchTracker, vTemp_);
  if (skipcount < OSC_EXTRASKIP) {
    ++skipcount;
    return;
  }
  skipcount = 0;
  uint16_t s = calcNumSamples(freq);
  double mx = (double)TFT_WIDTH / s;
  double my;
  uint16_t maxV = 0;
  uint16_t minV = 65535;
  for (i = 0; i < s; ++i) {
    if (maxV < oscbuf_[i]) maxV = oscbuf_[i];
    if (minV > oscbuf_[i]) minV = oscbuf_[i];
  }
  if (maxV - minV > OSC_NOISEFLOOR) {
    my = (double)(TFT_HEIGHT-10) / (maxV - minV);
  }
  else {
    my = (double)(TFT_HEIGHT-10) / OSC_NOISEFLOOR;
    minV = (((int)maxV + (int)minV) >> 1) - OSC_NOISEFLOOR/2;
  }
  M5.Lcd.fillRect(0, 0,
                  TFT_WIDTH, TFT_HEIGHT, BLACK);
  uint16_t y = TFT_HEIGHT - (oscbuf_[0] - minV) * my;
  for (i = 0; i < s; ++i) {
    uint16_t y2 = TFT_HEIGHT - (oscbuf_[i] - minV) * my;
    M5.Lcd.drawLine((uint16_t)(i * mx), y,
                    (uint16_t)((i+1)*mx), y2, LIGHTGREY);
    y = y2;
  }
  showFreq(freq);
}

void showTuner() {
  double *vTemp_ = vTemp[curbuf ^ 1];
  double freq = dywapitch_computepitch(&pitchTracker, vTemp_);
  double fnote;
  int note;
  if (freq > 0) {
    fnote = log2(freq)*12 - 36.376316562;
    note = fnote + 0.5;
  }
  else {
    note = -1;
  }
  uint32_t bgcolor, fgcolor;
  if (note >= 0) {
    double cent = (fnote - note) * 100;
    if (abs(cent) < 2.) {
      bgcolor = GREEN;
      fgcolor = BLACK;
    }
    else {
      bgcolor = DARKGREY;
      fgcolor = BLACK;
    }
    M5.Lcd.fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, bgcolor);
    M5.Lcd.setTextColor(fgcolor);
    M5.Lcd.drawRect(2, 36, TFT_WIDTH-3, TFT_HEIGHT-40, fgcolor);
    M5.Lcd.drawLine(TFT_WIDTH/2 + 1, 36, TFT_WIDTH/2 + 1, TFT_HEIGHT - 4, fgcolor);
    M5.Lcd.fillCircle(((double)TFT_WIDTH/2 + 1) + cent * ((double)(TFT_WIDTH-3)/100), (TFT_HEIGHT+34)/2, 5, fgcolor);
    char strbuf[8];
    sprintf(strbuf, "%s%d", notestr[note % 12], note / 12 - 1);
    M5.Lcd.drawCentreString(strbuf, TFT_WIDTH/2, 3, 4);
  }
  else {
    M5.Lcd.fillRect(0, 0, TFT_WIDTH, TFT_HEIGHT, DARKGREY);
    M5.Lcd.drawRect(2, 36, TFT_WIDTH-3, TFT_HEIGHT-40, BLACK);
    M5.Lcd.drawLine(TFT_WIDTH/2 + 1, 36, TFT_WIDTH/2 + 1, TFT_HEIGHT - 4, BLACK);
  }
}

void looptask(void *) {
  while (1) {
    if (needinit) {
      initMode();
      needinit = false;
    }
    if (semaphore) {
      switch(runmode) {
        case ModeSpectrumBars:
          showSpectrumBars();
          break;

        case ModeOscilloscope:
          showOscilloscope();
          break;

        case ModeTuner:
          showTuner();
          break;
      }
      semaphore = false;
    }
    else {
      vTaskDelay(10);
    }
  }
}

void loop() {
  M5.update();
  if (M5.BtnA.wasReleased()) {
    ++runmode;
    if (runmode >= ModeCount) runmode = 0;

    bufposcount = 0;
    needinit = true;;
  }
  uint16_t i,j;
  j = bufposcount * SAMPLES;
  uint16_t *adcBuffer = &oscbuf[curbuf][j];
  i2s_read_bytes(I2S_NUM_0,(char*)adcBuffer,READ_LEN,500);
  int32_t dc = 0;
  for (int i = 0; i < SAMPLES; ++i) {
    dc += adcBuffer[i];
  }
  dc /= SAMPLES;

  switch(runmode) {
    case ModeSpectrumBars:
      for (int i = 0; i < SAMPLES; ++i) {
        
        vTemp[curbuf][i] = (int)adcBuffer[i] - dc;
        vTemp[curbuf][i + SAMPLES] = 0;
      }
      curbuf ^= 1;
      semaphore = true;
      break;
    case ModeOscilloscope:
      for (i = 0; i < SAMPLES; ++i) {
        vTemp[curbuf][i + j] = (int)adcBuffer[i] - dc;
      }
      if (++bufposcount >= OSC_SKIPCOUNT) {
        bufposcount = 0;
        curbuf ^= 1;
        semaphore = true;
      }
      break;
    case ModeTuner:
      j = bufposcount * SAMPLES;
      for (i = 0; i < SAMPLES; ++i) {
        vTemp[curbuf][i + j] = (int)adcBuffer[i] - dc;
      }
      if (++bufposcount >= OSC_SKIPCOUNT) {
        bufposcount = 0;
        curbuf ^= 1;
        semaphore = true;
      }
      break;
  }
}
