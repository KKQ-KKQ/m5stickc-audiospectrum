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

// Modified on 2020.07.07 by KIRA Ryouta
// https://github.com/KKQ-KKQ/m5stickc-audiospectrum

// M5StickC Audio Spectrum             : 2019.06.01 : macsbug
//  https://macsbug.wordpress.com/2019/06/01/
// Audio Spectrum Display with M5STACK : 2017.12.31 : macsbug
//  https://macsbug.wordpress.com/2017/12/31/audio-spectrum-display-with-m5stack/
// https://github.com/tobozo/ESP32-8-Octave-Audio-Spectrum-Display/tree/wrover-kit
// https://github.com/G6EJD/ESP32-8266-Audio-Spectrum-Display
// https://github.com/kosme/arduinoFFT
 
#pragma GCC optimize ("O3")
#include <arduinoFFT.h>
#include <M5StickC.h>
#include <driver/i2s.h>
#include <math.h>

arduinoFFT FFT = arduinoFFT();  
#define PIN_CLK  0
#define PIN_DATA 34
#define READ_LEN (2 * 1024)
static uint8_t buffer[READ_LEN] = {0};
#define SAMPLES 1024         // Must be a power of 2
struct eqBand {
  const char *freqname;
  int peak;
  int lastpeak;
  uint16_t lastval;
};

#define TFT_WIDTH 160
#define TFT_HEIGHT 80
#define BANDS 8
#define BANDS_WIDTH ( TFT_WIDTH / BANDS )
#define BANDS_PADDING 8
#define BAR_WIDTH ( BANDS_WIDTH - BANDS_PADDING )
#define ATTENUATION 2.5
#define NOISE_FLOOR 316.227766017 // pow(10, ATTENUATION)
#define MAGNIFY 23
 
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
 
static double vReal[SAMPLES];
static double vImag[SAMPLES];
static uint16_t colormap[TFT_HEIGHT];//color palette for the band meter(pre-fill in setup)
 
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
  setCpuFrequencyMhz(160);
  M5.Lcd.setRotation(1);
  M5.Lcd.fillScreen(BLACK);
  M5.Lcd.setTextColor(LIGHTGREY);
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
  for (byte band = 0; band < BANDS; band++) {
    M5.Lcd.setCursor(BANDS_WIDTH*band + 2, 0);
    M5.Lcd.print(audiospectrum[band].freqname);
  }
}
 
void showSignal(const uint16_t *adcBuffer){
  for (int i = 0; i < SAMPLES; i++) {
    vReal[i] = adcBuffer[i];
    vImag[i] = 0;
  }
   
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

  double values[BANDS] = {};
  for (int i = 2; i < (SAMPLES/2); i++){ 
    // Don't use sample 0 and only first SAMPLES/2 are usable. 
    // Each array element represents a frequency and its value the amplitude.
    if (vReal[i] > NOISE_FLOOR) {
      byte bandNum = getBand(i);
      if(bandNum != 8 && vReal[i] > values[bandNum]) {
        values[bandNum] = vReal[i];
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

void loop() {
    i2s_read_bytes(I2S_NUM_0,(char*)buffer,READ_LEN,(500/portTICK_RATE_MS));
    showSignal((uint16_t *)buffer);
}
