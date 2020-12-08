
#include <table_fft.h>
#include <cr4_fft_1024_stm32.h>

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

static const uint16_t BUFFER_SIZE = 1024; // bytes

int data[BUFFER_SIZE];
uint16_t data16[BUFFER_SIZE];
uint32_t data32[BUFFER_SIZE];
uint32_t y[BUFFER_SIZE];
uint16_t i, j;

byte ylim = 60;                                       // OLED y-axis drawing boundary limit

void real_to_complex(uint16_t * in, uint32_t * out, int len) {
  //
  for (int i = 0; i < len; i++) out[i] = in[i];// * 8;
}

uint16_t asqrt(uint32_t x) { //good enough precision, 10x faster than regular sqrt
  //
  int32_t op, res, one;
  op = x;
  res = 0;
  one = 1 << 30;
  while (one > op) one >>= 2;
  while (one != 0) {
    if (op >= res + one) {
      op = op - (res + one);
      res = res +  2 * one;
    }
    res /= 2;
    one /= 4;
  }
  return (uint16_t) (res);
}

void inplace_magnitude(uint32_t * target, uint16_t len) {
  // 
  uint16_t * p16;
  for (int i = 0; i < len; i ++) {
    //
    int16_t real = target[i] & 0xFFFF;
    int16_t imag = target[i] >> 16;
//    target[i] = 10 * log10(real*real + imag*imag);
    uint32_t magnitude = asqrt(real*real + imag*imag);
    target[i] = magnitude; 
  }
}
void setADCs ()
{
  rcc_set_prescaler(RCC_PRESCALER_ADC,RCC_ADCPRE_PCLK_DIV_2 );

  int pinMapADCin = PIN_MAP[PB0].adc_channel;
  adc_set_sample_rate(ADC1, ADC_SMPR_239-_5); //~2.7 Mhz sample rate
 
  adc_set_reg_seqlen(ADC1, 1);
  ADC1->regs->SQR3 = pinMapADCin;
  ADC1->regs->CR2 |= ADC_CR2_CONT; // | ADC_CR2_DMA; // Set continuous mode and DMA
  ADC1->regs->CR2 |= ADC_CR2_SWSTART;

  //Serial.println("Calibrating ADC1");
  //adc_calibrate(ADC1); //TODO: Calibration doesnt ever return for some odd reason...

}
uint32_t perform_fft(uint32_t * indata, uint32_t * outdata, const int len) {
  //
  cr4_fft_1024_stm32(outdata, indata, len);
  inplace_magnitude(outdata, len);
}

void setup()
{
 display.begin(SSD1306_SWITCHCAPVCC, 0x3C);  // initialize with the I2C addr 0x3C (for the 128x32) 
  display.clearDisplay();
 Serial.begin(115200);
  setADCs();
};

void loop(){ 
while(1){  
  String serialBuffer="";
  //serialBuffer.reserve(5125);
  uint16_t min=1024, max=0;
  
  for (i = 0; i < 1024; i++) {
    data16[i] = analogRead(PB0);
   if(data16[i]>max) max=data16[i];
   if(data16[i]<min) min=data16[i];
  };
    
    real_to_complex(data16, data32, BUFFER_SIZE);  // data format conversion
    perform_fft(data32, y, BUFFER_SIZE);           // FFT computation
  
  display.clearDisplay();
  for (i = 4; i < 512; i+=4) { // In the current design, 60Hz and noise
    if(y[i]<5) y[i]=0;
    display.drawLine(i/4, ylim, i/4, ylim - y[i], WHITE); 
  };
  display.display();

    for (j=0; j<=896; j+=128){
      serialBuffer = "";
      if(j==0){
        //serialBuffer += "i"; // input data serial buffer starts with an 'i'
        //serialBuffer += ":";
      }
      
        for (i = j; i < j+128; i++) {
        serialBuffer += data16[i]; // serial data is sent as a buffer with 128 values seperated by a ':'
        serialBuffer += ":"; 
        };
        
      //if(j==896){
        //serialBuffer += "p";  // input data serial buffer ends with a 'p'
      //}
      Serial.println(serialBuffer);
    }

    /*
    for (j=0; j<=384; j+=128){
      serialBuffer = "";
      //if(j==0){
        //serialBuffer += "f"; // fft data serial buffer starts with an 'f'
       // serialBuffer += ":";
     // }
      
        for (i = j; i < j+128; i++) {
        serialBuffer += y[i]; // serial data is sent as a buffer with 128 values seperated by a ':'
        serialBuffer += ":"; 
        };
        
      if(j==384){
        serialBuffer += "t";  // fft data serial buffer ends with a 't'
      }
      Serial.println(serialBuffer);
    }*/
    
   /*for (i = 2; i < 256; i+=2) { // In the current design, 60Hz and noise
      //Serial.println(y[i]);
  };*/

  /*
  display.drawLine(0, 10, data[0], 10, WHITE);
  display.drawLine(0, 11, data[0], 11, WHITE);
  display.display();
  display.setCursor(0,0);
  display.print("min=");
  display.print(min,1);
  display.setCursor(8*6,0);
  display.print("max=");
  display.print(max);
  display.setCursor(16*6,0);
  display.print("v=");
  display.print((int)data[0]);
  display.display();
  */
};
}
