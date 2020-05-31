#include "globals.h"

SSOLED ssoled;


// display helper functions
void dp_printf(uint16_t x, uint16_t y, uint8_t font, uint8_t inv,
               const char *format, ...) {
  char loc_buf[64];
  char *temp = loc_buf;
  va_list arg;
  va_list copy;
  va_start(arg, format);
  va_copy(copy, arg);
  int len = vsnprintf(temp, sizeof(loc_buf), format, copy);
  va_end(copy);
  if (len < 0) {
    va_end(arg);
    return;
  };
  if (len >= sizeof(loc_buf)) {
    temp = (char *)malloc(len + 1);
    if (temp == NULL) {
      va_end(arg);
      return;
    }
    len = vsnprintf(temp, len + 1, format, arg);
  }
  va_end(arg);
  oledWriteString(&ssoled, 0, x, y, temp, font, inv, false);
  if (temp != loc_buf) {
    free(temp);
  }
}

void log_display(String s)
{
  Serial.println(s);
  if (runmode < 1)
  {
    //u8g2log.print(s);
    //u8g2log.print("\n");
  }
}

void setup_display(void)
{
  int rc;
// The I2C SDA/SCL pins set to -1 means to use the default Wire library
// If pins were specified, they would be bit-banged in software
// This isn't inferior to hw I2C and in fact allows you to go faster on certain CPUs
// The reset pin is optional and I've only seen it needed on larger OLEDs (2.4")
//    that can be configured as either SPI or I2C
//
// oledInit(SSOLED *, type, oled_addr, rotate180, invert, bWire, SDA_PIN, SCL_PIN, RESET_PIN, speed)

rc = oledInit(&ssoled, MY_OLED, OLED_ADDR, FLIP180, INVERT, USE_HW_I2C, SDA_PIN, SCL_PIN, RESET_PIN, 400000L); // use standard I2C bus at 400Khz
  if (rc != OLED_NOT_FOUND)
  {
    char *msgs[] = {(char *)"SSD1306 @ 0x3C", (char *)"SSD1306 @ 0x3D",(char *)"SH1106 @ 0x3C",(char *)"SH1106 @ 0x3D"};
    oledFill(&ssoled, 0, 1);
    oledWriteString(&ssoled, 0,0,0,msgs[rc], FONT_NORMAL, 0, 1);
    oledSetBackBuffer(&ssoled, ucBackBuffer);
    delay(2000);
    oledWriteString(&ssoled, 0,1,1,(char *)"Sensor Board", FONT_NORMAL, 0, 1);
  }
}

void display_sample() {
  // put your main code here, to run repeatedly:
int i, x, y;
char szTemp[32];
unsigned long ms;

  oledFill(&ssoled, 0x0, 1);
  oledWriteString(&ssoled, 0,16,0,(char *)"ss_oled Demo", FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,(char *)"Written by Larry Bank", FONT_SMALL, 1, 1);
  oledWriteString(&ssoled, 0,0,3,(char *)"**Demo**", FONT_LARGE, 0, 1);
  delay(2000);
  
 // Pixel and line functions won't work without a back buffer
#ifdef USE_BACKBUFFER
  oledFill(&ssoled, 0, 1);
  oledWriteString(&ssoled, 0,0,0,(char *)"Backbuffer Test", FONT_NORMAL,0,1);
  oledWriteString(&ssoled, 0,0,1,(char *)"3000 Random dots", FONT_NORMAL,0,1);
  delay(2000);
  oledFill(&ssoled, 0,1);
  ms = millis();
  for (i=0; i<3000; i++)
  {
    x = random(OLED_WIDTH);
    y = random(OLED_HEIGHT);
    oledSetPixel(&ssoled, x, y, 1, 1);
  }
  ms = millis() - ms;
  sprintf(szTemp, "%dms", (int)ms);
  oledWriteString(&ssoled, 0,0,0,szTemp, FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,(char *)"Without backbuffer", FONT_SMALL,0,1);
  delay(2000);
  oledFill(&ssoled, 0,1);
  ms = millis();
  for (i=0; i<3000; i++)
  {
    x = random(OLED_WIDTH);
    y = random(OLED_HEIGHT);
    oledSetPixel(&ssoled, x, y, 1, 0);
  }
  oledDumpBuffer(&ssoled, NULL);
  ms = millis() - ms;
  sprintf(szTemp, "%dms", (int)ms);
  oledWriteString(&ssoled, 0,0,0,szTemp, FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,(char *)"With backbuffer", FONT_SMALL,0,1);
  delay(2000);
  oledFill(&ssoled, 0, 1);
  oledWriteString(&ssoled, 0,0,0,(char *)"Backbuffer Test", FONT_NORMAL,0,1);
  oledWriteString(&ssoled, 0,0,1,(char *)"96 lines", FONT_NORMAL,0,1);
  delay(2000);
  ms = millis();
  for (x=0; x<OLED_WIDTH-1; x+=2)
  {
    oledDrawLine(&ssoled, x, 0, OLED_WIDTH-x, OLED_HEIGHT-1, 1);
  }
  for (y=0; y<OLED_HEIGHT-1; y+=2)
  {
    oledDrawLine(&ssoled, OLED_WIDTH-1,y, 0,OLED_HEIGHT-1-y, 1);
  }
  ms = millis() - ms;
  sprintf(szTemp, "%dms", (int)ms);
  oledWriteString(&ssoled, 0,0,0,szTemp, FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,(char *)"Without backbuffer", FONT_SMALL,0,1);
  delay(2000);
  oledFill(&ssoled, 0,1);
  ms = millis();
  for (x=0; x<OLED_WIDTH-1; x+=2)
  {
    oledDrawLine(&ssoled, x, 0, OLED_WIDTH-1-x, OLED_HEIGHT-1, 0);
  }
  for (y=0; y<OLED_HEIGHT-1; y+=2)
  {
    oledDrawLine(&ssoled, OLED_WIDTH-1,y, 0,OLED_HEIGHT-1-y, 0);
  }
  oledDumpBuffer(&ssoled, ucBackBuffer);
  ms = millis() - ms;
  sprintf(szTemp, "%dms", (int)ms);
  oledWriteString(&ssoled, 0,0,0,szTemp, FONT_NORMAL, 0, 1);
  oledWriteString(&ssoled, 0,0,1,(char *)"With backbuffer", FONT_SMALL,0,1);
  delay(2000);
#endif
}


void showPage(int page)
{
  
  char szTemp[32];
  oledFill(&ssoled, 0x0, 1); // Clear 

  switch (page)
  {
  case PAGE_VALUES:

    //u8g2.setFont(u8g2_font_profont11_mf);
    //u8g2.setCursor(1, 30);
    //u8g2.printf("Sats:%.2d", gps.tGps.satellites.value());
    //u8g2.setCursor(64, 30);
    //u8g2.printf("%02d:%02d:%02d", gps.tGps.time.hour(), gps.tGps.time.minute(), gps.tGps.time.second());

    //u8g2.setCursor(1, 40);
    //u8g2.printf("Alt:%.4d", gps.tGps.altitude.meters());
    
    //u8g2.setCursor(64, 50);
    //u8g2.printf("TX:%.3d", dataBuffer.data.txCounter);
    //u8g2.setCursor(1, 50);
    //u8g2.printf("Sleep:%.2d", dataBuffer.data.sleepCounter);
    
    
    //u8g2.setCursor(1, 20);
    //u8g2.printf("Ele:%.2f", dataBuffer.data.sun_elevation);

   
    //dp_printf(0, 1, FONT_NORMAL, 0, "Ele:%.2f", dataBuffer.data.sun_elevation);

    sprintf(szTemp, "Ele:%.2f", dataBuffer.data.sun_elevation);
    oledWriteString(&ssoled, 0,0,1,szTemp, FONT_NORMAL, 0, 1);

     sprintf(szTemp, "%.2f V", dataBuffer.data.busvoltage1);
    oledWriteString(&ssoled, 0,0,2,szTemp, FONT_NORMAL, 0, 1);
    
    sprintf(szTemp, "%.2f mA", dataBuffer.data.current_1);
    oledWriteString(&ssoled, 0,0,3,szTemp, FONT_NORMAL, 0, 1);
    
    //u8g2.setCursor(1, 40);
    //u8g2.printf("%.2f V", dataBuffer.data.busvoltage1);
    //u8g2.setCursor(1, 60);
    //u8g2.printf("%.2f mA", dataBuffer.data.current_1);
    

    break;

  case PAGE_SLEEP:
    oledWriteString(&ssoled, 0,0,1,(char *)"SLEEP", FONT_LARGE, 0, 1);

    
    break;
  }
  //u8g2.sendBuffer();
}

DataBuffer::DataBuffer()
{
}

void DataBuffer::set(deviceStatus_t input)
{
  data = input;
}

void DataBuffer::get()
{
}

DataBuffer dataBuffer;
deviceStatus_t sensorValues;