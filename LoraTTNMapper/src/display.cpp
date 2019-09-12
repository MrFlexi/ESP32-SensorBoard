#include "globals.h"

HAS_DISPLAY u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE, /* clock=*/ SCL, /* data=*/ SDA);   // ESP32 Thing, HW I2C with pin remapping
U8G2LOG u8g2log;
uint8_t u8log_buffer[U8LOG_WIDTH * U8LOG_HEIGHT];

void log_display(String s)
{
  Serial.println(s);
  if (runmode < 1)
  {
    u8g2log.print(s);
    u8g2log.print("\n");
  }
}

void setup_display(void)
{
  u8g2.begin();
  u8g2.setFont(u8g2_font_profont11_mf);                         // set the font for the terminal window
  u8g2log.begin(u8g2, U8LOG_WIDTH, U8LOG_HEIGHT, u8log_buffer); // connect to u8g2, assign buffer
  u8g2log.setLineHeightOffset(0);                               // set extra space between lines in pixel, this can be negative
  u8g2log.setRedrawMode(0);                                     // 0: Update screen with newline, 1: Update screen for every char
  u8g2.enableUTF8Print();
  log_display("SAP GTT");
  log_display("TTN-ABP-Mapper");
}



void showPage(int page)
{


  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_ncenB14_tr);  
  u8g2.drawStr(1,15,"   SAP GTT  ");
    
  u8g2.setFont(u8g2_font_profont11_mf);
  u8g2.setCursor(1, 30); u8g2.printf("Sats:%.2d", gps.tGps.satellites.value());
  u8g2.setCursor(64, 30 );u8g2.printf("%02d:%02d:%02d", gps.tGps.time.hour(), gps.tGps.time.minute(), gps.tGps.time.second());

  u8g2.setCursor(1, 40); u8g2.printf("Alt:%.2d", gps.tGps.altitude.meters());  
  u8g2.setCursor(1, 50); u8g2.printf("Len:%.2d", dataBuffer.data.lmic.dataLen); 
  
  #if (defined BAT_MEASURE_ADC || defined HAS_PMU)
    u8g2.setCursor(1, 60);
    u8g2.printf("B:%.2fV", dataBuffer.data.bat_voltage / 1000.0);
#endif
  
  
  u8g2.sendBuffer();
}
  

DataBuffer::DataBuffer()
{ 
}

void DataBuffer::set(bmeStatus_t input  )
{  
  data = input;
}

void DataBuffer::get()
{  
}

 DataBuffer dataBuffer;
 bmeStatus_t sensorValues;