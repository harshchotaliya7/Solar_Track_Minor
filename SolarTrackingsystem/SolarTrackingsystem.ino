#include "Arduino.h"
#include "uRTCLib.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LCD_I2C.h>

/************* Disclaimer ********************* 
*  Minor Project 2023 
*  By Harsh Chotaliya (20bec039@nirmauni.ac.in)
*  By Kinjal Ahuja (20bec057@nirmauni.ac.in)
***********************************************
*  Here RTC is used because whenever GPS is not  
*  there then RTC is used for timining.
*/

///////////////////// Timezone offset for gps timing.
#define  timeZoneOffsetHours    6  
#define  timeZoneOffsetMinutes  -30

///////////////////// Row and column of lcd
#define row_lcd   4
#define col_lcd   20

///////////////////// Variable for function of print
#define GPS_    0
#define RTC_    1

#define LCD_    0
#define SERIAL_  1

//////////////////////  I2C LCD 
LCD_I2C lcd(0xA7,col_lcd,row_lcd); 

///////////////////// RTC I2C ADDR - 0x68
uRTCLib rtc(0x68);

int prev_hour=0,prev_minute=0,prev_second=0;

//////////////////// Weak Days array
char daysOfTheWeek[7][12] = {"Sun", "Monday", "Tuesday", "Wednesday", "Thursday", "Fri", "Saturday"};

////////////////// GPS Module (runs on Serial1)
const uint32_t GPSBaud = 9600; 

int localMinutes,localHours;
////////////////// UTC to local time converter function 
int UTCtoLocalTime(int hour, int minute);

/////////////////// LCD/Serial Printing Function
void Print_data(bool L_S,bool gpsorrtc);

/////////////////// Sun position Function
void Calculate_Sun_Position(int hour , int minute , int second , int day , int month , int year);

////////////////// Julian Date Function
long JulianDate(int year, int month, int day);

TinyGPSPlus gps; 

void setup() {

  ///////////////// Serial Monitor
  Serial.begin(9600);
  
  ///////////////// GPS Module 
  Serial1.begin(GPSBaud);

  ///////////////// LCD Initialization
  lcd.begin(); 
  lcd.backlight();

  ///////////////// RTC Initalization
  URTCLIB_WIRE.begin();
  
  ///////////////// First time manually set the RTC data when POWER ON
  //rtc.set(0,20,11,1,22,10,23);
}

void loop() {
  if (Serial1.available() > 0) 
  {
    ///// print only if gps data is encoded correctly

    if (gps.encode(Serial1.read())) 
    {
      ///// also only print correct data if location is valid 
      if (gps.location.isValid()) 
      {
        Print_data(LCD_,GPS_);
      } 

      ///// other wise print invalid 
      else 
      {
        lcd.print(F("- location: INVALID"));
        Serial.print("location invalid");
      }
    }
   }
      ////// RTC to refresh every time 
      rtc.refresh();
 
      /******************************************************
      ** To view RTC Data on Serial Monitor / LCD ***********
      ******************************************************/ 
      Print_data(LCD_,RTC_);

      prev_hour=rtc.hour();
      prev_minute=rtc.minute();
      prev_second=rtc.second();

}

int UTCtoLocalTime(int hour, int minute)
{
  // Calculate the local time
  localHours = hour + timeZoneOffsetHours;
  localMinutes = minute + timeZoneOffsetMinutes;

  // Adjust for negative minutes or overflow
  if (localMinutes < 0) {
    localMinutes += 60;
    localHours -= 1;
  }
  if (localMinutes >= 60) {
    localMinutes -= 60;
    localHours += 1;
  }
}

void Calculate_Sun_Position(int hour , int minute , int second , int day , int month , int year)
{
  float T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JDate, JDx;

  JDate = JulianDate(year,month,day);
}

long JulianDate(int year, int month, int day)
{
  long JDate;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JDate = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524; 
  return JDate;
}

void Print_data(bool L_S,bool gpsorrtc)
{
    if(L_S == LCD_)
    {
      if(gpsorrtc==GPS_)
      { 
        lcd.setCursor(0, 0);
        lcd.print("Latitude:") ;
        lcd.setCursor(10, 0);
        lcd.print(gps.location.lat());
        lcd.setCursor(0, 1);
        lcd.print("Longitude:") ;
        lcd.setCursor(11, 1);
        lcd.print(gps.location.lng());
      }
      else
      {
        lcd.setCursor(0, 2);
        lcd.print("Date:");
        lcd.print(rtc.day());
        lcd.print('/');
        lcd.print(rtc.month());
        lcd.print('/');
        lcd.print(rtc.year());
        lcd.print(" (");
        lcd.print(daysOfTheWeek[rtc.dayOfWeek()-1]);
        lcd.print(") ");

        lcd.setCursor(0,3);
        lcd.print("Time:");
        lcd.setCursor(5, 3);
        lcd.print(rtc.hour());
        lcd.setCursor(7,3);
        lcd.print(':');
        lcd.print(rtc.minute());
        lcd.setCursor(10,3);
        lcd.print(':');
        lcd.print(rtc.second());
        lcd.setCursor(14,3);
        lcd.print("(");
        lcd.print(rtc.temp() / 100);
        lcd.print("\337C");
        lcd.print(")");
        if( (rtc.hour() != prev_hour) | (rtc.minute() != prev_minute) | (rtc.second() != prev_second) )
        {
          lcd.clear();
        }
      }
  }
  else
  {
    if(gpsorrtc==GPS_)
    {
      Serial.print(" latitude : ");
      Serial.print(gps.location.lat());
      Serial.print(" longitude : ");
      Serial.println(gps.location.lng());
    } 
    else
    {
      Serial.print("Date :");
      Serial.print(rtc.day());
      Serial.print('/');
      Serial.print(rtc.month());
      Serial.print('/');
      Serial.print(rtc.year());
      Serial.print(" (");
      Serial.print(daysOfTheWeek[rtc.dayOfWeek()-1]);
      Serial.println(") ");

      Serial.print("Time :");
      Serial.print(rtc.hour());
      Serial.print(':');
      Serial.print(rtc.minute());
      Serial.print(':');
      Serial.println(rtc.second());

      Serial.print("Temperature: ");
      Serial.print(rtc.temp()  / 100);
      Serial.print("\xC2\xB0");  
      Serial.println("C");
      Serial.println("--------------------------------");
      delay(900);
    }
  }
}