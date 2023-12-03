#include "Arduino.h"
#include "uRTCLib.h"
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LCD_I2C.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>


/************* Disclaimer ********************* 
*  Minor Project 2023 
*  By Harsh Chotaliya (20bec039@nirmauni.ac.in)
*  By Kinjal Ahuja (20bec057@nirmauni.ac.in)
***********************************************
*  Here RTC is used because whenever GPS is not  
*  there then RTC is used for timining.
***********************************************
* Serial  : Used for Serial Monitor
* Serial1 : Used for GPS - data
* Serial2 : Used for Motor Driver
* I2C : Addr - 0xA7 : Used for LCD 20x4
*              0x68 : Used for RTC
*              0x28 : Used for BNO055 - 9 axis Gyroscope
*/  

///////////////// If offset Required in Elevation & Azimuth for correction with actual data from website
#define elevation_offset 0.47
#define azimuth_offset 1.16

///////////////// Default value (Nirma University) if GPS signal is not found 
#define default_long  72.54420  
#define default_lat   23.12788

///////////////////// degree to radian
#define DEG_TO_RAD   0.01744   
#define RAD_TO_DEG   57.32484 

///////////////////// Timezone offset for gps timing.
#define  timeZoneOffsetHours    -6  
#define  timeZoneOffsetMinutes  30

///////////////////// Row and column of lcd
#define row_lcd   4
#define col_lcd   20

///////////////////// Variable for function of print
#define GPS_    0
#define RTC_    1

//////////////////// Variable to print data whether on LCD or Serial Monitor
#define LCD_    0
#define SERIAL_  1

#define BNO055_SAMPLERATE_DELAY_MS (100)

Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

//////////////////////  I2C LCD 
LCD_I2C lcd(0xA7,col_lcd,row_lcd); 

///////////////////// RTC I2C ADDR - 0x68
uRTCLib rtc(0x68);

//////////////////// Variable for Refreshing the LCD data
int prev_hour=0,prev_minute=0,prev_second=0;

//////////////////// Variable to hold the value of latitiude or longitude
double latitude_=0,longitude_=0;

//////////////////// Variable to hold current Elevation angle & Azimuth angle  
double ele_ang, azi_ang;

//////////////////// Variable to hold Julian Date Value
long JDate;

//////////////////// Varible to hold the value of every seconds (Used when we want to rotate the Solar Panel on particular interval of time)
long seconds=0;

///////////////// Variable to hold actual x_axis and y_axis angle 
double current_x_axis_angle,current_y_axis_angle;

//////////////////// Weak Days array
char daysOfTheWeek[7][12] = {"Sun", "Monday", "Tuesday", "Wed", "Thursday", "Fri", "Saturday"};

////////////////// GPS Module (runs on Serial1)
const uint32_t GPSBaud = 9600; 

////////////////// Variable to hold UTC Time
int UTCMinutes,UTCHours,dayofyear;

////////////////// UTC to local time converter function 
int LocaltoUTCTime(int hour, int minute);

/////////////////// LCD/Serial Printing Function
void Print_data(bool L_S,bool gpsorrtc);

/////////////////// Sun position Function
void Calculate_Sun_Position(int hour , int minute , int second , int day , int month , int year, float longitude , float latitude);

////////////////// day of the year
void day_of_the_year(int day, int month, int year);

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

  ///////////////// Sabertooth 
  Serial2.begin(19200);
  Serial2.write(64);

/////////// 64 - forward positive 
/////////// 64 - backward negative

/////////// 191 - forward negative
/////////// 191 - backward positive


  while (!Serial) delay(10);  // wait for serial port to open!
  if(!bno.begin())
  {
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }
  
  ///////////////// First time manually set the RTC data when POWER ON
  //rtc.set(50,52,20,6,01,12,23);
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
        latitude_=gps.location.lat();
        longitude_=gps.location.lng();
      } 
      ///// other wise print invalid 
      else 
      {
        latitude_= default_lat;
        longitude_ = default_long;
        lcd.print(F("- location: INVALID"));
        Serial.print("location invalid");
      }
      
    }
       
   }
   
  rtc.refresh();

  /*********************************************************************************
  ***** Local Time to UTC Time Convert for Sun position calulation *****************
  *********************************************************************************/
  
  LocaltoUTCTime(rtc.hour(),rtc.minute());
  
  /******************************************************
  ************* Sun position Calculation ****************
  ******************************************************/
  
  Calculate_Sun_Position(UTCHours,UTCMinutes,rtc.second(),rtc.day(),rtc.month(),rtc.year(),default_long,default_lat);
  
  sensors_event_t event;
  bno.getEvent(&event);
  current_x_axis_angle=event.orientation.y;
  current_y_axis_angle=event.orientation.z;

  if(current_y_axis_angle>180)
  {
    current_y_axis_angle=current_y_axis_angle-360;
  }

  Serial.print(current_x_axis_angle);
  Serial.print("     ");
  Serial.println(current_y_axis_angle);

  desire_angle_motor_direction(-20,ele_ang);
  
  /******************************************************
  ************* To view Elevation Data on LCD ***********
  ******************************************************/ 

  lcd.setCursor(0,0);
  lcd.print("elevation:");
  lcd.setCursor(12, 0);
  lcd.print(ele_ang);
  lcd.print("\337");

  /******************************************************
  ************* To view Azimuth Data on LCD *************
  ******************************************************/

  lcd.setCursor(0,1);
  lcd.print("azimuth:");
  lcd.setCursor(12,1);
  lcd.print(azi_ang);
  lcd.print("\337");

  /******************************************************
  ************* To view UTC Time on LCD *****************
  ******************************************************/

  lcd.setCursor(0,2);
  lcd.print("UTC Time:");
  lcd.setCursor(12,2);
  if(UTCHours<10)
  {
    lcd.print("0");
  }
  lcd.print(UTCHours);
  lcd.print(":");
  if(UTCMinutes<10)
  {
    lcd.print("0");
  }
  lcd.print(UTCMinutes);
  lcd.print(":");
  if(rtc.second()<10)
  {
    lcd.print("0");
  }
  lcd.print(rtc.second());

  /******************************************************
  *********** To view Local Time on LCD *****************
  ******************************************************/

  lcd.setCursor(0,3);
  lcd.print("Local Time:");
  lcd.setCursor(12, 3);
  if(rtc.hour()<10)
  {
    lcd.print("0");
  }
  lcd.print(rtc.hour());
  lcd.setCursor(14,3);
  lcd.print(':');
  if(rtc.minute()<10)
  {
    lcd.print("0");
  }
  lcd.print(rtc.minute());
  lcd.setCursor(17,3);
  lcd.print(':');
  if(rtc.second()<10)
  {
    lcd.print("0");
  }
  lcd.print(rtc.second());
  lcd.setCursor(18,3);
  
  /******************************************************
  ****** To change on LCD data on every second **********
  ******************************************************/

  if( (rtc.hour() != prev_hour) | (rtc.minute() != prev_minute) | (rtc.second() != prev_second) )
  {
    lcd.clear();
    seconds++;
  }

  prev_hour=rtc.hour();
  prev_minute=rtc.minute();
  prev_second=rtc.second();
}

int LocaltoUTCTime(int hour, int minute)
{
  // Calculate the local time
  UTCHours = hour + timeZoneOffsetHours;
  UTCMinutes = minute + timeZoneOffsetMinutes;

  // Adjust for negative minutes or overflow
  if (UTCMinutes < 0) {
    UTCMinutes += 60;
    UTCHours -= 1;
  }
  if (UTCMinutes >= 60) {
    UTCMinutes -= 60;
    UTCHours += 1;
  }
}

void Calculate_Sun_Position(int hour, int minute, int second, int day, int month, int year,float longitude , float latitude) 
{
  double T, JD_frac, L0, M, e, C, L_true, f, R, GrHrAngle, Obl, RA, Decl, HrAngle;
  long JDate, JDx;
  
  ////////// Julian Date
  JDate      = JulianDate(year, month, day);

  ////////// Julian Time 
  JD_frac    = (hour + minute / 60.0 + second / 3600.0) / 24.0 - 0.5;

  ////////// Substract Time reference 1 January 2000
  T          = JDate - 2451545; 
  T          = (T + JD_frac) / 36525.0;

  ////////// Mean Logitude
  L0         = DEG_TO_RAD * fmod(280.46645 + 36000.76983 * T, 360);

  ////////// Mean Anomaly
  M          = DEG_TO_RAD * fmod(357.5291 + 35999.0503 * T, 360);
  e          = 0.016708617 - 0.000042037 * T;
  C          = DEG_TO_RAD * ((1.9146 - 0.004847 * T) * sin(M) + (0.019993 - 0.000101 * T) * sin(2 * M) + 0.00029 * sin(3 * M));
  f          = M + C;

  ///////// obliquity of the ecliptic
  Obl        = DEG_TO_RAD * (23 + 26 / 60.0 + 21.448 / 3600. - 46.815 / 3600 * T);
  JDx        = JDate - 2451545;
  GrHrAngle  = 280.46061837 + (360 * JDx) % 360 + 0.98564736629 * JDx + 360.98564736629 * JD_frac;
  GrHrAngle  = fmod(GrHrAngle, 360.0);

  ///////// ecliptic logitude
  L_true     = fmod(C + L0, 2 * PI);

  R          = 1.000001018 * (1 - e * e) / (1 + e * cos(f));
  ///////// Right ascension
  RA         = atan2(sin(L_true) * cos(Obl), cos(L_true));

  ///////// declination
  Decl       = asin(sin(Obl) * sin(L_true));

  ///////// Hour angle
  HrAngle    = (DEG_TO_RAD * GrHrAngle) + (longitude * DEG_TO_RAD) - RA;
  
  // Azimuth measured east from north, so 0 degrees is North

  ///////// Elevation Angle
  ele_ang  = asin(sin(latitude*DEG_TO_RAD) * sin(Decl) + cos(latitude*DEG_TO_RAD) * (cos(Decl) * cos(HrAngle)));
  ele_ang = (ele_ang / DEG_TO_RAD)+ elevation_offset;

  ///////// Azimuth angle
  azi_ang  = PI + atan2(sin(HrAngle), cos(HrAngle) * sin(latitude*DEG_TO_RAD) - tan(Decl) * cos(latitude*DEG_TO_RAD)); 
  azi_ang   = (azi_ang   / DEG_TO_RAD) + azimuth_offset;
  
}

long JulianDate(int year, int month, int day) {
  long JDate;
  int A, B;
  if (month <= 2) {year--; month += 12;}
  A = year / 100; B = 2 - A + A / 4;
  JDate = (long)(365.25 * (year + 4716)) + (int)(30.6001 * (month + 1)) + day + B - 1524; 
  return JDate;
}

void day_of_the_year(int day, int month, int year)
{
  int day_in_month[13]={0,31,28,31,30,31,30,31,31,30,31,30,31};
  dayofyear=day;
  if(year%4==0)
  {
    day_in_month[2]=29;
  }
  for(int i=0;i<month;i++)
  {
    dayofyear+=day_in_month[i];
  }
}

void desire_angle_motor_direction(float desire_x_axis_angle,float desire_y_axis_angle)
{
  float x_diff = current_x_axis_angle - desire_x_axis_angle;
  float y_diff = current_y_axis_angle - desire_y_axis_angle;
  int speed_x,speed_y;
  if(fabs(x_diff)>3)
  {
    if(x_diff > 0)
    {
      speed_x=-9;
    }
    else if(x_diff < 0)
    {
      speed_x=9;
    }
    Serial2.write(191 - speed_x);
  }
  else
  {
    speed_x=0;
    Serial2.write(191);
  }
  if(fabs(y_diff)>3)
  {
    if (y_diff > 0) {
      speed_y = 9;
    } else if (y_diff < 0) {
      speed_y = -9;
    }
    Serial2.write(64 - speed_y);
  }
  else 
  {
    speed_y=0;
    Serial2.write(64);
  } 
}

void Print_data(bool L_S,bool gpsorrtc)
{
    if(L_S == LCD_)
    {
       if(gpsorrtc == GPS_)
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