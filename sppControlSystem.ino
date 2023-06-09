/*
 This file is part of the AberSailbot minimum viable control system (AMVCS).
 AMVCS is free software: you can
 redistribute it and/or modify it under the terms of the GNU General Public
 License as published by the Free Software Foundation, version 2.

 This program is distributed in the hope that it will be useful, but WITHOUT
 ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
 FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
 details.

 You should have received a copy of the GNU General Public License along with
 this program; if not, write to the Free Software Foundation, Inc., 51
 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.

 Copyright Colin Sauze

libraries required:
timelib
esp32servo
tinygps
*/



#include <stdio.h>
//#include <Servo.h>
#include <ESP32Servo.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <math.h>

#include <WiFi.h>
#include <WiFiAP.h>
#include <AsyncUDP.h>


//Time.h became TimeLib.h as of version 1.6.1
#include "TimeLib.h"
#include "TinyGPS.h"

#define HMC6343_ADDRESS 0x19
#define HMC6343_HEADING_REG 0x50
#define GPS_ENABLE_PIN 12
#define CMPS12_ADDRESS 0x60

#define GPS_READ_INTERVAL 15 //how many seconds to leave between GPS reads
#define WP_THRESHOLD 15 //how close (in metres) should we get before we change waypoint?

#define rad2deg(x) (180/M_PI) * x
#define deg2rad(x) x * M_PI/180

Servo rudderServo; // create servo object to control a servo

//SoftwareSerial myDebug(7, 8);
HardwareSerial myDebug(1);

TinyGPS gps;

#define HEADING 0
#define WIND_DIR 1
#define ROLL 2
#define PITCH 3
#define RUDDER 4
#define SAIL 5
#define LAT 6
#define LON 7
#define TIME 8

#define DEBUG_CRITICAL 1 //really important messages that we don't want to ignore and are prepared to sacrifice execution speed to see
#define DEBUG_IMPORTANT 2 //fairly important messages that we probably want to see, but might cause issues with execution speed
#define DEBUG_MINOR 3 //less important messages that we can safely turn off to improve execution speed
#define DEBUG_VERBOSE 4 //Dan and Rosias debug messages

#define DEBUG_THRESHOLD DEBUG_VERBOSE //set to 0 to show no debugging messages

const char *ssid = "boat";
AsyncUDP udp;
byte ledState=0;

  struct Data{
    uint16_t heading;
    uint16_t wind_dir;
    int8_t roll;
    int8_t pitch;
    int8_t rudder;
    byte sail;
    float lat;
    float lon;
    long unixtime;
  } 
  state;


//make printf work
static FILE uartout = {
  0} 
;

static int uart_putchar (char c, FILE *stream)
{
  myDebug.write(c) ;
  return 0 ;
}

static void say(byte level, char* msg)
{
  if(level<=DEBUG_THRESHOLD)
  {
    myDebug.print("Debug");
    myDebug.print(level);
    myDebug.print(": [Ctrl] ");
    myDebug.println(msg);
  }
}

//debugging printf that prepends "Debug:" to everything and can be easily turned off
void dprintf(byte level, const char *fmt, ...)
{      
  char outbuf[1024];

  if(level<=DEBUG_THRESHOLD)
  {
    printf("Debug%d: [Ctrl] ",level);
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(outbuf,1024,fmt, ap);
    myDebug.print(outbuf);
    va_end(ap);
  }
}


void setup() 
{
  //Serial.begin(9600); //baud rate makes no difference on 32u4

  Serial.begin(4800, SERIAL_8N1, 16, 17); //for GPS // 17 tx, 16 rx

  myDebug.begin(4800, SERIAL_8N1, 3, 1); //debug UART on GPIO 6 and 7
  say(DEBUG_MINOR, "Debug serial system set up");

  say(DEBUG_VERBOSE,"Control system start up");

  //required for printf  
  //but not on ESP32
  //fdev_setup_stream (&uartout, uart_putchar, NULL, _FDEV_SETUP_WRITE);
  //stdout = &uartout ;
  dprintf(DEBUG_IMPORTANT,"Printf configured \r\n");

  delay(5000);

  say(DEBUG_IMPORTANT,"Setting up servos...");
  //Use .attach for setting up connection to the servo
  rudderServo.attach(5, 1060, 1920); // Attach, with the output limited
  // between 1000 and 2000 ms
  rudderServo.writeMicroseconds(1500);
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setting up I2C...");
  Wire.begin(); // Initialise i2c for compass
  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setting up GPS...");
  pinMode(GPS_ENABLE_PIN, OUTPUT); //GPS on/off line
  //setup GPS
  digitalWrite(GPS_ENABLE_PIN,1);
  delay(1000);
  //turn off VTG
  Serial.println("$PSRF103,05,00,00,01*21\r");
  //turn off RMC
  Serial.println("$PSRF103,04,00,00,01*20\r");

  //turn off GSV
  Serial.println("$PSRF103,03,00,00,01*27\r");

  //turn off GSA
  Serial.println("$PSRF103,02,00,00,01*26\r");

  //turn off GLL
  Serial.println("$PSRF103,01,00,00,01*25\r");

  //turn off GGA
  Serial.println("$PSRF103,00,00,00,01*24\r");
  delay(1000);

  //leave GPS on to get its initial fix
  //digitalWrite(GPS_ENABLE_PIN,0);

  //setup WiFi
  say(DEBUG_IMPORTANT,"setting up WiFi");
  WiFi.softAP(ssid);
  IPAddress myIP = WiFi.softAPIP();

  say(DEBUG_IMPORTANT,"Done");

  say(DEBUG_IMPORTANT,"Setup Complete\n");
}

//computes an NMEA checksum
byte compute_checksum(char *data,byte length)
{                
  byte computed_checksum=0;

  for (byte i = 0; i < length; i++)
  {
    computed_checksum = (byte)computed_checksum ^ (byte)data[i];
  }

  return computed_checksum;
}

//reads heading from HMC6343 compass
int readCompass() {
  unsigned char high_byte, low_byte, angle8;
  char pitch, roll;
  unsigned int heading;
  
  Wire.beginTransmission(CMPS12_ADDRESS);  //starts communication with CMPS12
  Wire.write(1);                     //Sends the register we wish to start reading from
  Wire.endTransmission();
 
  // Request 5 bytes from the CMPS12
  // this will give us the 8 bit bearing, 
  // both bytes of the 16 bit bearing, pitch and roll
  Wire.requestFrom(CMPS12_ADDRESS, 5);       
  
  while(Wire.available() < 5);        // Wait for all bytes to come back
  
  angle8 = Wire.read();               // Read back the 5 bytes
  high_byte = Wire.read();
  low_byte = Wire.read();
  pitch = Wire.read();
  roll = Wire.read();

  // heading is angle16
  heading = high_byte;                 // Calculate 16 bit angle
  heading <<= 8;
  heading += low_byte;

//  may need to reimplement this later  
   heading=heading/10;
   roll=roll/10;
   pitch=pitch/10;
   
   myDebug.print("Heading = ");
   myDebug.print(heading);
   
// rewrite this to set state to our values
  state.roll=(int8_t)roll;
  state.pitch=(int8_t)pitch;
  state.heading=(uint16_t)heading;
  
  dprintf(DEBUG_IMPORTANT,"Heading: %d Roll: %d Pitch: %d\r\n",state.heading,state.roll,state.pitch);
  printf("heading=%d\r\n",heading);

  delay(100);
  
  return (int)heading; // Print the sensor readings to the serial port.
}

void readGPS() {
  unsigned long fix_age=9999,time,date;

  say(DEBUG_VERBOSE,"About to read GPS");
  digitalWrite(GPS_ENABLE_PIN,1); //turn the GPS on
  delay(1000);

  while(fix_age == TinyGPS::GPS_INVALID_AGE||fix_age>3000) //make sure the GPS has a fix, this might cause a wait the first time, but it should be quick any subsequent time
  {
    Serial.println("$PSRF103,04,01,00,01*21\r");
    dprintf(DEBUG_MINOR,"NMEA string: ");
    unsigned long start = millis();
    while(millis()<start+2000)
    {
      if(Serial.available())
      {
        int c = Serial.read();
        gps.encode(c);
        if(DEBUG_THRESHOLD>=DEBUG_MINOR)
        {
          myDebug.write(c);
        }
        if(c=='\n')
        {
          break;
        }
      }
    }
    gps.get_datetime(&date,&time,&fix_age);

    dprintf(DEBUG_MINOR,"fix age = %ld\r\n",fix_age);
    if(fix_age == TinyGPS::GPS_INVALID_AGE)
    {
      dprintf(DEBUG_IMPORTANT,"Invalid fix, fix_age=%ld\r\n",fix_age);
      say(DEBUG_IMPORTANT,"No GPS fix");
    }
  }

  digitalWrite(GPS_ENABLE_PIN,0); //turn the GPS off
   

  gps.get_datetime(&date,&time,&fix_age);
  gps.f_get_position(&state.lat,&state.lon,&fix_age);
   

  if(fix_age == TinyGPS::GPS_INVALID_AGE)
  {
    say(DEBUG_IMPORTANT,"Invalid fix");
  }

  else
  {
    
    say(DEBUG_IMPORTANT,"Fix Valid");
    dprintf(DEBUG_IMPORTANT,"lat=%ld lon=%ld\r\n",(long)(state.lat*1000),(long)(state.lon*1000));

    int year;
    byte month,day,hour,min,sec;
    unsigned long age;
      
    gps.crack_datetime(&year,&month,&day,&hour,&min,&sec,NULL,&age);
    
    setTime(hour,min,sec,day,month,year); //sets the time in the time library, lets us get unix time

  }

}

int mod(int value){ //keeps angles betweeen 0 and 360
  int newValue;
  if(value < 0){
    newValue = value + 360;
  }
  else if(value >= 360){
    newValue = value - 360;
  }
  else{
    newValue = value;
  }
  return newValue;
}

//calculates difference between two headings taking wrap around into account
int get_hdg_diff(int heading1,int heading2)
{
  int result;

  result = heading1-heading2;

  if(result<-180)
  {
    result = 360 + result;
    return result;
  } 

  if(result>180)
  {
    result = 0 - (360-result);
  }

  return result;
}

void loop()
{
  say(DEBUG_VERBOSE, "Entered loop");
  unsigned long last_gps_read=0;
  unsigned long last_time=0,time_now=0;
  int wp_hdg=0;
  float wp_dist=0.0;
  int wp_num=0;

  float igain=0.01;
  float pgain=0.1;
  float running_err=0.0;
  int hdg_err=0;
  int relwind;
  
  long last_telemetry=0;
  #define TELEMETRY_INTERVAL 10
  #define TARGET_LOOP_INTERVAL 100 //number of milliseconds between loop intervals

  #define NUM_OF_WAYPOINTS 1


  float wp_lats[NUM_OF_WAYPOINTS]; 
  float wp_lons[NUM_OF_WAYPOINTS];

  wp_lats[0]=52.4;
  wp_lons[0]=-4.4;


  while(1)
  {
    
    //make loop execute at constant speed
    time_now=millis();

    if(time_now-last_time>0&&time_now-last_time<TARGET_LOOP_INTERVAL)
    {
      delay(TARGET_LOOP_INTERVAL-(time_now-last_time));
    }

    last_time=millis();

    say(DEBUG_VERBOSE, "reading compass");
    readCompass();
    say(DEBUG_VERBOSE, "finished reading compass");
    //state.wind_dir=getTrueWind();        {

    //no wind sensor, so just use a fixed wind direction
    state.wind_dir=270;
    
    relwind=mod(state.wind_dir - state.heading);
    
    say(DEBUG_VERBOSE, "may read gps");
    if(millis()-last_gps_read>(GPS_READ_INTERVAL*1000)||millis()<last_gps_read) //read the GPS at the specified interval or whenever the millis count wraps around 
    {
      say(DEBUG_VERBOSE,"Reading GPS");
      readGPS();
      wp_hdg = (int) TinyGPS::course_to(state.lat, state.lon, wp_lats[wp_num],wp_lons[wp_num]);
      wp_dist = TinyGPS::distance_between(state.lat, state.lon, wp_lats[wp_num],wp_lons[wp_num]);
      if(wp_dist<WP_THRESHOLD)
      {       
        wp_num++;      
        if(wp_num==NUM_OF_WAYPOINTS) //reached last waypoint already
        {
          wp_num--;          
        }
        else //reached new waypoint
        {
          wp_hdg = (int) TinyGPS::course_to(state.lat, state.lon,wp_lats[wp_num],wp_lons[wp_num]);
          wp_dist = TinyGPS::distance_between(state.lat, state.lon, wp_lats[wp_num],wp_lons[wp_num]);
        }
      }
      last_gps_read=millis();
    }

    say(DEBUG_VERBOSE, "Finished possible gps read");

    //sail logic
    //sailLogic(relwind);

    //rudder logic
    hdg_err = get_hdg_diff(wp_hdg,state.heading);

    running_err = running_err + (float)hdg_err;
    if (abs(running_err > 4000))
    {
      running_err = 4000; // limit integral component
    }
    running_err = running_err * 0.9;

    /*dprintf("hdg_err = %d running_err = ",hdg_err);
    myDebug.println(running_err);*/

    state.rudder = (int) round((pgain * (float)hdg_err) + (igain * running_err));
    if(state.rudder<-5)
    {
      state.rudder=-5;
    }
    else if(state.rudder>5)
    {
      state.rudder=5;
    }

    say(DEBUG_VERBOSE, "correcting rudder");
    rudderServo.writeMicroseconds(1500+(state.rudder*100));

    say(DEBUG_VERBOSE, "possible telemetry incoming:");
    if(last_telemetry+(TELEMETRY_INTERVAL*1000)<millis())
    {
      char msgbuf[255];
      char nmeadata[80];
      char checksum;

      //generate key value telemetry packet
      snprintf(msgbuf,254,"time=%ld lat=%.4f lon=%.4f hdg=%d hdg_err=%d roll=%d pitch=%d rudder=%d wp_num=%d wp_hdg=%d wp_dist=%ld ",now(),state.lat,state.lon,state.heading,hdg_err,state.roll,state.pitch,state.rudder,wp_num,wp_hdg,(long)wp_dist);
      say(DEBUG_CRITICAL,msgbuf);
      udp.broadcastTo(msgbuf,1234);

      //generate compass NMEA string
      snprintf(nmeadata,79,"HDM,%d.0,M",state.heading);
      checksum=compute_checksum(nmeadata,strlen(nmeadata));
      snprintf(msgbuf,254,"$%s*%X\n",nmeadata,checksum);
      say(DEBUG_CRITICAL, msgbuf);
      udp.broadcastTo(msgbuf,10000);

      //generate GPS NMEA string
      char timestr[7];
      char hemNS;
      char hemEW;
      char latstr[10]; //DDMM.MMMM0
      char lonstr[11]; //DDDMM.MMMM0
      int degrees;
      float minutes;
      //generate time
      snprintf(timestr,6,"%2d%2d%2d",hour(),minute(),second());

      //generate lat/lon
      degrees = abs((int) state.lat);
      minutes = (fabs(state.lat) - (float)degrees) * 60.0;
      snprintf(latstr,9,"%2d%2.4f",degrees,minutes);
      //get the hemisphere north/south
      if (state.lat>0) {
        hemNS = 'N';
      }
      else {
        hemNS = 'S';
      } 

      degrees = abs((int) state.lon);
      minutes = (fabs(state.lon) - (float)degrees) * 60.0;
      snprintf(lonstr,10,"%3d%2.4f",degrees,minutes);
      //get the hemisphere east/east
      if (state.lon>0) {
        hemEW = 'E';
      }
      else {
        hemEW = 'W';
      } 

      //$GPGLL,LATMM.MMMM,N/S,LONMM.MMMM,E/W,HHMMSS,A,*SUM
      snprintf(nmeadata,79,"GPGLL,%s,%c,%s,%c,%s,A%",latstr,hemNS,lonstr,hemEW,timestr);
      checksum=compute_checksum(nmeadata,strlen(nmeadata));
      snprintf(msgbuf,254,"$%s*%X\n",nmeadata,checksum);
      
      say(DEBUG_CRITICAL, msgbuf);
      //send to UDP port 10000
      udp.broadcastTo(msgbuf,10000);
    

      if(DEBUG_THRESHOLD>=DEBUG_CRITICAL)
      {
        myDebug.print("lat=");
        myDebug.print(state.lat,5);
        myDebug.print(" lon=");
        myDebug.print(state.lon,5);
        myDebug.print(" wplat=");
        myDebug.print(wp_lats[wp_num],5);
        myDebug.print(" wplon=");
        myDebug.print(wp_lons[wp_num],5);
        myDebug.print(" running_err=");
        myDebug.println(running_err);
      }

      last_telemetry=millis();
    }
  }
}
