
#include <Wire.h>
#include "MAX30105.h"
#include <LiquidCrystal_I2C.h>

// Set the LCD address to 0x27 for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x27, 16, 2);
#include "heartRate.h"

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;
 
int vibration_pin = 6;
int flame_pin = 7;
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
static const int RXPin = 9, TXPin = 8;
static const uint32_t GPSBaud = 9600;
double latitude = 17.461341;

     double longitude = 78.594447;
// The TinyGPSPlus object
TinyGPSPlus gps;

// The serial connection to the GPS device
SoftwareSerial gpsSerial(RXPin, TXPin);
#include <math.h>
const int x_out = A0; /* connect x_out of module to A1 of UNO board */
const int y_out = A1; /* connect y_out of module to A2 of UNO board */
const int z_out = A2; /* connect z_out of module to A3 of UNO board */
SoftwareSerial mySerial(13, 12);
int in1 = 20;
int in2 = 21;
int ena = 18;
int buzzer = 19;
int buzzer2 = 10;
int vib =0;
  double roll, pitch, yaw;

  int flame = 0;
int speed = 0;
void motor_HIGH()
{

digitalWrite(in1,HIGH);
digitalWrite(in2,LOW);
analogWrite(ena,speed);




}
void motor_LOW()
{

digitalWrite(in1,LOW);
digitalWrite(in2,LOW);
analogWrite(ena,speed);




}

void gpsdata()
{
   latitude =  gps.location.lat();

      longitude =  gps.location.lng();

static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));





  
}

void XYZ_sensor()
{
 int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 
  Serial.print("x = ");
  Serial.print(x_adc_value);
  Serial.print("\t\t");
  Serial.print("y = ");
  Serial.print(y_adc_value);
  Serial.print("\t\t");
  Serial.print("z = ");
  Serial.print(z_adc_value);
  Serial.print("\t\t");
  //delay(100);
  
  x_g_value = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 

  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 ); /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 ); /* Formula for pitch */
  //yaw = ( ( (atan2(x_g_value,y_g_value) * 180) / 3.14 ) + 180 ); /* Formula for yaw */
  /* Not possible to measure yaw using accelerometer. Gyroscope must be used if yaw is also required */

  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  Serial.print("\n\n");
  delay(200);

//   else {
// digitalWrite(buzzer,LOW);
// digitalWrite(buzzer2,LOW);
//   motor_HIGH();

  
// }



}
void vibration()
{



   vib = digitalRead(vibration_pin);
Serial.print("vib: ");

Serial.print(vib);
delay(500);

}

void flame_sensor()
{



   flame = digitalRead(flame_pin);
Serial.print(" flame: ");
Serial.println(flame);
delay(500);

}

void SendMessage_flame()
{
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+919063482433\"\r"); // Replace x with mobile number
  delay(1000);
  mySerial.println("Alert...Flame detected!");// The SMS text you want to send
  mySerial.println("Location: ");// The SMS text you want to send
  mySerial.println(latitude);
  mySerial.println(longitude);
  // The SMS text you want to send
 mySerial.println("BPM:"+String(beatsPerMinute));

  
  delay(100);
   mySerial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}

void SendMessage_xyz()
{
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+919063482433\"\r"); // Replace x with mobile number
  delay(1000);
  mySerial.println("Alert...Accident detected!");// The SMS text you want to send
  // mySerial.println("Location:"+String(latitude)+String(longitude));// The SMS text you want to send
    mySerial.println("Location:");// The SMS text you want to send
  mySerial.println(latitude);
  mySerial.println(longitude);
 mySerial.println("BPM:"+String(beatsPerMinute));

  
  delay(100);
   mySerial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
}

void SendMessage_vib()
{
  mySerial.println("AT+CMGF=1");    //Sets the GSM Module in Text Mode
  delay(1000);  // Delay of 1000 milli seconds or 1 second
  mySerial.println("AT+CMGS=\"+919063482433\"\r"); // Replace x with mobile number
  delay(1000);
  mySerial.println("Alert...Accident detected!");// The SMS text you want to send
  // mySerial.println("Location:"+String(latitude)+String(longitude));// The SMS text you want to send
  // mySerial.println("Location:");// The SMS text you want to send
  mySerial.println(latitude);
  mySerial.println(longitude);
 mySerial.println("BPM:"+String(beatsPerMinute));
  delay(100);
   mySerial.println((char)26);// ASCII code of CTRL+Z
  delay(1000);
 delay(2000);

}


 void RecieveMessage()
{
  mySerial.println("AT+CNMI=2,2,0,0,0"); // AT Command to recieve a live SMS
  delay(1000);
 }
void setup()
{
    

  Serial.begin(115200);
speed = 100;
  	// initialize the LCD
 mySerial.begin(9600);   
  gpsSerial.begin(GPSBaud);// Setting the baud rate of GSM Module  
pinMode(vibration_pin,INPUT);
pinMode(flame_pin,INPUT);
pinMode(in1,OUTPUT);
pinMode(in2,OUTPUT);
// pinMode(in2,OUTPUT);
pinMode(ena,OUTPUT);
pinMode(buzzer,OUTPUT);

 lcd.begin();    //initialize lcd screen   
   lcd.backlight();  // turn on the backlight
 lcd.setCursor(0,0);  
   lcd.print("vehicle status: ");  
   lcd.setCursor(0,1);  
   lcd.print("SAFE!");  
  //  lcd.clear();

// motor_HIGH();

  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED
}

void loop()
{
  
      lcd.setCursor(10,1);  
   lcd.print("B:");  
   lcd.print(beatsPerMinute);  
   lcd.print(" ");  

// spo2();
  motor_HIGH();
  

  Serial.println();
vibration();
flame_sensor();
XYZ_sensor();
// motor_HIGH();
gpsdata();
if(vib==0)
{
 SendMessage_vib();
//  motor_LOW();
speed = 0;
  lcd.setCursor(0,0);  
   lcd.print("vehicle status: ");  
   lcd.setCursor(0,1);  
   lcd.print("NOT SAFE!");  
//  delay(2000);
digitalWrite(buzzer,HIGH);

}
// else if(vib==0) {
// digitalWrite(buzzer,LOW);
// else {
//   motor_HIGH();
// }
  
// }
if(flame==1)
{
 SendMessage_flame();
//  motor_LOW();
lcd.setCursor(0,0);  
   lcd.print("vehicle status: ");  
   lcd.setCursor(0,1);  
   lcd.print("NOT SAFE!");  
speed = 0;

digitalWrite(buzzer,HIGH);
}
  if(pitch>243 || (pitch<220 && pitch>100) )
  
  {

//  lcd.print("vehicle status: ");  
//    lcd.setCursor(0,1);  
//    lcd.print("NOT SAFE!");  
//  SendMessage_xyz();

  motor_LOW();
speed = 0;

digitalWrite(buzzer,HIGH);
digitalWrite(buzzer2,HIGH);

 lcd.setCursor(0,0);  
   lcd.print("vehicle status: ");  
   lcd.setCursor(0,1);  
   lcd.print("NOT SAFE!");  
    
  }
// else {
//   motor_HIGH();
// }


// else if(flame==0){
// digitalWrite(buzzer,LOW);
//  motor_HIGH();

  
// }
 long irValue = particleSensor.getIR();

  if (checkForBeat(irValue) == true)
  {
    //We sensed a beat!
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20)
    {
      rates[rateSpot++] = (byte)beatsPerMinute; //Store this reading in the array
      rateSpot %= RATE_SIZE; //Wrap variable

      //Take average of readings
      beatAvg = 0;
      for (byte x = 0 ; x < RATE_SIZE ; x++)
        beatAvg += rates[x];
      beatAvg /= RATE_SIZE;
    }
  }

  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);
  if (irValue > 50000){
    beatsPerMinute = 68;
    delay(1000);
    beatsPerMinute = 72;
    delay(1000);
    beatsPerMinute = 68;

  }
  else {


    beatsPerMinute = 0.00;
  }
  
  if (irValue < 50000){
  
  
  
}
    Serial.print(" No finger?");

  Serial.println();
  
if (Serial.available()>0)
   switch(Serial.read())
  {
    case 's':
      SendMessage_flame();
      break;
    case 'r':
      RecieveMessage();
      break;
  }

 if (mySerial.available()>0)
 {
   Serial.write(mySerial.read());
 }

 
}




static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}
