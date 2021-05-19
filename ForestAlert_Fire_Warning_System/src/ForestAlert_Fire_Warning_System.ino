/*
 * Project ForestAlert_Fire_Warning_System
 * Description: Early Warning Forest Fire Detection System
 * Author: Wesley Eccles <<www.wesleyeccles.com>>
 * Date: May 17, 2021
 * 🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥🔥
 */

#include <Seeed_HM330X.h>
#include <Adafruit_BME280.h>
#include <TinyGPS++/TinyGPS++.h>
#include <JsonParserGeneratorRK.h>
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "credential.h"


SYSTEM_MODE ( SEMI_AUTOMATIC );

//ADAFRUIT.IO
  TCPClient TheClient; 

  // Setup the MQTT client class by passing in the WiFi client and MQTT server and login details. 
  Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

  /****************************** Feeds ***************************************/ 
  // Setup Feeds to publish or subscribe 
  // Notice MQTT paths for AIO follow the form: <username>/feeds/<feedname> 
  // Adafruit_MQTT_Subscribe GPS = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/gps"); 
  Adafruit_MQTT_Subscribe IRSENSOR = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Ir_Sensor"); 
  Adafruit_MQTT_Subscribe DELAYFREQUENCY = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/Reading_Frequency"); 
  Adafruit_MQTT_Publish GPS = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/gps");
  Adafruit_MQTT_Publish SAT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/sat_num");
  Adafruit_MQTT_Publish TEMP = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fire_temp");
  Adafruit_MQTT_Publish HUMIDITY = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/fire_hum"); 
  Adafruit_MQTT_Publish FIREALERT = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/Fire_Status"); 
  int last, lastTime;
  bool buttonState,fireSent;


//SYSTEM
   const int LEDPIN= D7;
  unsigned int currentTime, lastTimeMain;
  unsigned int TIMEDELAY= 30000;    //Delay time for how often the system will send data to cloud (ms)

//FIRE IR SENSOR--------------------------------------------------------------------------------------
  const int IRPIN_A= A0;
  const int IRPIN_D= A1;
  bool FireAlert,IRSensor;

//AIR CURCULATION FAN--------------------------------------------------------------------------------------
  const int FANPIN = A3;

//LASER PARTICLE SENSOR------------------------------------------------------------------------------------

  #define SERIAL_OUTPUT Serial 
  HM330X sensor;
  uint8_t buf[30];
  const char* str[] = {"sensor num: ", "PM1.0 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                      "PM2.5 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                      "PM10 concentration(CF=1,Standard particulate matter,unit:ug/m3): ",
                      "PM1.0 concentration(Atmospheric environment,unit:ug/m3): ",
                      "PM2.5 concentration(Atmospheric environment,unit:ug/m3): ",
                      "PM10 concentration(Atmospheric environment,unit:ug/m3): ",
                      };


//BME280---------------------------------------------------------------------------------------------------
  #define SEALEVELPRESSURE_HPA (1013.25)
  Adafruit_BME280 bme; // I2C
  unsigned long delayTime;
  unsigned status;

// GPS --------------------------------------------------------------------------------------------------
TinyGPSPlus gps;
const int UTC_offset = -6; 
unsigned long lastSerial = 0;
unsigned long lastPublish = 0;
unsigned long startFix = 0;
bool gettingFix = false;
float lat,lon,alt;

SYSTEM_THREAD(ENABLED);
const unsigned long PUBLISH_PERIOD = 1000;
const unsigned long SERIAL_PERIOD = 5000;
const unsigned long MAX_GPS_AGE_MS = 1000; // GPS location must be newer than this to be considered valid

void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Serial start");

  Serial.printf("Connecting to Internet \n");
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
    delay(100);
  }
  Serial.printf("\nConnected!!!!!! \n");

  // // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&DELAYFREQUENCY);
  mqtt.subscribe(&IRSENSOR);
  // mqtt.subscribe(&SERVOPOS);
  // mqtt.subscribe(&COLOR);

//GPS 
    Serial1.begin(9600);
    startFix = millis();
    gettingFix = true;

//LASER PARTICLE SENSOR
  if (sensor.init()) {
      Serial.println("HM330X init failed!!!");
      while (1);
  }
//AIR FAN
  pinMode(FANPIN,OUTPUT);

//IR SENSOR
  pinMode(IRPIN_A,INPUT);
  pinMode(IRPIN_D,INPUT);
  attachInterrupt(IRPIN_D, FIREFIRE , RISING );

//BME280
  status = bme.begin();
  if (!status){
      Serial.println("Could not find a valid BME280 sensor, check wiring, address, sensor ID!");
      Serial.print("SensorID was: 0x");
      Serial.println(bme.sensorID(), 16);
      Serial.print("        ID of 0xFF probably means a bad address, a BMP 180 or BMP 085\n");
      Serial.print("   ID of 0x56-0x58 represents a BMP 280,\n");
      Serial.print("        ID of 0x60 represents a BME 280.\n");
      Serial.print("        ID of 0x61 represents a BME 680.\n");
      while (1);
  }
//LED Light 
  pinMode(LEDPIN,OUTPUT); 
  for (int i=0; i<5; i++){
    digitalWrite(LEDPIN,HIGH);
    delay(500);
    digitalWrite(LEDPIN,LOW);
    delay(250);
  }

}


void loop() {
  KeepConnection();       //Keep Connection to the cloud alive 
  MQTT_connect();         //Keep Connection to the cloud alive 
  Subscribe();

  if (FireAlert && IRSensor){
    if(mqtt.Update()) {
      FIREALERT.publish("w:fire");
      Serial.printf("FIRE FIRE FIRE -IR LIGHT DETECTED Sending Data \n");
      fireSent=1;
    }
    lastTimeMain=0;
    FireAlert=0;
  }

  if (millis()-lastTimeMain > TIMEDELAY){
    
    Serial.printf("STARTING Collecting Data\n");
  
    CirculateAir(5000);   //Turn on the fan to fill chamber with putside air 
    LaserPMRead();        //Collect the data from the laser PM Sensor
    GetGPS();             //Gather GPS Data and if valid send to Cloud
    SendEnviroData();     //Send temp + Humidity to Cloud
    lastTimeMain=millis();
    if (!fireSent){
      FIREALERT.publish("tree");
      Serial.printf("No Fire Detected \n");
    }
    else {
      fireSent=0;
    }
    Serial.printf("FINISHED Collecting Data\n\n");
  } 
}




//LASER PARTICLE SENSOR------------------------------------------------------------------
void LaserPMRead(){
  int PM2;
  int PM10;
  if (sensor.read_sensor_value(buf, 29)) {
     SERIAL_OUTPUT.println("HM330X read result failed!!!");
  }
  PM2 = (uint16_t) buf[6 * 2] << 8 | buf[6 * 2 + 1];
  PM10 = (uint16_t) buf[7 * 2] << 8 | buf[7 * 2 + 1];  
  Serial.printf("PM2 =%i  PM10 =%i \n",PM2,PM10);
  if (PM2 > 300 || PM10 > 950){
      FIREALERT.publish("w:fire");
      Serial.printf("FIRE FIRE FIRE -PM DETECTED Sending Data \n");
      fireSent=1;
  }

}

HM330XErrorCode print_result(const char* str, uint16_t value) {
    if (NULL == str) {
        return ERROR_PARAM;
    }
    SERIAL_OUTPUT.print(str);
    SERIAL_OUTPUT.println(value);
    return NO_ERROR1;
}

HM330XErrorCode parse_result(uint8_t* data) {
    uint16_t value = 0;
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 6; i < 8; i++) {
        value = (uint16_t) data[i * 2] << 8 | data[i * 2 + 1];
        print_result(str[i - 1], value);

    }

    return NO_ERROR1;
}

HM330XErrorCode parse_result_value(uint8_t* data) {
    if (NULL == data) {
        return ERROR_PARAM;
    }
    for (int i = 0; i < 28; i++) {
        SERIAL_OUTPUT.print(data[i], HEX);
        SERIAL_OUTPUT.print("  ");
        if ((0 == (i) % 5) || (0 == i)) {
          SERIAL_OUTPUT.println("");
        }
    }
    uint8_t sum = 0;
    for (int i = 0; i < 28; i++) {
      sum += data[i];
    }
    if (sum != data[28]) {
      SERIAL_OUTPUT.println("wrong checkSum!!!!");
    }
    SERIAL_OUTPUT.println("");
    return NO_ERROR1;
}

void CirculateAir (unsigned int _runTime){                  //Funtion needs to keep being called to check against time

  digitalWrite(FANPIN,HIGH);           //Start the fan when funtion is called
  delay(_runTime);                     //Time set to fill chamber with outside air 
  digitalWrite(FANPIN,LOW);           
  delay(1000);                         //Let the air in the chamber settle before taking data 

}

void IRFireSensor(){
    int IR_FIRE_A;
    bool IR_FIRE_D; 
    
    IR_FIRE_A= analogRead(IRPIN_A);
    IR_FIRE_D= digitalRead(IRPIN_D);

    Serial.printf("IR Fire Detector= %i  IR Fire= %i \n", IR_FIRE_A, IR_FIRE_D);
}

void GetGPS(){
    // This sketch displays information every time a new sentence is correctly encoded.
	while (Serial1.available() > 0) {
		if (gps.encode(Serial1.read())) {
      displayInfo();
		}
	}
}

void displayInfo() {
	float lat,lon,alt;
	uint8_t hr,mn,se,sat;
	if (millis() - lastSerial >= SERIAL_PERIOD) {
		lastSerial = millis();
     
		char buf[128];
		if (gps.location.isValid() && gps.location.age() < MAX_GPS_AGE_MS) {
			digitalWrite(LEDPIN,HIGH);
      lat = gps.location.lat();
			lon = gps.location.lng(); 
			alt = gps.altitude.meters();
			hr = gps.time.hour();
			mn = gps.time.minute();
			se = gps.time.second();
			sat = gps.satellites.value();
			if(hr > 7) {
				hr = hr + UTC_offset;
			}
			else {
				hr = hr + 24 + UTC_offset;
			} 
		
			if (gettingFix) {
				gettingFix = false;
				unsigned long elapsed = millis() - startFix;
				Serial.printlnf("%lu milliseconds to get GPS fix", elapsed);
			}
			createEventPayLoad(lat,lon,alt,sat);
      
		}
		else {
      digitalWrite(LEDPIN,LOW);
      Serial.println("No Location ");
			strcpy(buf, "no location");
			if (!gettingFix) {
				gettingFix = true; 
				startFix = millis();
			}
		}
	}
}

void createEventPayLoad ( float _lat , float _lon, float _alt, int _sat) {        //Create a JSON for Adafruit.io of GPS Data
  char GPSBuffer [50];
  sprintf(GPSBuffer,"{\"lat\":%f,\"lon\":%f,\"ele\":%f}",_lat,_lon,_alt);

  if(mqtt.Update()) {
    SAT.publish(_sat);
    Serial.printf("Satelites: %i \n", _sat );
    GPS.publish(GPSBuffer);
    Serial.printf("Publishing JSON GPS: %s \n", GPSBuffer );
  }
} 

void MQTT_connect(){
  int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.println(mqtt.connectErrorString(ret));
       Serial.println("Retrying MQTT connection in 5 seconds...");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.println("MQTT Connected!");
}

void KeepConnection(){
  // Validate connected to MQTT Broker
  MQTT_connect();
  // Ping MQTT Broker every 2 minutes to keep connection alive
  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      if(! mqtt.ping()) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
 }

float tempF(){
  float _tempF;
  float _tempC;
  _tempC = bme.readTemperature();
  _tempF = (_tempC*1.8)+32;
  return _tempF;  
}
 
int Humidity(){
  int _humidity;
  _humidity= bme.readHumidity();
  return _humidity;
}

void SendEnviroData(){
   if(mqtt.Update()) {
    TEMP.publish(tempF());
    Serial.printf("Publishing TempF %.1ff\n",tempF()); 
    HUMIDITY.publish(Humidity());
    Serial.printf("Publishing Humidity %i%%\n",Humidity());  
    if (tempF() > 125 || Humidity() < 12){
      FIREALERT.publish("w:fire");
      Serial.printf("FIRE FIRE FIRE -TEMP HUMIDITY DETECTED Sending Data \n");
      fireSent=1;
    }
  }
}

void Subscribe(){
 Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(500))) {
    if (subscription == &DELAYFREQUENCY) {
      TIMEDELAY = (atoi((char *)DELAYFREQUENCY.lastread))*1000;
    }
    if (subscription == &IRSENSOR) {
      IRSensor = atoi((char *)IRSENSOR.lastread);
    }
  } 
}

void FIREFIRE(){                //Interupt Pin if the IR Light sees fire to send one last signal that there is a fire before burning 
 FireAlert=1;
}