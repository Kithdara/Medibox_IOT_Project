#include <PubSubClient.h>
#include <WiFi.h>
#include <DHTesp.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include <ESP32Servo.h>

// Pin Definitions
const int DHT_PIN = 15; // DHT temperature and humidity sensor pin
const int LDR_PIN = 35; // Light-dependent resistor (LDR) pin
const int BUZ_PIN = 27; // Buzzer pin
const int SRV_PIN =21; // Servo motor pin

WiFiClient espClient;
PubSubClient mqttClient(espClient);
DHTesp dhtSensor;
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org");
Servo servoMotor; 
DynamicJsonDocument doc(512);
hw_timer_t *Timer0_Cfg = NULL;

// Objects and Variables
char tempAr[6];
char humidityAr[6];
char LDRAr[5];
char DaysAr[2];
int currentTime;
bool MainSwitch = true;

bool switch1,switch2,switch3;
int Alarm1,Alarm2,Alarm3;
bool updateDate1 = false, updateDate2 = false, updateDate3 = false;
bool oneTimePass1 = true,oneTimePass2 = true,oneTimePass3 = true;
bool Alarm1Playing = false,Alarm2Playing = false,Alarm3Playing = false;
int  Delay=1;
int Frequency =512;
int Days = 0;
bool Repeat= true;
bool Repeating =false;

float controllingFactor = 0.75; //Initial values 
int offset = 30;
float intensity;
int angle;


void IRAM_ATTR Timer0_ISR();


void setup() {
  Serial.begin(115200);
  pinMode(LDR_PIN, INPUT);
  pinMode(BUZ_PIN, OUTPUT);
  
  setupWifi();    // Set up Wi-Fi connection
  setupMqtt();   // Set up MQTT client
  setupTime();   // Set up NTP time client
  setupDHT();    // Set up DHT temperature and humidity sensor
  setupServo();  // Set up servo motor

  //initilize Timer interrupts
  Timer0_Cfg = timerBegin(0, 80, true);               
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);

  tone(BUZ_PIN,512,500); // Power on tone


}



void loop() {
 
 
  delay(1000);
  
  //MQTT connection
  if (!mqttClient.connected()){
    connectToBroker();
  }
  mqttClient.loop();

  //Update & publish senser Data
  updateTempAndHumidity();
  updateIntensity();
  updateSlidingWindow();

  mqttClient.publish("MediBoxTesttemp",tempAr);
  mqttClient.publish("MediBoxTestHumidity",humidityAr);
  mqttClient.publish("MediBoxTestLDR",LDRAr);

  //Run Scheduler
  if (Days){
  updateAlarm();
  }
}

void setupWifi(){
  //This function is used to connect to Woki-Guest WIFI newwork
  WiFi.begin("Wokwi-GUEST","");

  while (WiFi.status() != WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  
  
}

void setupMqtt(){
  // function to connect setuup MQTT 
  mqttClient.setServer("test.mosquitto.org",1883);
  mqttClient.setCallback(receveCallback);
  
}

void connectToBroker(){
  // function to connect MQTT broker
  while(!mqttClient.connected()){
  Serial.println("Attempting MQTT connection...");
  if(mqttClient.connect("ESP32-465445676348")){
    Serial.println("connected");
    mqttClient.subscribe("MediBoxSetAlarm");
    mqttClient.subscribe("MediBoxSettingsAlarm");
    mqttClient.subscribe("MediBoxShadeController");
    mqttClient.subscribe("MediBoxMainSwitch");
  }else{
    Serial.print("failed ");
    Serial.print(mqttClient.state());
    delay(5000);
  }  
  }
}

void updateTempAndHumidity(){
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  String(data.temperature,2).toCharArray(tempAr,6);
  String(data.humidity,2).toCharArray(humidityAr,6);
}

void receveCallback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrive[");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length];
  for (int i = 0; i<length; i++){
    Serial.print((char)payload[i]);
    payloadCharAr[i] =(char)payload[i];
  }
  Serial.println();

  if (strcmp(topic,"MediBoxSetAlarm")==0){
    //MediBoxSetAlarm publish data related to alarm schedule as JSON object
    deserializeJson(doc, payload);
    switch1 = doc["Switch1"];
    switch2 = doc["Switch2"];
    switch3 = doc["Switch3"];

    Alarm1 = doc["Alarm1"];
    Alarm2 = doc["Alarm2"];
    Alarm3 = doc["Alarm3"];
    
    Days = doc["Days"];
    
    //Disable Alarm1
    if (Alarm1Playing && !switch1){
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, false);
      timerAlarmDisable(Timer0_Cfg);
      noTone(BUZ_PIN);
      Alarm1Playing = false;
      oneTimePass1 = true;
    }

    //Disable Alarm2
    if (Alarm2Playing && !switch2){
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, false);
      timerAlarmDisable(Timer0_Cfg);
      noTone(BUZ_PIN);
      Alarm2Playing = false;
      oneTimePass2 = true;
    }

    //Disable Alarm3
    if (Alarm3Playing && !switch3){
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, false);
      timerAlarmDisable(Timer0_Cfg);
      noTone(BUZ_PIN);
      Alarm3Playing = false;
      oneTimePass3 = true;
    }
    
   
  }

  if (strcmp(topic,"MediBoxSettingsAlarm")==0){
    /*MediBoxSettingsAlarm publish data related to alarm settings
    There are two Alarm modes (Repeat on-off mode &  continuous mode)
    In repeat mode Alarm repet with given Delay
    */

    deserializeJson(doc, payload);
    Delay = doc["Delay"];
    Frequency = doc["Frequency"];
    Repeat = doc["Repeat"];
  }

  if (strcmp(topic,"MediBoxShadeController")==0){
    //MediBoxShadeController publish data as JSON object
    deserializeJson(doc, payload);
    offset = doc["minAngle"];
    controllingFactor = doc["ctrFactor"];
    
  }

  if (strcmp(topic,"MediBoxMainSwitch")==0){
    //Main switch is used to silence alarm
    deserializeJson(doc, payload);
    MainSwitch = doc["MainSwitch"];

    if (!MainSwitch){ 
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, false);
      timerAlarmDisable(Timer0_Cfg);
      noTone(BUZ_PIN);
      Alarm1Playing = false;
      Alarm2Playing = false;
      Alarm3Playing = false;

    }
   
    
    
  }
}

void updateIntensity(){
  //When the light intensity increases, the LDR sensor reading decreases.
  // LDR sensor readings are always between 0 - 4096 
  intensity = (4096- analogRead(LDR_PIN))/4096.0;
  String(intensity,2).toCharArray(LDRAr,5);
}


void setupServo(){
  servoMotor.attach(SRV_PIN);
}

void setupDHT(){
  dhtSensor.setup(DHT_PIN,DHTesp::DHT22);
}


void updateSlidingWindow(){
  angle = offset + (180 - offset)*intensity*controllingFactor;
  servoMotor.write(angle);
}


void setupTime(){
  timeClient.begin();
  timeClient.setTimeOffset(19800);
}

void updateTime(){
  timeClient.update();
  currentTime = ( timeClient.getHours()*3600 + timeClient.getMinutes()*60)*1000;
}

void updateAlarm(){
  /*
  This function is used to schedule Alarms.
  Main switch is used to silent Alarms
  */

  updateTime(); // Get current Time
  updateDays(); //Upday day count

  //Set Alarm1
  if (switch1 && currentTime == Alarm1 && MainSwitch){
    if (oneTimePass1){
      oneTimePass1 = false;
      Alarm1Playing = true;
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, true); //Call Alarm Inturrpt
      timerAlarmEnable(Timer0_Cfg);
    }
  }
  //Set Alarm2
 if (switch2 && currentTime == Alarm2 && MainSwitch){
    
    if (oneTimePass2){
     
      oneTimePass2 = false;
      Alarm2Playing = true;
      timerAlarmWrite(Timer0_Cfg, Delay*1000000, true);
      timerAlarmEnable(Timer0_Cfg);
    }
  }
    //Set Alarm3
   if (switch3 && currentTime == Alarm3 && MainSwitch){
    if (oneTimePass3){
      oneTimePass3 = false;
      Alarm3Playing = true;
      timerAlarmWrite(Timer0_Cfg,  Delay*1000000, true);
      timerAlarmEnable(Timer0_Cfg);
    }
  }

}


void IRAM_ATTR Timer0_ISR()
//Interrupt Fuction for alarm
{
    
    if (Repeat){

      if (Repeating){
        noTone(BUZ_PIN);
        Repeating = false;
      }else{
        tone(BUZ_PIN, Frequency);
        Repeating = true;
      }

    }else{
      tone(BUZ_PIN, Frequency);
    } 

}

void updateDays(){
  //Day count does not update at Midnight 

  //One minute after playing all scheduled alarms for the day, the number of days is reduced by one. 
  
  //This algorithm prevents any medication schedule from being missed, depending on when the user configures the system.
  if (currentTime == Alarm1){
      updateDate1 = true;
  }else{
      oneTimePass1 = true;
   }
    if (currentTime == Alarm2){
      updateDate2 = true;
  }else{
      oneTimePass2 = true;
  }
    if (currentTime == Alarm3){
      updateDate3 = true;
  }else{
      oneTimePass3 = true;
  }

    if (updateDate1 && updateDate2 && updateDate3){
       
      
      if (!(Alarm1==Alarm2 && Alarm1==Alarm3 &&Alarm1==currentTime)){
        Days= Days-1;
        updateDate1 = false;
        updateDate2 = false;
        updateDate3 = false;

        String(Days).toCharArray(DaysAr,2);
        mqttClient.publish("MediBoxDayCount",DaysAr); //Published remanning  day count

        oneTimePass1 = true;
        oneTimePass2 = true;
        oneTimePass3 = true;}
  
}
}
