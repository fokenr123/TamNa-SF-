#include <MQ135.h>
#include <dht.h>
//시간 설정 나중에 WIFI로 시간 받아오기
#include <core_build_options.h>
#include <swRTC.h>

swRTC rtc;
dht DHT;

//Relay
#define PIN2 2 // hiter relay
#define PIN3 3 // PC Fan realy
#define PIN4 4 // LED relay
#define MAINWP 5 // water pump
#define SUBWP1_PIN 6 //PH WaterPump +
#define SUBWP2_PIN 7 //PH WaterPump -
#define SUBWP3_PIN 8 //EC WaterPump +
#define SUBWP4_PIN 9 //EC WaterPump -
#define DHT22_PIN 10 //DHT22 Sensor
//MQ135 Sensor
#define ANALOG_MQ135_PIN 11    //  Define Analog PIN on Arduino Board
#define RZERO 206.85    //  Define RZERO Calibration Value
//EC Sensor
#define ECINPUT_PIN 12
#define Vref 4.95

MQ135 gasSensor = MQ135(ANALOG_MQ135_PIN);

int cnt = 0, i = 0;
unsigned long int avgValue;     //Store the average value of the sensor feedback


// water pump control
bool rrp = true;
bool reading_request_phase = true;

uint32_t waterMix = 0; // main water pump
uint32_t next_poll_time = 0; // sub water pump

unsigned long int r_d = 3000000; // main
unsigned long int response_delay = 300000; // sub
unsigned long int mainWP_running_time = 5000; // main
unsigned long int subWP_running_time = 600000; //sub

struct
{
  uint32_t total;
  uint32_t ok;
  uint32_t crc_error;
  uint32_t time_out;
  uint32_t connect;
  uint32_t ack_l;
  uint32_t ack_h;
  uint32_t unknown;
} stat[10] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup()
{
  // led time on/off -추후 wifi로 실시간 시간 수집
  rtc.stopRTC();
  rtc.setTime(16,16,00); // 시, 분, 초
  rtc.setDate(14,2,2020); //일, 월, 년
  rtc.startRTC();
  Serial.begin(9600);
  
  // Dht Serial setup
  Serial.begin(9600);
  Serial.println("dht22_test.ino");
  Serial.print("LIBRARY VERSION: ");
  Serial.println(DHT_LIB_VERSION);
  Serial.println();
  Serial.println("Type,\tstatus,\tHumidity (%),\tTemperature (C)");
  // MQ135 Serial setup
  float rzero = gasSensor.getRZero();
//  delay(3000);
  Serial.print("MQ135 RZERO Calibration Value : ");
  Serial.println(rzero);
  //EC Sensor Serial setup
  pinMode(ECINPUT_PIN, INPUT);

  //Pin Relay Mode setup
  pinMode(PIN2, OUTPUT);
  digitalWrite(PIN2, HIGH);
  pinMode(PIN3, OUTPUT);
  digitalWrite(PIN3, HIGH);
  pinMode(PIN4, OUTPUT);
  digitalWrite(PIN4, HIGH);
  pinMode(MAINWP, OUTPUT);
  digitalWrite(MAINWP, HIGH);
  pinMode(SUBWP1_PIN, OUTPUT);
  digitalWrite(SUBWP1_PIN, HIGH);
  pinMode(SUBWP2_PIN, OUTPUT);
  digitalWrite(SUBWP2_PIN, HIGH);
  pinMode(SUBWP3_PIN, OUTPUT);
  digitalWrite(SUBWP3_PIN, HIGH);
  pinMode(SUBWP4_PIN, OUTPUT);
  digitalWrite(SUBWP4_PIN, HIGH);
}

void loop() {
  if(cnt < 10) {
    Serial.print("Start Time : "); 
    Serial.print(rtc.getHours(), DEC);
    Serial.print(":");
    Serial.print(rtc.getMinutes(), DEC);
    Serial.print(":");
    Serial.print(rtc.getSeconds(), DEC);
    Serial.println();
    Serial.print("---------State----------------");
    Serial.println();
    Serial.print("Sensor Count : ");
    Serial.println();
    dht_f();
    mq135_f();
    ec_f();
    cnt++;
    Serial.print("-----------machine state---------");
    Serial.println();
    // Hiter controller
    if(DHT.humidity <= 60) {
      Serial.print("Hiter on");
      digitalWrite(PIN2,LOW);
      Serial.println();
    }
    else
      Serial.print("Hiter Off"); 
      digitalWrite(PIN2,HIGH);
      Serial.println();
      
    // PC Fan controller
    if(DHT.temperature <= 10){
      Serial.print("Fan on");
      digitalWrite(PIN3,LOW);
      Serial.println();
    }
    else
      Serial.print("Fan off");
      digitalWrite(PIN3,HIGH);
      Serial.println();

    // LED controller 예)16시 20분 ~ 16시 40분 까지 led on
    if(rtc.getHours() == 16){
      if(rtc.getMinutes() == 20){
        Serial.print("LED ON");
        digitalWrite(PIN4,LOW);
    }
      else if(rtc.getMinutes() == 40){
        Serial.print("LED off"); 
        digitalWrite(PIN4,HIGH);
    }
    }
    else{
      digitalWrite("LED off");
    
    }
  delay(1000);
  }
}

//DHT Sensor Controller
void dht_f()
{
  //Serial.print("DHT22, \t");
  int chk = DHT.read22(DHT22_PIN);
  stat.total++;
  //error Check
  switch (chk) {
    //OK
    case DHTLIB_OK:
      stat.ok++;
      Serial.print("DHT22 State : OK,\t");
      Serial.println();
      break;
    //Checksum error
    case DHTLIB_ERROR_CHECKSUM:
      stat.crc_error++;
      Serial.print("DHT22 State : Checksum error,\t");
      break;
    //Time out error
    case DHTLIB_ERROR_TIMEOUT:
      stat.time_out++;
      Serial.print("DHT22 State : Time out error,\t");
      break;
    //Connect error
    case DHTLIB_ERROR_CONNECT:
      stat.connect++;
      Serial.print("DHT22 State : Connect error,\t");
      break;
    //Ack Low error
    case DHTLIB_ERROR_ACK_L:
      stat.ack_l++;
      Serial.print("DHT22 State : Ack Low error,\t");
      break;
    //Ack High error
    case DHTLIB_ERROR_ACK_H:
      stat.ack_h++;
      Serial.print("Ack High error,\t");
      break;
    // Unknown error
    default:
      stat.unknown++;
      Serial.print("Unknown error,\t");
      break;
  }
  // DISPLAY DATA
  Serial.print("humidity : ");
  Serial.print(DHT.humidity, 1);
  Serial.println();
  Serial.print("Temperature : ");
  Serial.print(DHT.temperature, 1);
  Serial.println();

  if (stat.total % 20 == 0)
  {
    Serial.println("\nTOT\tOK\tCRC\tTO\tCON\tACK_L\tACK_H\tUNK");
    Serial.print(stat.total);
    Serial.print("\t");
    Serial.print(stat.ok);
    Serial.print("\t");
    Serial.print(stat.crc_error);
    Serial.print("\t");
    Serial.print(stat.time_out);
    Serial.print("\t");
    Serial.print(stat.connect);
    Serial.print("\t");
    Serial.print(stat.ack_l);
    Serial.print("\t");
    Serial.print(stat.ack_h);
    Serial.print("\t");
    Serial.print(stat.unknown);
    Serial.println("\n");
  }
  //delay(3000);

  waterPumpCon();

}

void mq135_f() {
  float ppm = gasSensor.getPPM();
  //delay(30000);
  digitalWrite(13,HIGH);
  Serial.print("CO2 Value : ");
  Serial.println(ppm);
}

void ec_f(){
  float sensorValue;
  int m;
  long sensorSum;
  int buf[10];                //buffer for read analog
  for(int i=0;i<10;i++)       //Get 10 sample value from the sensor for smooth the value
  { 
    buf[i]=analogRead(ECINPUT_PIN);//Connect the PH Sensor to ECINPUT_PIN port
    //delay(10);
  }
  for(int i=0;i<9;i++)        //sort the analog from small to large
  {
    for(int j=i+1;j<10;j++)
    {
      if(buf[i]>buf[j])
      {
        int temp=buf[i];
        buf[i]=buf[j];
        buf[j]=temp;
      }
    }
  }
       avgValue=0;
 
      for(int i=2;i<8;i++)                      //take the average value of 6 center sample
      avgValue+=buf[i];
    
     sensorValue = avgValue/6;
     //Serial.print("SensorValue : ");
     //Serial.print(sensorValue);
     //Serial.println();

    Serial.print("EC Value : ");
    Serial.print(7-1000*(sensorValue-365)*Vref/59.16/1023,2);
    Serial.println(" ");
   //delay(1000);
}


void relay_test_f() {
  //Pin2 Control
  digitalWrite(PIN2, HIGH);
  //Pin3 Control
  //digitalWrite(PIN3, HIGH);
  //Pin4 Control
  digitalWrite(PIN4, HIGH);
  //MAINWP Control
  digitalWrite(MAINWP, HIGH);
  //SUB WaterPump1 _ Control
  digitalWrite(SUBWP1_PIN, HIGH);
  //SUB WaterPump2 _ Control
  digitalWrite(SUBWP2_PIN, HIGH);
  //SUB WaterPump3 _ Control
  digitalWrite(SUBWP3_PIN, HIGH);
  //SUB WaterPump4 _ Control
  digitalWrite(SUBWP4_PIN, HIGH);

  delay(3000);  
  digitalWrite(PIN2, LOW);
  digitalWrite(PIN3, LOW);
  digitalWrite(PIN4, LOW);
  digitalWrite(MAINWP, LOW);
  digitalWrite(SUBWP1_PIN, LOW);
  digitalWrite(SUBWP2_PIN, LOW);
  digitalWrite(SUBWP3_PIN, LOW);
  digitalWrite(SUBWP4_PIN, LOW);

  delay(3000);
}

// water pump control code
void waterPumpCon() {
  if(rrp) {
    waterMix = millis() + r_d;
    rrp = false;
  } else {
    if(millis() >= waterMix) {
      subwaterPump_test1();
      rrp = true;
    } else {
      if(reading_request_phase) {
        next_poll_time = millis() + response_delay;
        reading_request_phase = false;
      } else {
        if(millis() >= next_poll_time) {
          mainwaterPump();
          reading_request_phase = true;
        }
      }
    }
  }
}

void mainwaterPump() {
  digitalWrite(MAINWP,LOW);
  delay(mainWP_running_time);
  digitalWrite(MAINWP,HIGH);
}

void subwaterPump_test1() {
  digitalWrite(SUBWP1_PIN,LOW);
  digitalWrite(SUBWP2_PIN,LOW);
  digitalWrite(SUBWP3_PIN,LOW);
  digitalWrite(SUBWP4_PIN,LOW);
  delay(subWP_running_time);
  digitalWrite(SUBWP1_PIN,HIGH);
  digitalWrite(SUBWP2_PIN,HIGH);
  digitalWrite(SUBWP3_PIN,HIGH);
  digitalWrite(SUBWP4_PIN,HIGH);
}
