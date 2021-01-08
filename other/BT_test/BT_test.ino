
#include <Nextion.h>

#include <SPI.h>
#include <SD.h>

#define LEVEL_LOW   (0)
#define CH0_OFFSET  (25) //Normal shaft height
#define NOTE  1000 //Tint set to buzzer

NexWaveform s0 = NexWaveform(0, 1, "s0");
NexDSButton bt0   = NexDSButton(0, 7, "bt0"); //Button Pause/Play
NexText txt_bpm  = NexText(0, 3, "t1"); //bpm
static uint8_t ch0_data = LEVEL_LOW;
int speakerPin=2;
int value=0;
int c=0;
uint32_t dual_state;


const int chipSelect = 4;


//***********************************************
int LastTime = 0;
bool BPMTiming = false;
bool BeatComplete = false;
int BPM = 0;    
#define UpperThreshold 550
#define LowerThreshold 490    
int Signal; // holds the incoming raw data. Signal value can range from 0-1024.
char buffer[100] = {0};//bpm
//***********************************************



//This example code is in the Public Domain (or CC0 licensed, at your option.)
//By Evandro Copercini - 2018
//
//This example creates a bridge between Serial and Classical Bluetooth (SPP)
//and also demonstrate that SerialBT have the same functionalities of a normal Serial



#include "BluetoothSerial.h"
#include <Wire.h>
#include "MAX30105.h" //sparkfun MAX3010X library


#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define USEFIFO

BluetoothSerial SerialBT;
MAX30105 particleSensor;

NexButton bDelay1 = NexButton(0, 4, "b2");
NexButton bDelay0 = NexButton(0, 5, "b3");

NexTouch *nex_listen_list[] = {
    &bDelay1,
    &bDelay0,
    NULL
};

int delay_param = 10;
int save_delay = 20000;
String dataString = "";

void bDelay1Callback(void *ptr) {
    delay_param--;
    Serial.println("bDelay1Callback");
}

void bDelay0Callback(void *ptr) {
    delay_param++;
    Serial.println("bDelay0Callback");
}

const int ecgPin = 34;
int ecgValue = 0;

void setup() {
  Serial.begin(115200);
  SerialBT.begin("ESP32tester"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");


  //pinMode(speakerPin,OUTPUT);
  nexInit(); //Initializes communication with the Nextion Display.


  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  while (!SD.begin(chipSelect)) {
    Serial.println("Card Mount Failed");
    delay (5000);
  }
  Serial.println("card initialized.");
  

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30102 was not found. Please check wiring/power/solder jumper at MH-ET LIVE MAX30102 board. ");
    //while (1);
  }


  bDelay1.attachPop(bDelay1Callback, &bDelay1);
  bDelay0.attachPop(bDelay0Callback, &bDelay0);
  

  //Setup to sense a nice looking saw tooth on the plotter
  byte ledBrightness = 0x7F; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
                    //Options: 1 = IR only, 2 = Red + IR on MH-ET LIVE MAX30102 board
  int sampleRate = 200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
}



double avered = 0; double aveir = 0;
double sumirrms = 0;
double sumredrms = 0;
int i = 0;
int Num = 100;//calculate SpO2 by this sampling interval
int ecg_last = 0;
int save_last = 0;

double ESpO2 = 95.0;//initial value of estimated SpO2
double FSpO2 = 0.7; //filter factor for estimated SpO2
double frate = 0.95; //low pass filter for IR/red LED value to eliminate AC component
#define TIMETOBOOT 3000 // wait for this time(msec) to output SpO2
#define SCALE 88.0 //adjust to display heart beat and SpO2 in the same scale
#define SAMPLING 1 //if you want to see heart beat more precisely , set SAMPLING to 1
#define FINGER_ON 30000 // if red signal is lower than this , it indicates your finger is not on the sensor
#define MINIMUM_SPO2 80.0



void loop(void) {

  /*
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  */





    ecgValue = analogRead(ecgPin);
    dataString += String(ecgValue);
    dataString += ",";
    
    
    if ((millis() - ecg_last) > delay_param) {
      s0.addValue(0, CH0_OFFSET + ecgValue/5);
      ecg_last = millis();
    }
    
/*
if(c>100){// display bpm
      txt_bpm.setText(buffer); //bpm
      memset(buffer, 0, sizeof(buffer)); //bpm
      itoa(BPM, buffer, 10); //bpm
      c=0;
    }
*/

  uint32_t ir, red , green;
  double fred, fir;
  double SpO2 = 0; //raw SpO2 before low pass filtered
  float ir_forGraph = 0;
  float red_forGraph = 0;
/*
  particleSensor.check(); //Check the sensor, read up to 3 samples

  while (particleSensor.available()) {//do we have new data
    
    
    red = particleSensor.getFIFOIR(); //why getFOFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
    ir = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
    i++;
    fred = (double)red;
    fir = (double)ir;
    avered = avered * frate + (double)red * (1.0 - frate);//average red level by low pass filter
    aveir = aveir * frate + (double)ir * (1.0 - frate); //average IR level by low pass filter
    sumredrms += (fred - avered) * (fred - avered); //square sum of alternate component of red level
    sumirrms += (fir - aveir) * (fir - aveir);//square sum of alternate component of IR level

    
    if ((i % SAMPLING) == 0) {//slow down graph plotting speed for arduino Serial plotter by thin out
      if ( millis() > TIMETOBOOT) {
         ir_forGraph = (2.0 * fir - aveir) / aveir * SCALE;
         red_forGraph = (2.0 * fred - avered) / avered * SCALE;
        //trancation for Serial plotter's autoscaling
    //    if ( ir_forGraph > 100.0) ir_forGraph = 100.0;
      //  if ( ir_forGraph < 80.0) ir_forGraph = 80.0;
     //   if ( red_forGraph > 100.0 ) red_forGraph = 100.0;
     //   if ( red_forGraph < 80.0 ) red_forGraph = 80.0;
        //        Serial.print(red); Serial.print(","); Serial.print(ir);Serial.print(".");
        if (ir < FINGER_ON) ESpO2 = MINIMUM_SPO2; //indicator for finger detached
      }
    }

    
    if ((i % Num) == 0) { // calculate SpO2 - low sampling interval
      double R = (sqrt(sumredrms) / avered) / (sqrt(sumirrms) / aveir);
      // Serial.println(R);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      //  Serial.print(SpO2);Serial.print(",");Serial.println(ESpO2);
      sumredrms = 0.0; sumirrms = 0.0; i = 0;
      break;
    }

    
    particleSensor.nextSample(); //We're finished with this sample so move to next sample
    
    //Serial.println(SpO2);
  }


  //SerialBT.printf ("$%d %f %f %f;", ecgValue, aveir/100, avered/100, ESpO2);
  //Serial.printf ("$%d %f %f %f;", ecgValue, aveir/100, avered/100, ESpO2);
  */
  nexLoop(nex_listen_list);

  if ((millis() - save_last) > save_delay) {
    File dataFile = SD.open("datalog.txt", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
      dataString = "";
    }
    // if the file isn't open, pop up an error:
    else {
      Serial.println("error opening datalog.txt");
    }
    save_last = millis();
  }
  
  //delay(delay_param);
}







// Both the SETUP and the main functions of the LOOP are executed with a priority of 1 and in core 1. (not on 0)

//https://www.instructables.com/ESP32-With-Arduino-IDE-Multi-Core-Programming/


/*
 * 
 * 
 * #include <Wire.h>
#include <LiquidCrystal_IC2.h> //biblioteca responsável pelo controle do display
 
LiquidCrystal_I2C lcd(0x27, 16, 2); //set the LCD address to 0x27 for a 16 chars and 2 line display
 
int count  = 0;
int blinked = 0;
 
String statusButton = "DESATIVADO";
 
//pinos usados
const uint8_t pin_led = 4;
const uint8_t pin_btn = 2;
 
//variaveis que indicam o núcleo
static uint8_t taskCoreZero = 0;
static uint8_t taskCoreOne  = 1;

void setup() {
  pinMode(pin_led, OUTPUT);
  pinMode(pin_btn, INPUT);
 
  //inicializa o LCD com os pinos SDA e SCL
  lcd.begin(19, 23);
 
  // Liga a luz do display
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Piscadas:");
 
  //cria uma tarefa que será executada na função coreTaskZero, com prioridade 1 e execução no núcleo 0
  //coreTaskZero: piscar LED e contar quantas vezes
  xTaskCreatePinnedToCore(
                    coreTaskZero,  
                    "coreTaskZero",
                    10000,     
                    NULL,      
                    1,         
                    NULL,      
                    taskCoreZero);
                     
  delay(500); //tempo para a tarefa iniciar

//cria uma tarefa que será executada na função coreTaskOne, com prioridade 2 e execução no núcleo 1
  //coreTaskOne: atualizar as informações do display
  xTaskCreatePinnedToCore(
                    coreTaskOne, 
                    "coreTaskOne",
                    10000,   
                    NULL,    
                    2,       
                    NULL,    
                    taskCoreOne); 
 
    delay(500); //tempo para a tarefa iniciar
 
   //cria uma tarefa que será executada na função coreTaskTwo, com prioridade 2 e execução no núcleo 0
   //coreTaskTwo: vigiar o botão para detectar quando pressioná-lo
   xTaskCreatePinnedToCore(
                    coreTaskTwo,  
                    "coreTaskTwo",
                    10000,      // número de palavras a serem alocadas para uso com a pilha da tarefa 
                    NULL,       // parâmetro de entrada para a tarefa (pode ser NULL)
                    2,          // prioridade da tarefa (0 a N)
                    NULL,       // referência para a tarefa (pode ser NULL) 
                    taskCoreZero);         // Núcleo que executará a tarefa 
    
}
 
void loop() {}

*/
  
