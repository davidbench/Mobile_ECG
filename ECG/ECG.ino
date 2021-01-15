#include <Nextion.h>
#include "FS.h"
#include "SD.h"
#include "SPI.h"
#include "BluetoothSerial.h"
#include <Wire.h>
#include "MAX30105.h"
#include "DS3231M.h"
#include <Sodaq_BMP085.h>
#include <ADXL345.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

#define VERBOSE   1
#define REPORT    1
#define TIMING    1
#define ERROR_MSG 1

/*
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

#define USEFIFO
*/


BluetoothSerial SerialBT;

TwoWire I2C2 = TwoWire(1);
MAX30105 particleSensor;
DS3231M_Class DS3231M;  ///< Create an instance of the DS3231M class
Sodaq_BMP085 bmp;
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified();

NexWaveform s0 = NexWaveform(0, 1, "s0");
NexButton bDelay1 = NexButton(0, 4, "b2");
NexButton bDelay0 = NexButton(0, 5, "b3");

NexTouch *nex_listen_list[] = {
    &bDelay1,
    &bDelay0,
    NULL
};

String dataString = "";

const byte SDA2   = 33;
const byte SCL2   = 32;
const byte ecgPin = 34;

unsigned int ecgValue = 0;
bool alert_button = false;

const int  Interval_High  = 1;      //through delay
      int      Last_High  = 0;      //unused
const int  Interval_Med   = 200;
      long     Last_Med   = 0;
const int  Interval_Low   = 5000;
      long     Last_Low   = 0;
      int  Interval_Chart = 20;
      long     Last_Chart = 0;

int i = 0;


// MAX stuff
double ESpO2 = 95.0;//initial value of estimated SpO2
const double FSpO2 = 0.7; //filter factor for estimated SpO2
const double frate = 0.95; //low pass filter for IR/Red LED value to eliminate AC component
int SpO2_Samp = 100;//calculate SpO2 by this sampling interval
const int timeFromBoot = 3000; // wait for this time(msec) to output SpO2

double Red_AVG = 0; double IR_AVG = 0;
double IR_SUM_RMS = 0;
double Red_SUM_RMS = 0;
uint32_t IR, Red;
double Red_f, IR_f, SpO2; //raw SpO2 before low pass filtered
float IR_forGraph, Red_forGraph;
int acc_x, acc_y, acc_z;
float Temperature, Altitude, RealAltitude;
int32_t Pressure;
float headingDegrees;
float pitchDegrees;
double R;
sensors_event_t mag_event;

TaskHandle_t xHandle_update_chart = NULL;
TaskHandle_t xHandle_get_Med = NULL;
TaskHandle_t xHandle_get_Low = NULL;

unsigned long micro_align = 0;
unsigned long micros_tt[10] = {0,0,0,0,0,0,0,0,0,0};

byte sample_H[10];
byte sample_M[10];
byte sample_L[21];

bool I2C1_inuse = false;

typedef struct {
    File file;
    long bytes_written;
    DateTime created;
    DateTime created_ref;
    long created_micro;
    long limit_bytes;
    unsigned int limit_seconds;
} out_file;

out_file out_files[3]; //0 = High, 1 = Med, 2 = Low

DateTime get_clock(void) {
  return DS3231M.now();
}

DateTime timenow;

void bDelay1Callback(void *ptr) {
    Interval_Chart--;
    Serial.println("bDelay1Callback");
}

void bDelay0Callback(void *ptr) {
    Interval_Chart++;
    Serial.println("bDelay0Callback");
}

int tt(int slot = 0) {
  if (!micros_tt[slot]) {
    micros_tt[slot] = micros();
    return 0;
  }
  else {
    long t = micros();   
    int rtn = t - micros_tt[slot];
    micros_tt[slot] = t;
    return rtn;
  }
}

void SO(bool EN, String msg, bool CR = true) {
    if (!EN) return;
    if (CR) Serial.println(msg);
    else Serial.print(msg);
}

void open_file(byte fn) {
    tt(1);
    SO(VERBOSE,"Opening new file",0);
    SO(VERBOSE,String(fn),0);
    SO(VERBOSE," : ",0);
    DateTime filenow = get_clock();
    char speed = fn ? ((fn == 1) ? 'M' : 'L') : 'H';
    char * filename = get_filename(filenow,speed);
    SO(VERBOSE,filename);
    out_files[0].file = SD.open(filename, FILE_WRITE);
    out_files[0].bytes_written = 0;
    out_files[0].created = filenow;
    out_files[0].created_ref = DateTime(
            filenow.year(), filenow.month() , filenow.day(),
            filenow.hour(), filenow.minute(), 0);
    out_files[0].created_micro = micros();
    SO(TIMING,"  Time: ",0);
    SO(TIMING,String(tt(1)),0);
    SO(TIMING," us");
}

void file_write(byte fn, const unsigned char * data_packet, unsigned long data_length, bool time_sens = false) {
    out_files[fn].file.write(data_packet, data_length);
    out_files[fn].bytes_written += data_length;
    
    SO(VERBOSE,String(data_length),0);
    SO(VERBOSE," Bytes Written to File",0);
    SO(VERBOSE,String(fn),0);
    SO(VERBOSE,".");
    SO(VERBOSE,"   File Size : ",0);
    SO(VERBOSE,String(out_files[fn].bytes_written));
    
        
    if ((
        (out_files[fn].bytes_written >= out_files[fn].limit_bytes) ||
        ((get_clock() - out_files[fn].created).totalseconds() >= out_files[fn].limit_seconds)) &&
        !time_sens) {
          out_files[0].file.close();
          open_file(fn);
    }
}

void setup() {
    Serial.begin(115200);
    
    SerialBT.begin("ESP32tester"); //Bluetooth device name
    
    //I2C #1
    bmp.begin();    
    mag.begin();
    adxl.powerOn();
    
    nexInit();      //Serial
    SDcard_init();  //SPI
    
    //I2C #2
    I2C2.begin(SDA2, SCL2, I2C_SPEED_FAST);
    particleSensor.begin(I2C2, I2C_SPEED_FAST);
    DS3231M.begin(I2C2, I2C_SPEED_FAST);
    MAX_setup();
    
    //Nextion callbacks
    bDelay1.attachPop(bDelay1Callback, &bDelay1);
    bDelay0.attachPop(bDelay0Callback, &bDelay0);   

      DS3231M.adjust(DateTime(2021, 1, 12, 2, 8, 40)); delay(2000);
    DateTime check = get_clock();
    SO_clock(VERBOSE, check);

    out_files[0].limit_bytes   = 125000;
    out_files[0].limit_seconds = 125;
    out_files[1].limit_bytes   = 30000;
    out_files[1].limit_seconds = 240;
    out_files[2].limit_bytes   = 15000;
    out_files[2].limit_seconds = 600;

    open_file(0);
    open_file(1);
    open_file(2);

    SO(VERBOSE,"Spawning get_Low on Core0");
    xTaskCreatePinnedToCore(get_Low, "get_Low", 10000, NULL, 1, &xHandle_get_Low, 0);
    configASSERT( xHandle_get_Low );
    SO(VERBOSE,"Spawning get_Med on Core0");
    xTaskCreatePinnedToCore(get_Med, "get_Med", 10000, NULL, 1, &xHandle_get_Med, 0);
    configASSERT( xHandle_get_Med );
    SO(VERBOSE,"Spawning update_chart on Core0");
    xTaskCreatePinnedToCore(update_chart, "update_chart", 10000, NULL, 1, &xHandle_update_chart, 0);
    configASSERT( xHandle_update_chart );
}

void update_chart(void * params) {
  while(1) {
    if ((millis() - Last_Chart) > Interval_Chart) {
    Last_Chart = millis();
    s0.addValue(0, 25 + ecgValue/5);
    s0.addValue(1, map(IR/100, 1450,1550,0,200));
    s0.addValue(2, map(Red/100, 1450,1550,0,200));
    //while(1){delay(1);}
    }
    delay(7);
  }
}

void get_Med(void * params) {
  while(1){
  if ((millis() - Last_Med) > Interval_Med) {
    while (I2C1_inuse) {delay(1);};
    I2C1_inuse = true;
    Last_Med = millis();
    get_acc();
    //SO(TIMING,"T: to ACC   = ",0); SO(TIMING,String(tt(0)));   
    get_mag();
    //SO(TIMING,"T: to mag   = ",0); SO(TIMING,String(tt(0)));
    I2C1_inuse = false;
    //file_write(1, sample_M, 10);
    
  }
  delay(50);
  }
    //while(1){delay(1);}
    
}

byte * float2bytes(float float_var) {
    byte O[4];
    *(float*)(O) = float_var;
    return O;
}

void get_Low(void * params) {
    while(1){
    if ((millis() - Last_Low) > Interval_Low) {
      
    Last_Low = millis();
    if (I2C1_inuse) {
      delay(52);
      if (I2C1_inuse) {
        delay(17);
        if (I2C1_inuse) {
          continue;
        }
      }
    }
    I2C1_inuse = true;
    //long micro_before = micros();
    //tt(6);
    get_BMP();
    //SO(TIMING,"T: to BMP   = ",0); SO(TIMING,String(tt(6)));
    I2C1_inuse = false;
    
    get_button();
    //SO(TIMING,"T: to Butto   = ",0); SO(TIMING,String(tt(6)));
    
/*
    int seconds_diff = (out_files[0].created_ref - timenow).totalseconds();

    byte Temperature_bytes[4];
    *(float*)(Temperature_bytes) = Temperature;
    byte Altitude_bytes[4];
    *(float*)(Altitude_bytes) = Altitude;
    byte RealAltitude_bytes[4];
    *(float*)(RealAltitude_bytes) = RealAltitude`;

    int32_t Pressure;
    
    sample_L[0]  = (byte)(seconds_diff >> 2);
    sample_L[1]  = (byte)((byte)(seconds_diff << 6) | ((byte)(micro_before >> 16) & B00111111));
    sample_L[2]  = (byte)(micro_before >> 8);
    sample_L[3]  = (byte)(micro_before);
    
    sample_L[4]  = (byte)(Temperature_bytes[0]);
    sample_L[5]  = (byte)(Temperature_bytes[1]);
    sample_L[6]  = (byte)(Temperature_bytes[2]);
    sample_L[7]  = (byte)(Temperature_bytes[3]);
    
    sample_L[8]  = (byte)(Altitude_bytes[0]);
    sample_L[9]  = (byte)(Altitude_bytes[1]);
    sample_L[10] = (byte)(Altitude_bytes[2]);
    sample_L[11] = (byte)(Altitude_bytes[3]);

    sample_L[12] = (byte)(RealAltitude_bytes[0]);
    sample_L[13] = (byte)(RealAltitude_bytes[1]);
    sample_L[14] = (byte)(RealAltitude_bytes[2]);
    sample_L[15] = (byte)(RealAltitude_bytes[3]);

    sample_L[16] = (byte)(Pressure >> 24);
    sample_L[17] = (byte)(Pressure >> 16);
    sample_L[18] = (byte)(Pressure >> 8);
    sample_L[19] = (byte)(Pressure);

    sample_L[20] = (byte)(alert_button);   
    
    file_write(2, sample_L, 21);
    */
    }
    delay(1000);
    }
    //while(1){delay(1);}
}



void loop(void) {

    
    SO(TIMING,"T: to Start = ",0); SO(TIMING,String(tt(0)));
    timenow = get_clock();
    SO_clock(REPORT, timenow);
    long micro_before = micros();
    SO(TIMING,"T: to Clock = ",0); SO(TIMING,String(tt(0)));
    get_ecg();
    SO(TIMING,"T: to ECG   = ",0); SO(TIMING,String(tt(0)));
    get_MAX();
    SO(TIMING,"T: to MAX   = ",0); SO(TIMING,String(tt(0)));

    int seconds_diff = (out_files[0].created_ref - timenow).totalseconds();
    
    sample_H[0] = (byte)(seconds_diff >> 2);
    sample_H[1] = (byte)((byte)(seconds_diff << 6) | ((byte)(micro_before >> 16) & B00111111));
    sample_H[2] = (byte)(micro_before >> 8);
    sample_H[3] = (byte)(micro_before);
    sample_H[4] = (byte)(ecgValue >> 4);
    sample_H[5] = (byte)((byte)(ecgValue << 4) | ((byte)(Red >> 14) & B00001111));
    sample_H[6] = (byte)(Red >> 6);
    sample_H[7] = (byte)((byte)(Red << 2) | ((byte)(IR >> 16) & B00000011));
    sample_H[8] = (byte)(IR >> 8);
    sample_H[9] = (byte)(IR);

    SO(TIMING,"T: to sample prep = ",0); SO(TIMING,String(tt(0)));
    
//    dataString += String(ecgValue);
//    dataString += ","; 

    SO(TIMING,"T: to Buffe   = ",0); SO(TIMING,String(tt(0)));
/*
    if ((millis() - Last_Chart) > Interval_Chart) {
        if( xHandle_update_chart != NULL ) {
            vTaskDelete( xHandle_update_chart );
            xHandle_update_chart = NULL;
        }
        Last_Chart = millis();
        SO(VERBOSE,"Spawning update_chart on Core0");
        xTaskCreatePinnedToCore(update_chart, "update_chart", 10000, NULL, 1, &xHandle_update_chart, 0);
        //update_chart();
        configASSERT( xHandle_update_chart );
    }

    if ((millis() - Last_Low) > Interval_Low) {
        if( xHandle_get_Low != NULL ) {
            vTaskDelete( xHandle_get_Low );
            xHandle_get_Low = NULL;
        }
        Last_Low = millis();
        SO(VERBOSE,"Spawning get_Low on Core0");
        xTaskCreatePinnedToCore(get_Low, "get_Low", 10000, NULL, 1, &xHandle_get_Low, 0);
        configASSERT( xHandle_get_Low );
    }

    if ((millis() - Last_Med) > Interval_Med) {
        if( xHandle_get_Med != NULL ) {
            vTaskDelete( xHandle_get_Med );
            xHandle_get_Med = NULL;
        }
        Last_Med = millis();
        SO(VERBOSE,"Spawning get_Med on Core0");
        xTaskCreatePinnedToCore(get_Med, "get_Med", 10000, NULL, 1, &xHandle_get_Med, 0);
        configASSERT( xHandle_get_Med );
    }
*/
    SO(TIMING,"T: to Core0   = ",0); SO(TIMING,String(tt(0)));
    
    delay(Interval_High);
    SO(TIMING,"T: to Delay   = ",0); SO(TIMING,String(tt(0)));
    
    nexLoop(nex_listen_list);

    SO(TIMING,"T:to NLoop   = ",0); SO(TIMING,String(tt(0)));
    

    
    /*
    if ((millis() - Last_Low) > Interval_Low) {
        File dataFile = SD.open("/datalog.txt", FILE_WRITE);
        
        if (dataFile) {
            dataFile.println(dataString); //print
            dataFile.close();
        
            SO(VERBOSE,"File Write");
            dataString = "";
        }
        else {
            SO(ERROR_MSG,"error opening datalog.txt");
        }
        
        Last_Low = millis();
    }
    */

    file_write(0, sample_H, 10);
    SO(TIMING,"T: to SaveF   = ",0); SO(TIMING,String(tt(0)));


    
}




char * get_filename(DateTime filetime, char speed) {
    SO(VERBOSE, "Getting file name for time:");
    SO(VERBOSE, "     ",0);
    SO_clock(VERBOSE, filetime);
    
    static char filename[20];  /// if there are issues turn this to static char
    //https://forum.arduino.cc/index.php?topic=63659.0

    sprintf(filename, "/%1d%1d%02d%02d%02d.dSo"
                        ,filetime.year()%10
                        ,timenow.month()%10
                        ,timenow.day()
                        ,timenow.hour()
                        ,timenow.minute()
                        );
    if (timenow.month()==10) filename[2] = 'O';
    if (timenow.month()==11) filename[2] = 'N';
    if (timenow.month()==12) filename[2] = 'R';
    filename[11] = speed;
   
    SO(VERBOSE, "Filename: ",0);
    SO(VERBOSE, filename);
    return filename;
}


void get_BMP(void) {
    Temperature = bmp.readTemperature();
    Pressure = bmp.readPressure();
    Altitude = bmp.readAltitude();
    RealAltitude = bmp.readAltitude(101500); //sea level pressure
    SO(REPORT,"BMP: Temperature = ",0);  SO(REPORT,String(Temperature),0);   SO(REPORT," *C");
    SO(REPORT,"     Pressure = ",0);     SO(REPORT,String(Pressure),0);      SO(REPORT," Pa");
    SO(REPORT,"     Altitude = ",0);     SO(REPORT,String(Altitude),0);      SO(REPORT," meters");
    SO(REPORT,"     Real altitude = ",0);SO(REPORT,String(RealAltitude),0);  SO(REPORT," meters");
}

void get_MAX(void) {
    
    particleSensor.check(); //Check the sensor, read up to 3 samples
    
    
    while (particleSensor.available()) {
        SO(1,"MAX: Data Available");
        Red = particleSensor.getFIFOIR(); //why getFIFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
        IR = particleSensor.getFIFORed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
        process_particle();
        particleSensor.nextSample();
    }
    

    //particleSensor.check(); //Check the sensor, read up to 3 samples
    //if (particleSensor.available()) {
    //    Red = particleSensor.getIR(); //why getFIFOIR output Red data by MAX30102 on MH-ET LIVE breakout board
      //  IR = particleSensor.getRed(); //why getFIFORed output IR data by MAX30102 on MH-ET LIVE breakout board
  
        SO(REPORT,"MAX: Raw    = RED(",0);   SO(REPORT,String(Red),0);
        SO(REPORT,"), IR(",0);      SO(REPORT,String(IR),0); SO(REPORT,")");
        SO(REPORT,"     Filter = RED(",0);   SO(REPORT,String(Red_f),0);
        SO(REPORT,"), IR(",0);      SO(REPORT,String(IR_f),0); SO(REPORT,")");
        SO(REPORT,"     AVG    = RED(",0);   SO(REPORT,String(Red_AVG),0);
        SO(REPORT,"), IR(",0);      SO(REPORT,String(IR_AVG),0); SO(REPORT,")");
        SO(REPORT,"     SUMRMS = RED(",0);   SO(REPORT,String(Red_SUM_RMS),0);
        SO(REPORT,"), IR(",0);      SO(REPORT,String(IR_SUM_RMS),0); SO(REPORT,")");
        SO(REPORT,"     Graph  = RED(",0);   SO(REPORT,String(Red_forGraph),0);
        SO(REPORT,"), IR(",0);      SO(REPORT,String(IR_forGraph),0); SO(REPORT,")");
        SO(REPORT,"     Graph  = R: ",0);   SO(REPORT,String(R),0);
        SO(REPORT,", SpO2: ",0);      SO(REPORT,String(SpO2),0);
        SO(REPORT,", ESpO2: ",0);      SO(REPORT,String(ESpO2));
        SO(TIMING,"R2R MAX = ",0); SO(TIMING,String(tt(2)),0);  SO(TIMING," us");
    //}
    //else {
        //SO(REPORT || VERBOSE,"MAX: No new data");
    //}
        

}

void get_button(void) {

}

void get_ecg(void) {
    ecgValue = analogRead(ecgPin);
    SO(REPORT,"ECG: ",0); SO(REPORT,String(ecgValue));
    SO(TIMING,"R2R ECG = ",0); SO(TIMING,String(tt(3)),0);  SO(TIMING," us");
}

void get_acc(void) {
    adxl.readAccel(&acc_x, &acc_y, &acc_z); //read the accelerometer values and store them in variables  x,y,z
    // Output x,y,z values
    SO(REPORT,"Acc: ",0);   SO(REPORT,String(acc_x),0);
    SO(REPORT,", ",0);      SO(REPORT,String(acc_y),0);
    SO(REPORT,", ",0);      SO(REPORT,String(acc_z));
    SO(TIMING,"R2R acc = ",0); SO(TIMING,String(tt(4)),0);  SO(TIMING," us");
}

void get_mag(void) {
    mag.getEvent(&mag_event);
    SO(REPORT,"Mag: ",0);   SO(REPORT,String(mag_event.magnetic.x),0);
    SO(REPORT,", ",0);      SO(REPORT,String(mag_event.magnetic.y),0);
    SO(REPORT,",  ",0);     SO(REPORT,String(mag_event.magnetic.z),0);
    SO(REPORT," uT");

    // Hold the module so that Z is pointing 'up' and you can measure the heading with x&y
    // Calculate heading when the magnetometer is level, then correct for signs of axis.
    float heading = atan2(mag_event.magnetic.y, mag_event.magnetic.x);    
    float pitch = atan2(sqrt(pow(mag_event.magnetic.x,2) + pow(mag_event.magnetic.y,2)), mag_event.magnetic.z);
    // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
    // Find yours here: http://www.magnetic-declination.com/
    // Mine is: -13* 2' W, which is ~13 Degrees, or (which we need) 0.22 radians
    // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
    float declinationAngle = 0.22;
    heading += declinationAngle;
    // Correct for when signs are reversed.
    if(heading < 0)
      heading += 2*PI;
    // Check for wrap due to addition of declination.
    if(heading > 2*PI)
      heading -= 2*PI;
    // Convert radians to degrees for readability.
    headingDegrees  = heading   * 180/M_PI; 
    pitchDegrees    = pitch     * 180/M_PI;
    SO(REPORT,"     Heading = ",0); SO(REPORT,String(headingDegrees),0);  SO(REPORT," Deg");
    SO(REPORT,"     Pitch   = ",0); SO(REPORT,String(pitchDegrees),0);    SO(REPORT," Deg");

    SO(TIMING,"R2R mag = ",0); SO(TIMING,String(tt(5)),0);  SO(TIMING," us");
}



void SO_clock(bool SO_mode, DateTime timenow) {
    char output_buffer[32];
    sprintf(output_buffer, "RTC:  %04d-%02d-%02d  %02d:%02d:%02d",
            timenow.year(), timenow.month(), timenow.day(),
            timenow.hour(), timenow.minute(), timenow.second());
    SO(SO_mode, output_buffer);
}

void MAX_setup(void) {
    //MAX Setup
    byte ledBrightness = 0xFF; //0x7F; //Options: 0=Off to 255=50mA
    byte sampleAverage = 4;//4; //Options: 1, 2, 4, 8, 16, 32
    byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
                      //Options: 1 = IR only,  2 = Red + IR on MH-ET LIVE MAX30102 board
    int sampleRate = 1600;//200; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
    int pulseWidth = 411; //411//Options: 69, 118, 215, 411
    int adcRange = 16384; //Options: 2048, 4096, 8192, 16384
    particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these settings
    // Period = 1000/sampleRate*sampleAverage*ledMode ??
    
}

void SDcard_init(void) {
    if(!SD.begin()){
        SO(ERROR_MSG,"Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        SO(ERROR_MSG,"No SD card attached");
        return;
    }

    SO(VERBOSE,"SD Card Type: ",0);
    if(cardType == CARD_MMC){
        SO(VERBOSE,"MMC");
    } else if(cardType == CARD_SD){
        SO(VERBOSE,"SDSC");
    } else if(cardType == CARD_SDHC){
        SO(VERBOSE,"SDHC");
    } else {
        SO(VERBOSE,"UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    char output_buffer[30];
    sprintf(output_buffer, "SD Card Size: %lluMB\n", cardSize);
    SO(VERBOSE, output_buffer);
    sprintf(output_buffer,"Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    SO(VERBOSE, output_buffer);
    sprintf(output_buffer,"Used space: %lluMB\n",   SD.usedBytes() / (1024 * 1024));
    SO(VERBOSE, output_buffer);
}


void process_particle(void) {   
    i++;
    Red_f = (double)Red;
    IR_f = (double)IR;

    Red_AVG = Red_AVG * frate + (double)Red * (1.0 - frate);//average Red level by low pass filter
    IR_AVG = IR_AVG * frate + (double)IR * (1.0 - frate);//average Red level by low pass filter
    Red_SUM_RMS += (Red_f - Red_AVG) * (Red_f - Red_AVG);//square sum of alternate component of IR level
    IR_SUM_RMS += (IR_f - IR_AVG) * (IR_f - IR_AVG);//square sum of alternate component of IR level

    if ( millis() > timeFromBoot) {
        IR_forGraph = (2.0 * IR_f - IR_AVG) / IR_AVG * 88;
        Red_forGraph = (2.0 * Red_f - Red_AVG) / Red_AVG * 88;
    }
    
    if ((i % SpO2_Samp) == 0) { // calculate SpO2 - low sampling interval
      R = (sqrt(Red_SUM_RMS) / Red_AVG) / (sqrt(IR_SUM_RMS) / IR_AVG);
      SpO2 = -23.3 * (R - 0.4) + 100; //http://ww1.microchip.com/downloads/jp/AppNotes/00001525B_JP.pdf
      ESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
      Red_SUM_RMS = 0.0; IR_SUM_RMS = 0.0; i = 0;
    }
}

    











/*
  if (Serial.available()) {
    SerialBT.write(Serial.read());
  }
  if (SerialBT.available()) {
    Serial.write(SerialBT.read());
  }
  */

// word(high, low); // returns the integer value of the high and low bytes
    // highByte(int);
    // lowByte(int);
    // << 8 and | (bitwise)
    // https://www.arduino.cc/reference/en/language/structure/bitwise-operators/bitshiftleft/
    // https://playground.arduino.cc/Code/BitMath/#bit_shift



//multithread

/*

Both the SETUP and the main functions of the LOOP are executed with a priority of 1 and in core 1. (not on 0)
https://www.instructables.com/ESP32-With-Arduino-IDE-Multi-Core-Programming/

 * #include <WIRe.h>
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


//file system
  /*if ((i % SpO2_Samp) == 0) { // calculate SpO2 - low sampling interval
        
    double R = (sqrt(Red_SUM_RMS) / Red_AVG) / (sqrt(IR_SUM_RMS) / IR_AVG);
    
   *  = IR_AVG * frate + (double)IR * (1.0 - frate); //average IR level by low pass filter
    sESpO2 = FSpO2 * ESpO2 + (1.0 - FSpO2) * SpO2;//low pass filter
        Red_SUM_RMS = 0.0; IR_SUM_RMS = 0.0; i = 0;
    }
   * void listDIR(fs::FS &fs, const char * dIRname, uint8_t levels){
    Serial.printf("Listing dIRectory: %s\n", dIRname);

    File root = fs.open(dIRname);
    if(!root){
        Serial.println("Failed to open dIRectory");
        return;
    }
    if(!root.isDIRectory()){
        Serial.println("Not a dIRectory");
        return;
    }

    File file = root.openNextFile();
    while(file){
        if(file.isDIRectory()){
            Serial.print("  DIR : ");
            Serial.println(file.name());
            if(levels){
                listDIR(fs, file.name(), levels -1);
            }
        } else {
            Serial.print("  FILE: ");
            Serial.print(file.name());
            Serial.print("  SIZE: ");
            Serial.println(file.size());
        }
        file = root.openNextFile();
    }
}

void createDIR(fs::FS &fs, const char * path){
    Serial.printf("Creating DIR: %s\n", path);
    if(fs.mkdIR(path)){
        Serial.println("DIR created");
    } else {
        Serial.println("mkdIR failed");
    }
}

void removeDIR(fs::FS &fs, const char * path){
    Serial.printf("Removing DIR: %s\n", path);
    if(fs.rmdIR(path)){
        Serial.println("DIR removed");
    } else {
        Serial.println("rmdIR failed");
    }
}

void readFile(fs::FS &fs, const char * path){
    Serial.printf("Reading file: %s\n", path);

    File file = fs.open(path);
    if(!file){
        Serial.println("Failed to open file for reading");
        return;
    }

    Serial.print("Read from file: ");
    while(file.available()){
        Serial.write(file.read());
    }
    file.close();
}

void writeFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Writing file: %s\n", path);

    File file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }
    if(file.print(message)){
        Serial.println("File written");
    } else {
        Serial.println("Write failed");
    }
    file.close();
}

void appendFile(fs::FS &fs, const char * path, const char * message){
    Serial.printf("Appending to file: %s\n", path);

    File file = fs.open(path, FILE_APPEND);
    if(!file){
        Serial.println("Failed to open file for appending");
        return;
    }
    if(file.print(message)){
        Serial.println("Message appended");
    } else {
        Serial.println("Append failed");
    }
    file.close();
}

void renameFile(fs::FS &fs, const char * path1, const char * path2){
    Serial.printf("Renaming file %s to %s\n", path1, path2);
    if (fs.rename(path1, path2)) {
        Serial.println("File rena");
    } else {
        Serial.println("Rename failed");
    }
}

void deleteFile(fs::FS &fs, const char * path){
    Serial.printf("Deleting file: %s\n", path);
    if(fs.remove(path)){
        Serial.println("File deleted");
    } else {
        Serial.println("Delete failed");
    }
}

void testFileIO(fs::FS &fs, const char * path){
    File file = fs.open(path);
    static uint8_t buf[512];
    size_t len = 0;
    uint32_t start = millis();
    uint32_t end = start;
    if(file){
        len = file.size();
        size_t flen = len;
        start = millis();
        while(len){
            size_t toRead = len;
            if(toRead > 512){
                toRead = 512;
            }
            file.read(buf, toRead);
            len -= toRead;
        }
        end = millis() - start;
        Serial.printf("%u bytes read for %u ms\n", flen, end);
        file.close();
    } else {
        Serial.println("Failed to open file for reading");
    }


    file = fs.open(path, FILE_WRITE);
    if(!file){
        Serial.println("Failed to open file for writing");
        return;
    }

    size_t i;
    start = millis();
    for(i=0; i<2048; i++){
        file.write(buf, 512);
    }
    end = millis() - start;
    Serial.printf("%u bytes written for %u ms\n", 2048 * 512, end);
    file.close();
}

void setup(){
    Serial.begin(115200);
    if(!SD.begin()){
        Serial.println("Card Mount Failed");
        return;
    }
    uint8_t cardType = SD.cardType();

    if(cardType == CARD_NONE){
        Serial.println("No SD card attached");
        return;
    }

    Serial.print("SD Card Type: ");
    if(cardType == CARD_MMC){
        Serial.println("MMC");
    } else if(cardType == CARD_SD){
        Serial.println("SDSC");
    } else if(cardType == CARD_SDHC){
        Serial.println("SDHC");
    } else {
        Serial.println("UNKNOWN");
    }

    uint64_t cardSize = SD.cardSize() / (1024 * 1024);
    Serial.printf("SD Card Size: %lluMB\n", cardSize);

    listDIR(SD, "/", 0);
    createDIR(SD, "/mydIR");
    listDIR(SD, "/", 0);
    removeDIR(SD, "/mydIR");
    listDIR(SD, "/", 2);
    writeFile(SD, "/hello.txt", "Hello ");
    appendFile(SD, "/hello.txt", "World!\n");
    readFile(SD, "/hello.txt");
    deleteFile(SD, "/foo.txt");
    renameFile(SD, "/hello.txt", "/foo.txt");
    readFile(SD, "/foo.txt");
    testFileIO(SD, "/test.txt");
    Serial.printf("Total space: %lluMB\n", SD.totalBytes() / (1024 * 1024));
    Serial.printf("Used space: %lluMB\n", SD.usedBytes() / (1024 * 1024));
}


*/
