#define ENABLE_GxEPD2_GFX 0

// uncomment next line to use class GFX of library GFX_Root instead of Adafruit_GFX
//#include <GFX.h>
// Note: if you use this with ENABLE_GxEPD2_GFX 1:
//       uncomment it in GxEPD2_GFX.h too, or add #include <GFX.h> before any #include <GxEPD2_GFX.h>

#include <GxEPD2_BW.h>
#include <GxEPD2_3C.h>
#include <GxEPD2_7C.h>
#include <Fonts/FreeMonoBold9pt7b.h>

#if defined(__AVR)
#if defined (ARDUINO_AVR_MEGA2560)
#define MAX_DISPLAY_BUFFER_SIZE 5000 // e.g. full height for 200x200
#else
#define MAX_DISPLAY_BUFFER_SIZE 800 
#endif
#define MAX_HEIGHT(EPD) (EPD::HEIGHT <= MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8) ? EPD::HEIGHT : MAX_DISPLAY_BUFFER_SIZE / (EPD::WIDTH / 8))
//GxEPD2_BW<GxEPD2_290, MAX_HEIGHT(GxEPD2_290)> display(GxEPD2_290(/*CS=10*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7));
//GxEPD2_BW<GxEPD2_290_T5, MAX_HEIGHT(GxEPD2_290_T5)> display(GxEPD2_290_T5(/*CS=10*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7)); // GDEW029T5
GxEPD2_BW<GxEPD2_420, MAX_HEIGHT(GxEPD2_420)> display(GxEPD2_420(/*CS=10*/ SS, /*DC=*/ 8, /*RST=*/ 9, /*BUSY=*/ 7));
#define MAX_HEIGHT_3C(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE / 2) / (EPD::WIDTH / 8))
#define MAX_HEIGHT_7C(EPD) (EPD::HEIGHT <= (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2) ? EPD::HEIGHT : (MAX_DISPLAY_BUFFER_SIZE) / (EPD::WIDTH / 2)) // 2 pixel per byte
#endif

#include "GxEPD2_boards_added.h"
//#include "GxEPD2_more_boards_added.h" // private


const int heartPin = A1;
const int time_x = 170;
const int time_x_gap = 60;
const int ecg_y = 50;
byte heartValue[time_x];

uint16_t ecg_max, ecg_max_next = 1023;
uint16_t ecg_min, ecg_min_next = 0;

struct DrawECGParameters
{
  uint16_t x, y, w, h;
  byte ECG_v[time_x];
};


void drawECGBoxCallback(const void* parameters)
{
  const DrawECGParameters* p = reinterpret_cast<const DrawECGParameters*>(parameters);
  display.fillRect(p->x, p->y, p->w, p->h, GxEPD_WHITE);
  for (int t = 1; t<time_x; t++)
    display.drawLine(p->x+t-1, p->y+p->ECG_v[t-1],p->x+t, p->y+p->ECG_v[t], GxEPD_BLACK);
  
  //display.drawLine(p->x,p->y,p->x+p->w,p->y,GxEPD_BLACK);
  display.drawLine(p->x,p->y,p->x,p->y+p->h,GxEPD_BLACK);
  //display.drawLine(p->x,p->y+p->h,p->x+p->w,p->y+p->h,GxEPD_BLACK);
  display.drawLine(p->x+p->w,p->y,p->x+p->w,p->y+p->h,GxEPD_BLACK); 
}

void setup()
{
  Serial.begin(115200);
  display.init(115200);

  // first update should be full refresh
  helloWorld();
  delay(1000);
  
  //display.powerOff();
  //Serial.println("setup done");
}

void loop()
{   
  //display.fillScreen(GxEPD_WHITE);

  display.setRotation(0);

  DrawECGParameters ECG_params;
  ECG_params.h = ecg_y;
  ECG_params.w = time_x;

  ecg_max_next = 0;
  ecg_min_next = 1023;

  for (ECG_params.x=0; ECG_params.x<300; ECG_params.x+=(time_x_gap+ECG_params.w))
    for (ECG_params.y=0; ECG_params.y<400; ECG_params.y+=ECG_params.h) {
      for (int t = 0; t<time_x; t++) {   
        uint16_t ecg_value = analogRead(heartPin);
        ecg_max_next = (ecg_max_next < ecg_value) ? ecg_value : ecg_max_next;
        ecg_min_next = (ecg_min_next > ecg_value) ? ecg_value : ecg_min_next;
        ECG_params.ECG_v[t] = ECG_params.h - map(ecg_value, max(ecg_min - 2,0) , min(ecg_max + 2,1023), 0, ECG_params.h);

        Serial.println(ECG_params.ECG_v[t]);
        delay(10);
      }
     
      ecg_max = ecg_max_next;
      ecg_min = ecg_min_next;
      
      display.setPartialWindow(ECG_params.x, ECG_params.y, ECG_params.w, ECG_params.h);
      display.drawPaged(drawECGBoxCallback, &ECG_params);
    
    }

}

void helloWorldCallback(const void*)
{
  //uint16_t x = (display.width() - 160) / 2;
  //uint16_t y = display.height() / 2;
  display.fillScreen(GxEPD_WHITE);
  display.drawLine(time_x,0,time_x,300,GxEPD_BLACK);
  display.drawLine(time_x+time_x_gap,0,time_x+time_x_gap,300,GxEPD_BLACK);
  
}

void helloWorld()
{
  //Serial.println("helloWorld");
  //uint16_t x = (display.width() - 160) / 2;
  //uint16_t y = display.height() / 2;
  display.setRotation(0);
  //display.setFont(&FreeMonoBold9pt7b);
  //display.setTextColor(GxEPD_BLACK);
  display.setFullWindow();
  display.drawPaged(helloWorldCallback, 0);
  //Serial.println("helloWorld done");
}


