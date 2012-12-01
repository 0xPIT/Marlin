#ifndef ULTRALCD_H
#define ULTRALCD_H

#include <Print.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <SPI.h>

#include "Marlin.h"
#include "language.h"
#include "menu/menu.h"
//#include "encoder/encoder.h"

extern Adafruit_ST7735 lcd;

extern void beep();
extern void beepshort();

extern void lcd_status();
extern void lcd_init();
extern void lcd_status(const char* message);

extern void buttons_init();
extern void buttons_check();

#define LCD_UPDATE_INTERVAL   100
#define STATUSTIMEOUT       15000

//extern volatile char buttons;  //the last checked buttons in a bit array.

namespace State {
  typedef enum Button_e {
    Undefined = 0,
    
    On = 1,
    Off,
    
    Open,
    Closed,
    
    Pressed,
    Held,
    Released,
    
    Clicked,
    DoubleClicked
    
  } Button;
};

extern volatile State::Button button;
extern State::Button getButton(void);
  
#define CLICKED (getButton() == State::Clicked)
#define BLOCK 
//{ blocking = millis() + blocktime; }

#if (SDCARDDETECT > -1)
# ifdef SDCARDDETECTINVERTED 
#  define CARDINSERTED (READ(SDCARDDETECT) != 0)
# else
#  define CARDINSERTED (READ(SDCARDDETECT) == 0)
# endif //SDCARDTETECTINVERTED
#else
  //If we don't have a card detect line, aways asume the card is inserted
# define CARDINSERTED true
#endif
  
// blocking time for recognizing a new keypress of one key, ms
#define blocktime 250
  
enum MainStatus {
  Main_Status, 
  Main_Menu, 
  Main_Prepare,
  Sub_PrepareMove,
  Main_Control,
  Main_SD,
  Sub_TempControl,
  Sub_MotionControl,
  Sub_RetractControl, 
  Sub_PreheatPLASettings, 
  Sub_PreheatABSSettings
};

class MainMenu
{
public:
  MainStatus status;
  int8_t activeline;
  uint8_t displayStartingRow;
  bool force_lcd_update;
  long lastencoderpos;
  int8_t lineoffset;
  int8_t lastlineoffset;
  bool linechanging;
  bool tune;
  
public:
  MainMenu();
  void update();  
  void showStatus();
  void showMainMenu();
  void showPrepare();
  void showTune();
  void showControl();
  void showControlMotion();
  void showControlTemp();
  void showControlRetract();
  void showAxisMove();
  void showSD();
  void showPLAsettings();
  void showABSsettings();

private:
  //FORCE_INLINE 
  void updateActiveLines(const uint8_t &maxlines, volatile long &encoderpos)
  {
    if (linechanging) {
      return; // an item is changint its value, do not switch lines hence
    }

    long curencoderpos = encoderpos;
    lastlineoffset = lineoffset;
    force_lcd_update = false;
    
    if ((abs(curencoderpos - lastencoderpos) < 1)) { 

      lcd.setCursor(0, activeline * 8);

      //lcd.print((activeline + lineoffset) ? ' ' : ' '); 
      lcd.print(' ');

      if (curencoderpos < 0) {  
        curencoderpos = 0;
        lineoffset--; 
      } 

      if (lineoffset < 0) lineoffset = 0;

      if (curencoderpos > LCD_HEIGHT) { 
        lineoffset++; 

        curencoderpos = (LCD_HEIGHT - 1); 

        if (lineoffset > (maxlines + 1 - LCD_HEIGHT)) lineoffset = maxlines + 1 - LCD_HEIGHT; 

        if (curencoderpos > maxlines) curencoderpos = maxlines; 
      } 

      lastencoderpos = encoderpos = curencoderpos;
      activeline = curencoderpos;
      
      if (activeline < 0) activeline = 0;

      if (activeline > LCD_HEIGHT - 1) activeline = LCD_HEIGHT - 1;

      if (activeline > maxlines) {
        activeline = maxlines;
        curencoderpos = maxlines;
      }

      if (lastlineoffset != lineoffset) {
        force_lcd_update = true;
      }

      lcd.setCursor(0, activeline * 8);
      lcd.print((activeline + lineoffset) ? '>' : 'x' /*\003'*/);    
    } 
  }
  
  FORCE_INLINE void clearIfNecessary()
  {
    if (lastlineoffset != lineoffset || force_lcd_update) {
      force_lcd_update = true;
      lcd.fillScreen(ST7735_BLACK);
    } 
  }
};

//conversion routines, could need some overworking
char *ftostr51(const float &x);
char *ftostr52(const float &x);
char *ftostr31(const float &x);
char *ftostr3(const float &x);

#define LCD_INIT lcd_init();
#define LCD_MESSAGE(x) lcd_status(x);
#define LCD_MESSAGEPGM(x) lcd_statuspgm(PSTR(x));
#define LCD_ALERTMESSAGEPGM(x) lcd_alertstatuspgm(PSTR(x));
#define LCD_STATUS lcd_status()

extern void lcd_statuspgm(const char* message);
extern void lcd_alertstatuspgm(const char* message);
  
extern char *ftostr3(const float &x);
extern char *itostr2(const uint8_t &x);
extern char *ftostr31(const float &x);
extern char *ftostr32(const float &x);
extern char *itostr31(const int &xx);
extern char *itostr3(const int &xx);
extern char *itostr4(const int &xx);
extern char *ftostr51(const float &x);


//TODO: These do not belong here.
extern int plaPreheatHotendTemp;
extern int plaPreheatHPBTemp;
extern int plaPreheatFanSpeed;

extern int absPreheatHotendTemp;
extern int absPreheatHPBTemp;
extern int absPreheatFanSpeed;

#endif //ULTRALCD_H
