#include "language.h"
#include "temperature.h"
#include "ultralcd.h"

#ifdef ULTRA_LCD

#include "Marlin.h"
#include "language.h"
#include "temperature.h"
#include "ConfigurationStore.h"

// TODO: This should be in a preferences file.
int plaPreheatHotendTemp;
int plaPreheatHPBTemp;
int plaPreheatFanSpeed;

int absPreheatHotendTemp;
int absPreheatHPBTemp;
int absPreheatFanSpeed;

#define cs   10
#define dc   9
#define rst  8
Adafruit_ST7735 lcd(cs, dc, rst);

extern volatile int feedmultiply;
extern volatile bool feedmultiplychanged;
extern volatile int extrudemultiply;

extern long position[4];

#ifdef SDSUPPORT
# include "cardreader.h"
  extern CardReader card;
#endif

//volatile char buttons=0;  //the last checked buttons in a bit array.
long encoderpos=0;
short lastenc=0;

static char messagetext[LCD_WIDTH]="";
static char conv[8];
static unsigned long previous_millis_lcd=0;
static unsigned long blocking=0;
static MainMenu menu;

void lcdProgMemprint(const char *str)
{
  char ch = pgm_read_byte(str);
  while (ch) {
    lcd.print(ch);
    ch = pgm_read_byte(++str);
  }
}
#define LCD_PRINT_PGM(x) lcdProgMemprint(PSTR(x))

int intround(const float &x) 
{
  return int(0.5 + x);
}

void lcd_status(const char* message)
{
  strncpy(messagetext,message,LCD_WIDTH);
  messagetext[strlen(message)]=0;
}

void lcd_statuspgm(const char* message)
{
  char ch=pgm_read_byte(message);
  char *target=messagetext;
  uint8_t cnt=0;
  while(ch && cnt < LCD_WIDTH) {
    *target = ch;
    target++;
    cnt++;
    ch = pgm_read_byte(++message);
  }
  *target=0;
}

void lcd_alertstatuspgm(const char* message)
{
  lcd_statuspgm(message);
  menu.showStatus();
}

FORCE_INLINE void setCursor(uint8_t x, uint8_t y)
{
  lcd.setCursor(x * 5, y * 8);
}

FORCE_INLINE void clear()
{
  //lcd.clear();
  lcd.fillScreen(ST7735_BLACK);
}

void lcd_init()
{
  //beep();
  buttons_init();
/*
   byte Degree[8] =
   {
    B01100,
    B10010,
    B10010,
    B01100,
    B00000,
    B00000,
    B00000,
    B00000
   };
   byte Thermometer[8] =
   {
    B00100,
    B01010,
    B01010,
    B01010,
    B01010,
    B10001,
    B10001,
    B01110
   };
   byte uplevel[8]={
    B00100,
    B01110,
    B11111,
    B00100,
    B11100,
    B00000,
    B00000,
    B00000
   }; //thanks joris
   byte refresh[8]={
    B00000,
    B00110,
    B11001,
    B11000,
    B00011,
    B10011,
    B01100,
    B00000,
   }; //thanks joris
   byte folder [8]={
    B00000,
    B11100,
    B11111,
    B10001,
    B10001,
    B11111,
    B00000,
    B00000
   }; //thanks joris
 */

  lcd.initR(INITR_REDTAB);
  lcd.setRotation(3);   // so that sainsmart lcd's mmc socket is on top
  lcd.fillScreen(ST7735_BLACK);
/*
   lcd.begin(LCD_WIDTH, LCD_HEIGHT);
   lcd.createChar(1,Degree);
   lcd.createChar(2,Thermometer);
   lcd.createChar(3,uplevel);
   lcd.createChar(4,refresh);
   lcd.createChar(5,folder);
 */
  LCD_MESSAGEPGM(WELCOME_MSG);
}

void lcd_status()
{ 
/*
  static uint8_t oldbuttons=0;

  unsigned long ms=millis();

  for (int8_t i = 0; i < 8; i++) {
    if (blocking > ms) {
      buttons &= ~(1 << i);
    }
  }

  if ((buttons == oldbuttons) 
    && ((millis() - previous_millis_lcd) < LCD_UPDATE_INTERVAL)) 
  {
    return;
  }

  oldbuttons = buttons;
*/

  if ((millis() - previous_millis_lcd) < LCD_UPDATE_INTERVAL) {
    return;
  }
  previous_millis_lcd = millis();

  menu.update();
}

#define DIR_CCW 0x10
#define DIR_CW  0x20

const uint8_t stateTable[6][4] __attribute__((__progmem__)) = {
  { 0x03, 0x02, 0x01, 0x00 }, { 0x23, 0x00, 0x01, 0x00 },
  { 0x13, 0x02, 0x00, 0x00 }, { 0x03, 0x05, 0x04, 0x00 },
  { 0x03, 0x03, 0x04, 0x10 }, { 0x03, 0x05, 0x03, 0x20}
};

volatile uint8_t state = 0;

uint8_t rotary_process() {
  uint8_t pinstate = (digitalRead(BTN_EN1) << 1) | digitalRead(BTN_EN2);
  state = pgm_read_byte(&stateTable[state & 0xf][pinstate]);
  return (state & 0x30);
}

void buttons_init()
{
  pinMode(BTN_EN1, INPUT);
  digitalWrite(BTN_EN1, HIGH);

  pinMode(BTN_EN2, INPUT);
  digitalWrite(BTN_EN2, HIGH);

  pinMode(BTN_ENC, INPUT);
  digitalWrite(BTN_ENC, HIGH);

#if (SDCARDDETECT > -1)
  pinMode(SDCARDDETECT,INPUT);
  digitalWrite(SDCARDDETECT,HIGH);
#endif
}

volatile State::Button button;

State::Button getButton(void)
{
  State::Button ret = button;
  button = State::Open; // reset
//SERIAL_ECHOPGM("btn: ");
//SERIAL_ECHOLN(itostr2(ret));
  return ret;
}

#define ENC_CLICKTIME 20
#define ENC_DOUBLECLICKTIME 100
#define ENC_HOLDTIME 300

static bool doubleClickEnabled = true;
static uint16_t keyDownTicks = 0;   
static uint8_t doubleClickTicks = 0;

void buttons_check()
{
  uint8_t btn = digitalRead(BTN_ENC);

  if (btn == 0) { // key is down
    keyDownTicks++;
    if (keyDownTicks > ENC_HOLDTIME) {
      button = State::Held;
    }
  }
    
  if (btn == 1) { // key is up
    if (keyDownTicks > ENC_CLICKTIME) {
      if (button == State::Held) {
        button = State::Released;
      }
      else {
        #define ENC_SINGLECLICKONLY 1
        if (doubleClickTicks > ENC_SINGLECLICKONLY) {   // prevent trigger in single click mode
          if (doubleClickTicks < ENC_DOUBLECLICKTIME) {
            button = State::DoubleClicked;
            doubleClickTicks = 0;
          }
        }
        else {
          doubleClickTicks = (doubleClickEnabled) ? ENC_DOUBLECLICKTIME : ENC_SINGLECLICKONLY;
        }
      }
    }

    keyDownTicks = 0;
  }

  if (doubleClickTicks > 0) {
    /* why this additional decrement? */ // doubleClickTicks--;
    if (--doubleClickTicks == 0) {
      button = State::Clicked;
    }
  }

  uint8_t enc = rotary_process();
  if (enc) {
    (enc == DIR_CW) ? encoderpos++ : encoderpos--;
  }
}


MainMenu::MainMenu()
{
  status = Main_Status;
  displayStartingRow = 0;
  activeline = 0;
  force_lcd_update = true;
  linechanging = false;
  tune = false;
}

void MainMenu::showStatus()
{
  static int olddegHotEnd0 = -1;
  static int oldtargetHotEnd0 = -1;

  force_lcd_update = true;
  if (force_lcd_update) {
    encoderpos = feedmultiply;
    //clear();

    // FIXME
    /*
       setCursor(0,0); LCD_PRINT_PGM("\002000/000\001 ");
       #if defined BED_USES_THERMISTOR || defined BED_USES_AD595
       setCursor(10,0); LCD_PRINT_PGM("B000/000\001 ");
       #elif EXTRUDERS > 1
       setCursor(10,0); LCD_PRINT_PGM("\002000/000\001 ");
       #endif
    */
  }

  int tHotEnd0=intround(degHotend0());
  if ((tHotEnd0 != olddegHotEnd0) || force_lcd_update) {
    setCursor(1, 0);
    lcd.print(ftostr3(tHotEnd0));
    olddegHotEnd0=tHotEnd0;
  }

  int ttHotEnd0 = intround(degTargetHotend0());
  if((ttHotEnd0 != oldtargetHotEnd0) || force_lcd_update) {
    setCursor(5,0);
    lcd.print(ftostr3(ttHotEnd0));
    oldtargetHotEnd0 = ttHotEnd0;
  }

#if defined BED_USES_THERMISTOR || defined BED_USES_AD595
  static int oldtBed = -1;
  static int oldtargetBed = -1;

  int tBed = intround(degBed());
  if ((tBed != oldtBed) || force_lcd_update) {
    setCursor(11,0);
    lcd.print(ftostr3(tBed));
    oldtBed=tBed;
  }

  int targetBed = intround(degTargetBed());
  if ((targetBed != oldtargetBed) || force_lcd_update) {
    setCursor(15, 0);
    lcd.print(ftostr3(targetBed));
    oldtargetBed=targetBed;
  }
#elif EXTRUDERS > 1
  static int olddegHotEnd1=-1;
  static int oldtargetHotEnd1=-1;

  int tHotEnd1 = intround(degHotend1());
  if ((tHotEnd1 != olddegHotEnd1) || force_lcd_update) {
    setCursor(11,0);
    lcd.print(ftostr3(tHotEnd1));
    olddegHotEnd1=tHotEnd1;
  }

  int ttHotEnd1=intround(degTargetHotend1());
  if ((ttHotEnd1 != oldtargetHotEnd1) || force_lcd_update) {
    setCursor(15, 0);
    lcd.print(ftostr3(ttHotEnd1));
    oldtargetHotEnd1 = ttHotEnd1;
  }
#endif

  static uint16_t oldtime=0;
  if(starttime!=0) {
    setCursor(0,1);
    uint16_t time = millis() / 60000 - starttime / 60000;

    if(starttime != oldtime) {
      lcd.print(itostr2(time/60)); LCD_PRINT_PGM("h "); lcd.print(itostr2(time%60)); LCD_PRINT_PGM("m");
      oldtime=time;
    }
  }

  static int oldzpos=0;
  int currentz=current_position[2]*100;
  if((currentz!=oldzpos)||force_lcd_update)
  {
    setCursor(10,1);
    LCD_PRINT_PGM("Z:"); lcd.print(ftostr52(current_position[2]));
    oldzpos=currentz;
  }

  static int oldfeedmultiply=0;
  int curfeedmultiply=feedmultiply;

  if(feedmultiplychanged == true) {
    feedmultiplychanged = false;
    encoderpos = curfeedmultiply;
  }

  if(encoderpos!=curfeedmultiply||force_lcd_update)
  {
    curfeedmultiply=encoderpos;
    if(curfeedmultiply<10)
      curfeedmultiply=10;
    if(curfeedmultiply>999)
      curfeedmultiply=999;
    feedmultiply=curfeedmultiply;
    encoderpos=curfeedmultiply;
  }

  if((curfeedmultiply!=oldfeedmultiply)||force_lcd_update)
  {
    oldfeedmultiply=curfeedmultiply;

    lcd.fillRect(    0, 2 * 8, 
                 4 * 5, 1 * 8, ST7735_BLACK);

    setCursor(0, 2);
    lcd.print(itostr3(curfeedmultiply)); LCD_PRINT_PGM("% ");
  }

  if(messagetext[0]!='\0')
  {
    setCursor(0,LCD_HEIGHT-1);
    lcd.print(messagetext);
    uint8_t n=strlen(messagetext);
    for(int8_t i=0; i<LCD_WIDTH-n; i++) {
      lcd.print(" ");
    }
    messagetext[0]='\0';
  }

  #ifdef SDSUPPORT
  static uint8_t oldpercent=101;
  uint8_t percent=card.percentDone();
  if(oldpercent!=percent ||force_lcd_update)
  {
    setCursor(10,2);
    lcd.print(itostr3((int)percent));
    LCD_PRINT_PGM("%SD");
  }
  #endif

  force_lcd_update=false;
}

enum {ItemP_exit, ItemP_autostart,ItemP_disstep,ItemP_home, ItemP_origin, ItemP_preheat_pla, ItemP_preheat_abs, ItemP_cooldown,/*ItemP_extrude,*/ ItemP_move};

//any action must not contain a ',' character anywhere, or this breaks:
#define MENUITEM(repaint_action, click_action) \
{ \
  if (force_lcd_update)  { setCursor(0, line);  repaint_action; } \
  if ((activeline == line) && CLICKED) { click_action } \
}

void MainMenu::showPrepare()
{
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemP_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_MAIN),   status=Main_Menu; beepshort(); );
      break;

    case ItemP_autostart:
      MENUITEM(  LCD_PRINT_PGM(MSG_AUTOSTART),  
                 #ifdef SDSUPPORT
                 card.lastnr=0; card.setroot(); card.checkautostart(true);
                 #endif
                 beepshort(); );
      break;

    case ItemP_disstep:
      MENUITEM(  LCD_PRINT_PGM(MSG_DISABLE_STEPPERS),   enquecommand("M84"); beepshort(); );
      break;

    case ItemP_home:
      MENUITEM(  LCD_PRINT_PGM(MSG_AUTO_HOME),   enquecommand("G28"); beepshort(); );
      break;

    case ItemP_origin:
      MENUITEM(  LCD_PRINT_PGM(MSG_SET_ORIGIN),   enquecommand("G92 X0 Y0 Z0"); beepshort(); );
      break;

    case ItemP_preheat_pla:
      MENUITEM(  LCD_PRINT_PGM(MSG_PREHEAT_PLA),   setTargetHotend0(plaPreheatHotendTemp); setTargetBed(plaPreheatHPBTemp);
                 #if FAN_PIN > -1
                 FanSpeed = plaPreheatFanSpeed;
                 analogWrite(FAN_PIN,  FanSpeed);
                 #endif
                 beepshort(); );
      break;

    case ItemP_preheat_abs:
      MENUITEM(  LCD_PRINT_PGM(MSG_PREHEAT_ABS),   setTargetHotend0(absPreheatHotendTemp); setTargetBed(absPreheatHPBTemp);
                 #if FAN_PIN > -1
                 FanSpeed = absPreheatFanSpeed;
                 analogWrite(FAN_PIN,  FanSpeed);
                 #endif
                 beepshort(); );
      break;

    case ItemP_cooldown:
      MENUITEM(  LCD_PRINT_PGM(MSG_COOLDOWN),   setTargetHotend0(0); setTargetHotend1(0); setTargetHotend2(0); setTargetBed(0); beepshort(); );
      break;

//    case ItemP_extrude:
    //    MENUITEM(  LCD_PRINT_PGM(" Extrude")  ,  enquecommand("G92 E0");enquecommand("G1 F700 E50");beepshort(); ) ;
    //  break;

    case ItemP_move:
      MENUITEM(  LCD_PRINT_PGM(MSG_MOVE_AXIS),  status=Sub_PrepareMove; beepshort(); );
      break;

    default:
      break;
    }

    line++;
  }
  updateActiveLines(ItemP_move,encoderpos);
}

enum {
  ItemAM_exit,
  ItemAM_X, ItemAM_Y, ItemAM_Z, ItemAM_E, ItemAM_ERetract
};

void MainMenu::showAxisMove()
{
  uint8_t line=0;
  int oldencoderpos=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemAM_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_PREPARE_ALT),   status=Main_Prepare; beepshort(); );
      break;

    case ItemAM_X: {
      if(force_lcd_update) {
        setCursor(0,line); LCD_PRINT_PGM(" X:");
        setCursor(11,line); lcd.print(ftostr52(current_position[X_AXIS]));
      }

      if((activeline!=line))
        break;

      if (CLICKED) {
        linechanging=!linechanging;
        if(linechanging) {
          enquecommand("G91");
        }
        else {
          enquecommand("G90");
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if (linechanging) {
        if (encoderpos > 0) {
          enquecommand("G1 F700 X0.1");
          oldencoderpos=encoderpos;
          encoderpos=0;
        }
        else if (encoderpos < 0) {
          enquecommand("G1 F700 X-0.1");
          oldencoderpos=encoderpos;
          encoderpos=0;
        }
        setCursor(11,line); lcd.print(ftostr52(current_position[X_AXIS]));
      }
    }
    break;

    case ItemAM_Y: {
      if (force_lcd_update) {
        setCursor(0,line); LCD_PRINT_PGM(" Y:");
        setCursor(11,line); lcd.print(ftostr52(current_position[Y_AXIS]));
      }

      if ((activeline!=line))
        break;

      if (CLICKED) {
        linechanging = !linechanging;
        if(linechanging) {
          enquecommand("G91");
        }
        else {
          enquecommand("G90");
          encoderpos=activeline;
          beepshort();
        }
        
      }

      if (linechanging) {
        if (encoderpos > 0) {
          enquecommand("G1 F700 Y0.1");
          oldencoderpos = encoderpos;
          encoderpos = 0;
        }

        else if (encoderpos < 0) {
          enquecommand("G1 F700 Y-0.1");
          oldencoderpos=encoderpos;
          encoderpos=0;
        }
        setCursor(11,line); lcd.print(ftostr52(current_position[Y_AXIS]));
      }
    }
    break;

    case ItemAM_Z: {
      if (force_lcd_update) {
        setCursor(0,line); LCD_PRINT_PGM(" Z:");
        setCursor(11,line); lcd.print(ftostr52(current_position[Z_AXIS]));
      }

      if (activeline!=line)
        break;

      if (CLICKED) {
        linechanging=!linechanging;
        if(linechanging) {
          enquecommand("G91");
        }
        else {
          enquecommand("G90");
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging) {
        if (encoderpos > 0) {
          enquecommand("G1 F70 Z0.1");
          oldencoderpos=encoderpos;
          encoderpos=0;
        }
        else if (encoderpos < 0) {
          enquecommand("G1 F70 Z-0.1");
          oldencoderpos=encoderpos;
          encoderpos=0;
        }
        setCursor(11,line); lcd.print(ftostr52(current_position[Z_AXIS]));
      }
    }
    break;

    case ItemAM_E:
      // ErikDB: TODO: this length should be changed for volumetric.
      MENUITEM(  LCD_PRINT_PGM(MSG_EXTRUDE),   enquecommand("G92 E0"); enquecommand("G1 F70 E1"); beepshort(); );
      break;
    case ItemAM_ERetract:
      // ErikDB: TODO: this length should be changed for volumetric.
      MENUITEM(  LCD_PRINT_PGM(MSG_RETRACT),   enquecommand("G92 E0"); enquecommand("G1 F700 E-1"); beepshort(); );
      break;
    
    default:
      break;
    }
    
    line++;
  }
  
  updateActiveLines(ItemAM_ERetract,encoderpos);
}

enum {ItemT_exit,ItemT_speed,ItemT_flow,ItemT_nozzle,
      #if (HEATER_BED_PIN > -1)
      ItemT_bed,
      #endif
      ItemT_fan};

void MainMenu::showTune()
{
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    //Serial.println((int)(line-lineoffset));
    switch(i)
    {
    case ItemT_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_MAIN),   status=Main_Menu; beepshort(); );
      break;
    case ItemT_speed:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_SPEED);
        setCursor(13,line); lcd.print(ftostr3(feedmultiply));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)       //AnalogWrite(FAN_PIN,  fanpwm);
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=feedmultiply;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>400) encoderpos=400;
        feedmultiply = encoderpos;
        feedmultiplychanged=true;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
    case ItemT_nozzle:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetHotend0())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetHotend0());
        }
        else
        {
          setTargetHotend0(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;
      #if (HEATER_BED_PIN > -1)
    case ItemT_bed:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_BED);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetBed())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetBed());
        }
        else
        {
          setTargetBed(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;
      #endif

    case ItemT_fan:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FAN_SPEED);
        setCursor(13,line); lcd.print(ftostr3(FanSpeed));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)       //nalogWrite(FAN_PIN,  fanpwm);
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=FanSpeed;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>255) encoderpos=255;
        FanSpeed=encoderpos;
        analogWrite(FAN_PIN,  FanSpeed);
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
    case ItemT_flow:     //axis_steps_per_unit[i] = code_value();
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FLOW);
        setCursor(13,line); lcd.print(ftostr52(axis_steps_per_unit[E_AXIS]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(axis_steps_per_unit[E_AXIS]*100.0);
        }
        else
        {
          float factor=float(encoderpos)/100.0/float(axis_steps_per_unit[E_AXIS]);
          position[E_AXIS]=lround(position[E_AXIS]*factor);
          //current_position[E_AXIS]*=factor;
          axis_steps_per_unit[E_AXIS]= encoderpos/100.0;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>999999) encoderpos=999999;
        setCursor(13,line); lcd.print(ftostr52(encoderpos/100.0));
      }

    } break;
    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemT_fan,encoderpos);
}

/*does not work
   #define MENUCHANGEITEM(repaint_action,  enter_action, accept_action,  change_action) \
   {\
     if(force_lcd_update)  { setCursor(0,line);  repaint_action; } \
     if(activeline==line)  \
     { \
       if(CLICKED) \
       { \
         linechanging=!linechanging; \
         if(linechanging)  {enter_action;} \
         else {accept_action;} \
       }  \
       else \
       if(linechanging) {change_action};}\
   }
 */

enum {
  ItemCT_exit,ItemCT_nozzle0,
  #ifdef AUTOTEMP
  ItemCT_autotempactive,
  ItemCT_autotempmin,ItemCT_autotempmax,ItemCT_autotempfact,
  #endif
  #if EXTRUDERS > 1
  ItemCT_nozzle1,
  #endif
  #if EXTRUDERS > 2
  ItemCT_nozzle2,
  #endif
  #if defined BED_USES_THERMISTOR || defined BED_USES_AD595
  ItemCT_bed,
  #endif
  ItemCT_fan,
  ItemCT_PID_P,ItemCT_PID_I,ItemCT_PID_D,ItemCT_PID_C,
  ItemCT_PLA_PreHeat_Setting,
  ItemCT_ABS_PreHeat_Setting,
};

void MainMenu::showControlTemp()
{
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemCT_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_CONTROL),   status=Main_Control; beepshort(); );
      break;
    case ItemCT_nozzle0:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetHotend0())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetHotend0());
        }
        else
        {
          setTargetHotend0(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
      #if EXTRUDERS > 1
    case ItemCT_nozzle1:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE1);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetHotend1())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetHotend1());
        }
        else
        {
          setTargetHotend1(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
      #endif
      #if EXTRUDERS > 2
    case ItemCT_nozzle2:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE2);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetHotend2())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetHotend2());
        }
        else
        {
          setTargetHotend1(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
      #endif
      #ifdef AUTOTEMP
    case ItemCT_autotempmin:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_MIN);
        setCursor(13,line); lcd.print(ftostr3(autotemp_min));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(autotemp_min);
        }
        else
        {
          autotemp_min=encoderpos;
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
    case ItemCT_autotempmax:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_MAX);
        setCursor(13,line); lcd.print(ftostr3(autotemp_max));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(autotemp_max);
        }
        else
        {
          autotemp_max=encoderpos;
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
    case ItemCT_autotempfact:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FACTOR);
        setCursor(13,line); lcd.print(ftostr32(autotemp_factor));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(autotemp_factor*100);
        }
        else
        {
          autotemp_max=encoderpos;
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>99) encoderpos=99;
        setCursor(13,line); lcd.print(ftostr32(encoderpos/100.));
      }

    } break;
    case ItemCT_autotempactive:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_AUTOTEMP);
        setCursor(13,line);
        if(autotemp_enabled)
          LCD_PRINT_PGM(MSG_ON);
        else
          LCD_PRINT_PGM(MSG_OFF);
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        autotemp_enabled=!autotemp_enabled;
        setCursor(13,line);
        if(autotemp_enabled)
          LCD_PRINT_PGM(MSG_ON);
        else
          LCD_PRINT_PGM(MSG_OFF);
        
      }

    } break;
      #endif //autotemp
      #if defined BED_USES_THERMISTOR || defined BED_USES_AD595
    case ItemCT_bed:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_BED);
        setCursor(13,line); lcd.print(ftostr3(intround(degTargetBed())));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=intround(degTargetBed());
        }
        else
        {
          setTargetBed(encoderpos);
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;
      #endif
    case ItemCT_fan:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FAN_SPEED);
        setCursor(13,line); lcd.print(ftostr3(FanSpeed));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)       //nalogWrite(FAN_PIN,  fanpwm);
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=FanSpeed;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>255) encoderpos=255;
        FanSpeed=encoderpos;
        analogWrite(FAN_PIN,  FanSpeed);
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
      #ifdef PIDTEMP
    case ItemCT_PID_P:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(" PID-P: ");
        setCursor(13,line); lcd.print(itostr4(Kp));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)Kp;
        }
        else
        {
          Kp= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>9990) encoderpos=9990;
        setCursor(13,line); lcd.print(itostr4(encoderpos));
      }

    } break;
    case ItemCT_PID_I:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_PID_I);
        setCursor(13,line); lcd.print(ftostr51(Ki/PID_dT));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(Ki*10/PID_dT);
        }
        else
        {
          Ki= encoderpos/10.*PID_dT;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>9990) encoderpos=9990;
        setCursor(13,line); lcd.print(ftostr51(encoderpos/10.));
      }

    } break;
    case ItemCT_PID_D:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_PID_D);
        setCursor(13,line); lcd.print(itostr4(Kd*PID_dT));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(Kd/5./PID_dT);
        }
        else
        {
          Kd= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>9990) encoderpos=9990;
        setCursor(13,line); lcd.print(itostr4(encoderpos));
      }

    } break;
    case ItemCT_PID_C:
      #ifdef PID_ADD_EXTRUSION_RATE
      {
        if(force_lcd_update)
        {
          setCursor(0,line); LCD_PRINT_PGM(MSG_PID_C);
          setCursor(13,line); lcd.print(itostr3(Kc));
        }

        if((activeline!=line) )
          break;

        if(CLICKED)
        {
          linechanging=!linechanging;
          if(linechanging)
          {
            encoderpos=(long)Kc;
          }
          else
          {
            Kc= encoderpos;
            encoderpos=activeline;

          }
          
          beepshort();
        }
        if(linechanging)
        {
          if(encoderpos<0) encoderpos=0;
          if(encoderpos>990) encoderpos=990;
          setCursor(13,line); lcd.print(itostr3(encoderpos));
        }

      }
      #endif
      #endif
      break;
    case ItemCT_PLA_PreHeat_Setting:
      MENUITEM(  LCD_PRINT_PGM(MSG_PREHEAT_PLA_SETTINGS),   status=Sub_PreheatPLASettings; beepshort(); );
      break;
    case ItemCT_ABS_PreHeat_Setting:
      MENUITEM(  LCD_PRINT_PGM(MSG_PREHEAT_ABS_SETTINGS),   status=Sub_PreheatABSSettings; beepshort(); );
      break;
    default:
      break;
    }
    line++;
  }

  updateActiveLines(ItemCT_ABS_PreHeat_Setting,encoderpos);
}

enum {
  ItemCM_exit,
  ItemCM_acc, ItemCM_xyjerk,
  ItemCM_vmaxx, ItemCM_vmaxy, ItemCM_vmaxz, ItemCM_vmaxe,
  ItemCM_vtravmin,ItemCM_vmin,
  ItemCM_amaxx, ItemCM_amaxy, ItemCM_amaxz, ItemCM_amaxe,
  ItemCM_aret, ItemCM_xsteps,ItemCM_ysteps, ItemCM_zsteps, ItemCM_esteps
};

void MainMenu::showControlMotion()
{
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemCM_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_CONTROL),   status=Main_Control; beepshort(); );
      break;
    case ItemCM_acc:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_ACC);
        setCursor(13,line); lcd.print(itostr3(acceleration/100)); LCD_PRINT_PGM("00");
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)acceleration/100;
        }
        else
        {
          acceleration= encoderpos*100;
          encoderpos=activeline;
        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos)); LCD_PRINT_PGM("00");
      }

    } break;
    case ItemCM_xyjerk:     //max_xy_jerk
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_VXY_JERK);
        setCursor(13,line); lcd.print(itostr3(max_xy_jerk));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)max_xy_jerk;
        }
        else
        {
          max_xy_jerk= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;

    case ItemCM_vmaxx:
    case ItemCM_vmaxy:
    case ItemCM_vmaxz:
    case ItemCM_vmaxe:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_VMAX);
        if(i==ItemCM_vmaxx) LCD_PRINT_PGM(MSG_X);
        if(i==ItemCM_vmaxy) LCD_PRINT_PGM(MSG_Y);
        if(i==ItemCM_vmaxz) LCD_PRINT_PGM(MSG_Z);
        if(i==ItemCM_vmaxe) LCD_PRINT_PGM(MSG_E);
        setCursor(13,line); lcd.print(itostr3(max_feedrate[i-ItemCM_vmaxx]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)max_feedrate[i-ItemCM_vmaxx];
        }
        else
        {
          max_feedrate[i-ItemCM_vmaxx]= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;

    case ItemCM_vmin:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_VMIN);
        setCursor(13,line); lcd.print(itostr3(minimumfeedrate));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(minimumfeedrate);
        }
        else
        {
          minimumfeedrate= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;
    case ItemCM_vtravmin:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_VTRAV_MIN);
        setCursor(13,line); lcd.print(itostr3(mintravelfeedrate));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)mintravelfeedrate;
        }
        else
        {
          mintravelfeedrate= encoderpos;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }

    } break;

    case ItemCM_amaxx:
    case ItemCM_amaxy:
    case ItemCM_amaxz:
    case ItemCM_amaxe:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(" Amax ");
        if(i==ItemCM_amaxx) LCD_PRINT_PGM(MSG_X);
        if(i==ItemCM_amaxy) LCD_PRINT_PGM(MSG_Y);
        if(i==ItemCM_amaxz) LCD_PRINT_PGM(MSG_Z);
        if(i==ItemCM_amaxe) LCD_PRINT_PGM(MSG_E);
        setCursor(13,line); lcd.print(itostr3(max_acceleration_units_per_sq_second[i-ItemCM_amaxx]/100)); LCD_PRINT_PGM("00");
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)max_acceleration_units_per_sq_second[i-ItemCM_amaxx]/100;
        }
        else
        {
          max_acceleration_units_per_sq_second[i-ItemCM_amaxx]= encoderpos*100;
          encoderpos=activeline;
        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos)); LCD_PRINT_PGM("00");
      }

    } break;

    case ItemCM_aret:     //float retract_acceleration = 7000;
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_A_RETRACT);
        setCursor(13,line); lcd.print(ftostr3(retract_acceleration/100)); LCD_PRINT_PGM("00");
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)retract_acceleration/100;
        }
        else
        {
          retract_acceleration= encoderpos*100;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<10) encoderpos=10;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr3(encoderpos)); LCD_PRINT_PGM("00");
      }

    } break;
    case ItemCM_xsteps:     //axis_steps_per_unit[i] = code_value();
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_XSTEPS);
        setCursor(11,line); lcd.print(ftostr52(axis_steps_per_unit[X_AXIS]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(axis_steps_per_unit[X_AXIS]*100.0);
        }
        else
        {
          float factor=float(encoderpos)/100.0/float(axis_steps_per_unit[X_AXIS]);
          position[X_AXIS]=lround(position[X_AXIS]*factor);
          //current_position[X_AXIS]*=factor;
          axis_steps_per_unit[X_AXIS]= encoderpos/100.0;
          encoderpos=activeline;
        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>999999) encoderpos=999999;
        setCursor(11,line); lcd.print(ftostr52(encoderpos/100.0));
      }

    } break;
    case ItemCM_ysteps:     //axis_steps_per_unit[i] = code_value();
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_YSTEPS);
        setCursor(11,line); lcd.print(ftostr52(axis_steps_per_unit[Y_AXIS]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(axis_steps_per_unit[Y_AXIS]*100.0);
        }
        else
        {
          float factor=float(encoderpos)/100.0/float(axis_steps_per_unit[Y_AXIS]);
          position[Y_AXIS]=lround(position[Y_AXIS]*factor);
          //current_position[Y_AXIS]*=factor;
          axis_steps_per_unit[Y_AXIS]= encoderpos/100.0;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>999999) encoderpos=999999;
        setCursor(11,line); lcd.print(ftostr52(encoderpos/100.0));
      }

    } break;
    case ItemCM_zsteps:     //axis_steps_per_unit[i] = code_value();
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_ZSTEPS);
        setCursor(11,line); lcd.print(ftostr51(axis_steps_per_unit[Z_AXIS]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(axis_steps_per_unit[Z_AXIS]*100.0);
        }
        else
        {
          float factor=float(encoderpos)/100.0/float(axis_steps_per_unit[Z_AXIS]);
          position[Z_AXIS]=lround(position[Z_AXIS]*factor);
          //current_position[Z_AXIS]*=factor;
          axis_steps_per_unit[Z_AXIS]= encoderpos/100.0;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>999999) encoderpos=999999;
        setCursor(11,line); lcd.print(ftostr52(encoderpos/100.0));
      }

    } break;

    case ItemCM_esteps:     //axis_steps_per_unit[i] = code_value();
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_ESTEPS);
        setCursor(11,line); lcd.print(ftostr51(axis_steps_per_unit[E_AXIS]));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(axis_steps_per_unit[E_AXIS]*100.0);
        }
        else
        {
          float factor=float(encoderpos)/100.0/float(axis_steps_per_unit[E_AXIS]);
          position[E_AXIS]=lround(position[E_AXIS]*factor);
          //current_position[E_AXIS]*=factor;
          axis_steps_per_unit[E_AXIS]= encoderpos/100.0;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<5) encoderpos=5;
        if(encoderpos>999999) encoderpos=999999;
        setCursor(11,line); lcd.print(ftostr52(encoderpos/100.0));
      }

    } break;
    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemCM_esteps,encoderpos);
}

enum {
  ItemR_exit,
  ItemR_autoretract,
  ItemR_retract_length,ItemR_retract_feedrate,ItemR_retract_zlift,
  ItemR_unretract_length,ItemR_unretract_feedrate,

};

void MainMenu::showControlRetract()
{
  #ifdef FWRETRACT
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemR_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_CONTROL),   status=Main_Control; beepshort(); );
      break;

    //float retract_length=2, retract_feedrate=1200, retract_zlift=0.4;
    //float retract_recover_length=0, retract_recover_feedrate=500;
    case ItemR_autoretract:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_AUTORETRACT);
        setCursor(13,line);
        if(autoretract_enabled)
          LCD_PRINT_PGM(MSG_ON);
        else
          LCD_PRINT_PGM(MSG_OFF);
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        autoretract_enabled=!autoretract_enabled;
        setCursor(13,line);
        if(autoretract_enabled)
          LCD_PRINT_PGM(MSG_ON);
        else
          LCD_PRINT_PGM(MSG_OFF);
        
      }

    } break;

    case ItemR_retract_length:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_CONTROL_RETRACT);
        setCursor(13,line); lcd.print(ftostr52(retract_length));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(retract_length*100);
        }
        else
        {
          retract_length= encoderpos/100.;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(ftostr52(encoderpos/100.));
      }

    } break;
    case ItemR_retract_feedrate:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_CONTROL_RETRACTF);
        setCursor(13,line); lcd.print(itostr4(retract_feedrate));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(retract_feedrate/5);
        }
        else
        {
          retract_feedrate= encoderpos*5.;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr4(encoderpos*5));
      }

    } break;
    case ItemR_retract_zlift:     //float retract_acceleration = 7000;
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_CONTROL_RETRACT_ZLIFT);
        setCursor(13,line); lcd.print(ftostr52(retract_zlift));;
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(retract_zlift*10);
        }
        else
        {
          retract_zlift= encoderpos/10.;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(ftostr52(encoderpos/10.));
      }

    } break;
    case ItemR_unretract_length:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_CONTROL_RETRACT_RECOVER);
        setCursor(13,line); lcd.print(ftostr52(retract_recover_length));;
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)(retract_recover_length*100);
        }
        else
        {
          retract_recover_length= encoderpos/100.;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(ftostr52(encoderpos/100.));
      }

    } break;

    case ItemR_unretract_feedrate:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_CONTROL_RETRACT_RECOVERF);
        setCursor(13,line); lcd.print(itostr4(retract_recover_feedrate));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=(long)retract_recover_feedrate/5;
        }
        else
        {
          retract_recover_feedrate= encoderpos*5.;
          encoderpos=activeline;

        }
        
        beepshort();
      }
      if(linechanging)
      {
        if(encoderpos<1) encoderpos=1;
        if(encoderpos>990) encoderpos=990;
        setCursor(13,line); lcd.print(itostr4(encoderpos*5));
      }

    } break;

    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemR_unretract_feedrate,encoderpos);
  #endif
}

enum {
  ItemC_exit,ItemC_temp,ItemC_move,
  #ifdef FWRETRACT
  ItemC_rectract,
  #endif
  ItemC_store, ItemC_load,ItemC_failsafe
};

void MainMenu::showControl()
{
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case ItemC_exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_MAIN_WIDE),   status=Main_Menu; beepshort(); );
      break;
    case ItemC_temp:
      MENUITEM(  LCD_PRINT_PGM(MSG_TEMPERATURE_WIDE),   status=Sub_TempControl; beepshort(); );
      break;
    case ItemC_move:
      MENUITEM(  LCD_PRINT_PGM(MSG_MOTION_WIDE),   status=Sub_MotionControl; beepshort(); );
      break;
      #ifdef FWRETRACT
    case ItemC_rectract:
      MENUITEM(  LCD_PRINT_PGM(MSG_RECTRACT_WIDE),   status=Sub_RetractControl; beepshort(); );
      break;
      #endif
    case ItemC_store:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_STORE_EPROM);
      }
      if((activeline==line) && CLICKED)
      {
        //enquecommand("M84");
        beepshort();
        
        Config_StoreSettings();
      }
    } break;
    case ItemC_load:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_LOAD_EPROM);
      }
      if((activeline==line) && CLICKED)
      {
        //enquecommand("M84");
        beepshort();
        
        Config_RetrieveSettings();
      }
    } break;
    case ItemC_failsafe:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_RESTORE_FAILSAFE);
      }
      if((activeline==line) && CLICKED)
      {
        //enquecommand("M84");
        beepshort();
        
        Config_ResetDefault();
      }
    } break;
    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemC_failsafe,encoderpos);
}

void MainMenu::showSD()
{
  #ifdef SDSUPPORT
  uint8_t line=0;

  clearIfNecessary();
  static uint8_t nrfiles=0;
  if(force_lcd_update)
  {
    if(card.cardOK)
    {
      nrfiles=card.getnrfilenames();
    }
    else
    {
      nrfiles=0;
      lineoffset=0;
    }
  }
  bool enforceupdate=false;
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {
    case 0:
      MENUITEM(  LCD_PRINT_PGM(MSG_MAIN),   status=Main_Menu; beepshort(); );
      break;
//     case 1:
//       {
//         if(force_lcd_update)
//         {
//           setCursor(0,line);
//           if(CARDINSERTED)
//           {
//             LCD_PRINT_PGM(" \004Refresh");
//           }
//           else
//           {
//             LCD_PRINT_PGM(" \004Insert Card");
//           }
//
//         }
//         if((activeline==line) && CLICKED)
//         {
//           
//           beepshort();
//           card.initsd();
//           force_lcd_update=true;
//            nrfiles=card.getnrfilenames();
//         }
//       }break;
    case 1:
      MENUITEM(  lcd.print(" "); card.getWorkDirName();
                 if(card.filename[0]=='/') LCD_PRINT_PGM(MSG_REFRESH);
                 else {
                   lcd.print("\005");
                   lcd.print(card.filename);
                   lcd.print("/..");
                 },
                 
                 if(SDCARDDETECT == -1) card.initsd();
                 card.updir();
                 enforceupdate=true;
                 lineoffset=0;
                 beepshort(); );

      break;
    default:
    {
  #define FIRSTITEM 2
      if(i-FIRSTITEM<nrfiles)
      {
        if(force_lcd_update)
        {
          card.getfilename(i-FIRSTITEM);
          //Serial.print("Filenr:");Serial.println(i-2);
          setCursor(0,line); LCD_PRINT_PGM(" ");
          if(card.filenameIsDir)
          {
            lcd.print("\005");
            card.longFilename[LCD_WIDTH-2] = '\0';
          }
          if (card.longFilename[0])
          {
            card.longFilename[LCD_WIDTH-1] = '\0';
            lcd.print(card.longFilename);
          }
          else
          {
            lcd.print(card.filename);
          }
        }
        if((activeline==line) && CLICKED)
        {
          BLOCK
          card.getfilename(i-FIRSTITEM);
          if(card.filenameIsDir)
          {
            for(uint8_t i=0; i<strlen(card.filename); i++) {
              card.filename[i]=tolower(card.filename[i]);
            }
            card.chdir(card.filename);
            lineoffset=0;
            enforceupdate=true;
          }
          else
          {
            char cmd[30];
            for(uint8_t i=0; i<strlen(card.filename); i++) {
              card.filename[i]=tolower(card.filename[i]);
            }
            sprintf(cmd,"M23 %s",card.filename);
            //sprintf(cmd,"M115");
            enquecommand(cmd);
            enquecommand("M24");
            beep();
            status=Main_Status;
            if (card.longFilename[0])
            {
              card.longFilename[LCD_WIDTH-1] = '\0';
              lcd_status(card.longFilename);
            }
            else
            {
              lcd_status(card.filename);
            }
          }
        }
      }

    }
    break;
    }
    line++;
  }
  updateActiveLines(FIRSTITEM+nrfiles-1,encoderpos);
  if(enforceupdate)
  {
    force_lcd_update=true;
    enforceupdate=false;
  }
  #endif
}

enum {
  ItemM_watch,
  ItemM_prepare,
  ItemM_control,
  ItemM_file,
  ItemM_pause
};

///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
// New Menu
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////

// ----------------------------------------------------------------------------- 
//
class Point {
public:
  uint8_t x;
  uint8_t y;
  Point(uint8_t _x, uint8_t _y)
    : x(_x), y(_y)
  {}
  Point() : x(0), y(0) {}
  Point& operator= (const Point &p);
};

Point& Point::operator= (const Point &p) {
  x = p.x;
  y = p.y;
  return *this;
}

class Size {
public:
  uint8_t w;
  uint8_t h;
  Size(uint8_t _w, uint8_t _h)
    : w(_w), h(_h) 
  {}
};

class Rect {
public:
  Point origin;
  Size size;
  Rect(uint8_t x, uint8_t y, uint8_t w, uint8_t h) 
    : origin(x, y), size(w, h) 
  {}
};

Rect Screen(0, 0, 160, 128);
Point Cursor;

namespace State {  
  typedef enum SystemMode_e {
    None        = 0,
    StatusScreen  = (1<<0),
    MainMenu    = (1<<1),
    //Edit        = (1<<2),
    //Idle   = (1<<4)
  } PanelMode;

  typedef enum EditMode_e {
    EditModeNone   = 0,
    EditModeLayout = (1<<0),
    EditModeUnit   = (1<<1),
    EditModeScale  = (1<<2)
  } EditMode; 
};

uint8_t panelState = State::StatusScreen;
uint8_t editState = State::EditModeNone;

/*
 ST7735_BLACK
 ST7735_BLUE
 ST7735_RED
 ST7735_GREEN
 ST7735_CYAN
 ST7735_MAGENTA
 ST7735_YELLOW  
 ST7735_WHITE
*/

struct MenuItemRenderPosition_s {
  const MenuItem_t *item;
  Point pos;
};

const uint8_t maxDisplayedMenuItems = 5;
struct MenuItemRenderPosition_s LastMenuRendered[maxDisplayedMenuItems];

uint8_t menuNOP(menuAction_t action) {
  return TRUE;
}

void renderMenuItem(const MenuItem_t * i, uint8_t position) 
{
  const uint8_t menuLeftMargin = 3;

  if (menuCurrentItem == menuPreviousItem) return;

  if (i == menuCurrentItem) { 
    lcd.setTextColor(ST7735_BLACK, ST7735_WHITE);
  }
  else {
    lcd.setTextColor(ST7735_WHITE, ST7735_BLACK);
  }

  Cursor.y = menuLeftMargin;
  Cursor.x = position * (/*font.height*/ 5 + 2);

  // retain where the menu has been rendered to  
  LastMenuRendered[position].item = i;
  LastMenuRendered[position].pos = Cursor;

  lcd.setCursor(Cursor.x, Cursor.y);

  // lcd.print(menuLeftMargin, lcdCursor.line, menuLabel(i));
  printf(" %c%s%c ", (i == menuCurrentItem?'>':' '), menuLabel(i), (i == menuCurrentItem?'<':' '));

}


uint8_t menuActionStatus(menuAction_t action) 
{
  //Encoder.setAccelerationEnabled(true); 
  panelState = State::StatusScreen;
  editState = State::EditModeNone; // State::Edit::PrintSpeed

  menuPreviousItem = &menuNull;


  return TRUE;
}

static struct {
  uint8_t line;
  uint8_t column;
} lcdCursor = { 0, 0 };


uint8_t oldencoderpos;


uint8_t menuActionExtrude(menuAction_t action) {
  return TRUE;
}


uint8_t menuActionMove(menuAction_t action) {
  static bool usingRelativeCoordinates = false;
  extern const MenuItem_t miMoveX, miMoveY, miMoveZ;
  uint8_t axis;

  if (action == menuActionDisplay) {
    if (!usingRelativeCoordinates) {
      enquecommand("G91");
      oldencoderpos = encoderpos;
      encoderpos = 0;
      usingRelativeCoordinates = true;
    }
    else if (encoderpos != 0) {
      char cmd[14] = "G1 F700 $$0.1";
           if (menuCurrentItem == &miMoveX) cmd[8] = 'X', axis = X_AXIS;
      else if (menuCurrentItem == &miMoveY) cmd[8] = 'Y', axis = Y_AXIS;
      else if (menuCurrentItem == &miMoveZ) cmd[8] = 'Z', axis = Z_AXIS; 
      else return FALSE;
      cmd[9] = (encoderpos > 0) ? '+': '-';
      enquecommand(cmd);
    }

    // FIXME: Position
    lcd.print(ftostr52(current_position[axis]));
  }

//  if (action == menuActionTrigger) { 
//    trigger could be used switch through predefined increment steps 0.1, 0.5, 1, 5
//  }

  if (action == menuActionParent) { 
    enquecommand("G90");
    usingRelativeCoordinates = false;
    encoderpos = oldencoderpos;
  }

  return TRUE;
}

// menuItem Arguments:
//   Name, Label, Next, Previous, Parent, Child, Callback


menuItem(miStatus, "", menuNull, menuNull, menuNull, menuNull, menuActionStatus);

menuItem(miPrepare, "Prepare...",   miControl, menuNull,  miStatus, menuNull, menuNOP);

menuItem(miControl, "Control...", miCard,    miPrepare, miStatus,    miMoveX,  menuNOP);
  menuItem(miMoveX,   "Move X",   miMoveY,   menuNull,  miControl, menuNull, menuActionMove);
  menuItem(miMoveY,   "Move Y",   miMoveZ,   miMoveX,   miControl, menuNull, menuActionMove);
  menuItem(miMoveZ,   "Move Z",   miExtrude, miMoveY,   miControl, menuNull, menuActionMove);
  menuItem(miExtrude, "Extrude",  menuNull,  miMoveZ,   miControl, menuNull, menuActionExtrude);

menuItem(miCard,    "Card...",      miResume,  miControl, miStatus, menuNull, menuNOP);
menuItem(miResume,  "Resume Print", menuNull,  miCard,    miStatus, menuNull, menuNOP);


void MainMenu::showMainMenu()
{
  menuRender(renderMenuItem, maxDisplayedMenuItems);

  return;


  if (tune) {
    if(!(movesplanned() || IS_SD_PRINTING)) {
      force_lcd_update = true;
      tune = false;
    }
  }
  else {
    if (movesplanned() || IS_SD_PRINTING) {
      force_lcd_update = true;
      tune = true;
    }
  }

  clearIfNecessary();

  uint8_t line = 0;
 
  for(int8_t i = lineoffset; i < lineoffset + LCD_HEIGHT; i++) {
    switch (i) {
      case ItemM_watch:
        MENUITEM(  LCD_PRINT_PGM(MSG_WATCH), status=Main_Status; beepshort(); );
        break;

      case ItemM_prepare:
        MENUITEM(  if(!tune) LCD_PRINT_PGM(MSG_PREPARE); else LCD_PRINT_PGM(MSG_TUNE); ,   status=Main_Prepare; beepshort(); );
        break;

      case ItemM_control:
        MENUITEM(  LCD_PRINT_PGM(MSG_CONTROL_ARROW),   status=Main_Control; beepshort(); );
        break;

#ifdef SDSUPPORT
      case ItemM_file: {
        if(force_lcd_update) {
          setCursor(0,line);
          if(CARDINSERTED) {
            if(card.sdprinting)
              LCD_PRINT_PGM(MSG_STOP_PRINT);
            else
              LCD_PRINT_PGM(MSG_CARD_MENU);
          }
          else {
            LCD_PRINT_PGM(MSG_NO_CARD);
          }
        }

        if(CARDINSERTED&&(activeline==line)&&CLICKED) {
          card.printingHasFinished();
          
          status=Main_SD;
          beepshort();
        }
      } break;

      case ItemM_pause: {
        if(force_lcd_update) {
          setCursor(0,line);
          if(CARDINSERTED) {
            if(card.sdprinting)
              LCD_PRINT_PGM(MSG_PAUSE_PRINT);
            else
              LCD_PRINT_PGM(MSG_RESUME_PRINT);
          }
        }
        if(CARDINSERTED && (activeline==line) && CLICKED) {
          if(card.sdprinting) {
            card.pauseSDPrint();
            beepshort();
            status = Main_Status;
          }
          else {
            card.startFileprint();
            starttime=millis();
            beepshort();
            status = Main_Status;
          }
        }
      } break;
#else
      case ItemM_file:
        break;
      case ItemM_pause:
        break;
#endif
    default:
      break;
    }
    line++;
  }

  uint8_t numberOfLines = 4;

#ifndef SDSUPPORT
  numberOfLines = 3;
#endif

  updateActiveLines(numberOfLines, encoderpos);
}

void MainMenu::update()
{
  static MainStatus oldstatus = Main_Menu;   //init automatically causes foce_lcd_update=true
  static unsigned long timeoutToStatus = 0;

#if (SDCARDDETECT > -1) //This code is only relivant if you have an SDcard detect pin.
  static bool oldcardstatus=false;
  if((CARDINSERTED != oldcardstatus)) {
    force_lcd_update=true;
    oldcardstatus=CARDINSERTED;

    //lcd_init();     // to maybe revive the lcd if static electricty killed it.
    //Serial.println("echo: SD CHANGE");
    
    if(CARDINSERTED) {
      card.initsd();
      LCD_MESSAGEPGM(MSG_SD_INSERTED);
    }
    else {
      card.release();
      LCD_MESSAGEPGM(MSG_SD_REMOVED);
    }
  }
#endif

  if (status != oldstatus) {
    force_lcd_update = true;
    encoderpos = 0;
    lineoffset = 0;
    oldstatus = status;
  }

  if ((encoderpos != lastencoderpos) || CLICKED) {
    timeoutToStatus = millis() + STATUSTIMEOUT;
  }

  switch (status) 
  {
    case Main_Status: {
      showStatus();
      if (CLICKED) {
        linechanging = false;
        BLOCK
        status = Main_Menu;
        timeoutToStatus = millis() + STATUSTIMEOUT;
      }
    } break;

    case Main_Menu: {
      showMainMenu();
      linechanging=false;
    } break;

    case Main_Prepare:
      (tune) ? showTune() : showPrepare();
      break;

    case Sub_PrepareMove:
      showAxisMove();
      break;

    case Main_Control:
      showControl();
      break;

    case Sub_MotionControl:
      showControlMotion();
      break;

    case Sub_RetractControl:
      showControlRetract();
      break;

    case Sub_TempControl:
      showControlTemp();
      break;

    case Main_SD:
      showSD();
      break;

    case Sub_PreheatPLASettings:
      showPLAsettings();
      break;

    case Sub_PreheatABSSettings:
      showABSsettings();
      break;
  }

  if (timeoutToStatus < millis()) {
    status = Main_Status;
  }

  lastencoderpos=encoderpos;
}

enum {
  ItemPLAPreHeat_Exit,
  ItemPLAPreHeat_set_PLA_FanSpeed,
  ItemPLAPreHeat_set_nozzle,
  ItemPLAPreHeat_set_HPB,
  ItemPLAPreHeat_Store_Eprom
};

void MainMenu::showPLAsettings()
{
  #ifdef ULTIPANEL
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {

    case ItemPLAPreHeat_Exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_TEMPERATURE_RTN),   status=Sub_TempControl; beepshort(); );
      break;

    case ItemPLAPreHeat_set_PLA_FanSpeed:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FAN_SPEED);
        setCursor(13,line); lcd.print(ftostr3(plaPreheatFanSpeed));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=plaPreheatFanSpeed;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>255) encoderpos=255;
        plaPreheatFanSpeed=encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;

    case ItemPLAPreHeat_set_nozzle:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE);
        setCursor(13,line); lcd.print(ftostr3(plaPreheatHotendTemp));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=plaPreheatHotendTemp;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        plaPreheatHotendTemp = encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;

    case ItemPLAPreHeat_set_HPB:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_BED);
        setCursor(13,line); lcd.print(ftostr3(plaPreheatHPBTemp));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=plaPreheatHPBTemp;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>250) encoderpos=150;
        plaPreheatHPBTemp = encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;
    case ItemPLAPreHeat_Store_Eprom:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_STORE_EPROM);
      }
      if((activeline==line) && CLICKED)
      {
        //enquecommand("M84");
        beepshort();
        
        Config_StoreSettings();
      }
    } break;
    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemPLAPreHeat_Store_Eprom,encoderpos);
  #endif
}

enum {
  ItemABSPreHeat_Exit,
  ItemABSPreHeat_set_FanSpeed,
  ItemABSPreHeat_set_nozzle,
  ItemABSPreHeat_set_HPB,
  ItemABSPreHeat_Store_Eprom
};

void MainMenu::showABSsettings()
{
  #ifdef ULTIPANEL
  uint8_t line=0;
  clearIfNecessary();
  for(int8_t i=lineoffset; i<lineoffset+LCD_HEIGHT; i++)
  {
    switch(i)
    {

    case ItemABSPreHeat_Exit:
      MENUITEM(  LCD_PRINT_PGM(MSG_TEMPERATURE_RTN),   status=Sub_TempControl; beepshort(); );
      break;

    case ItemABSPreHeat_set_FanSpeed:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_FAN_SPEED);
        setCursor(13,line); lcd.print(ftostr3(absPreheatFanSpeed));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=absPreheatFanSpeed;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>255) encoderpos=255;
        absPreheatFanSpeed=encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;

    case ItemABSPreHeat_set_nozzle:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_NOZZLE);
        setCursor(13,line); lcd.print(ftostr3(absPreheatHotendTemp));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=absPreheatHotendTemp;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>260) encoderpos=260;
        absPreheatHotendTemp = encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;

    case ItemABSPreHeat_set_HPB:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_BED);
        setCursor(13,line); lcd.print(ftostr3(absPreheatHPBTemp));
      }

      if((activeline!=line) )
        break;

      if(CLICKED)
      {
        linechanging=!linechanging;
        if(linechanging)
        {
          encoderpos=absPreheatHPBTemp;
        }
        else
        {
          encoderpos=activeline;
          beepshort();
        }
        
      }
      if(linechanging)
      {
        if(encoderpos<0) encoderpos=0;
        if(encoderpos>250) encoderpos=150;
        absPreheatHPBTemp = encoderpos;
        setCursor(13,line); lcd.print(itostr3(encoderpos));
      }
    } break;
    case ItemABSPreHeat_Store_Eprom:
    {
      if(force_lcd_update)
      {
        setCursor(0,line); LCD_PRINT_PGM(MSG_STORE_EPROM);
      }
      if((activeline==line) && CLICKED)
      {
        //enquecommand("M84");
        beepshort();
        
        Config_StoreSettings();
      }
    } break;
    default:
      break;
    }
    line++;
  }
  updateActiveLines(ItemABSPreHeat_Store_Eprom,encoderpos);
  #endif
}

//**********************************************************************************************************
//  convert float to string with +123.4 format
char *ftostr3(const float &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/100)%10+'0';
  conv[1]=(xx/10)%10+'0';
  conv[2]=(xx)%10+'0';
  conv[3]=0;
  return conv;
}

char *itostr2(const uint8_t &x)
{
  //sprintf(conv,"%5.1f",x);
  int xx=x;
  conv[0]=(xx/10)%10+'0';
  conv[1]=(xx)%10+'0';
  conv[2]=0;
  return conv;
}

//  convert float to string with +123.4 format
char *ftostr31(const float &x)
{
  int xx=x*10;
  conv[0]=(xx>=0) ? '+' : '-';
  xx=abs(xx);
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *ftostr32(const float &x)
{
  long xx=x*100;
  conv[0]=(xx>=0) ? '+' : '-';
  xx=abs(xx);
  conv[1]=(xx/100)%10+'0';
  conv[2]='.';
  conv[3]=(xx/10)%10+'0';
  conv[4]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr31(const int &xx)
{
  conv[0]=(xx>=0) ? '+' : '-';
  conv[1]=(xx/1000)%10+'0';
  conv[2]=(xx/100)%10+'0';
  conv[3]=(xx/10)%10+'0';
  conv[4]='.';
  conv[5]=(xx)%10+'0';
  conv[6]=0;
  return conv;
}

char *itostr3(const int &xx)
{
  conv[0]=(xx/100)%10+'0';
  conv[1]=(xx/10)%10+'0';
  conv[2]=(xx)%10+'0';
  conv[3]=0;
  return conv;
}

char *itostr4(const int &xx)
{
  conv[0]=(xx/1000)%10+'0';
  conv[1]=(xx/100)%10+'0';
  conv[2]=(xx/10)%10+'0';
  conv[3]=(xx)%10+'0';
  conv[4]=0;
  return conv;
}

//  convert float to string with +1234.5 format
char *ftostr51(const float &x)
{
  long xx=x*10;
  conv[0]=(xx>=0) ? '+' : '-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]=(xx/10)%10+'0';
  conv[5]='.';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

//  convert float to string with +123.45 format
char *ftostr52(const float &x)
{
  long xx=x*100;
  conv[0]=(xx>=0) ? '+' : '-';
  xx=abs(xx);
  conv[1]=(xx/10000)%10+'0';
  conv[2]=(xx/1000)%10+'0';
  conv[3]=(xx/100)%10+'0';
  conv[4]='.';
  conv[5]=(xx/10)%10+'0';
  conv[6]=(xx)%10+'0';
  conv[7]=0;
  return conv;
}

void beep()
{
  #if (BEEPER > -1)
  pinMode(BEEPER,OUTPUT);
  for(int8_t i=0; i<20; i++) {
    WRITE(BEEPER,HIGH);
    delay(5);
    WRITE(BEEPER,LOW);
    delay(5);
  }
  #endif
}

void beepshort()
{
  #if (BEEPER > -1)
  pinMode(BEEPER,OUTPUT);
  for(int8_t i=0; i<10; i++) {
    WRITE(BEEPER,HIGH);
    delay(3);
    WRITE(BEEPER,LOW);
    delay(3);
  }
  #endif
}

#endif //ULTRA_LCD

