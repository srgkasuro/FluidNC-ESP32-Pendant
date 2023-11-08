#include <lvgl.h>
#include <TFT_eSPI.h>
#include <ui.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

#include "FS.h"

void runselectedFile(lv_event_t *e = NULL);
#define JSCALIBRATION_FILE "/JoystickZero"
#define TOUCHCALIBRATION_FILE "/touchcalibrationData"
#define RXD2 26
#define TXD2 27
#define S2SPEED 115200
#define Enc_A 14    // Encoder Pin A
#define Enc_B 12    // Encoder Pin B
#define ZEROBUT 25  // Button zum nullen Pin
#define ZEROBUTLED 16  // Button zum nullen Pin
#define JSX 32      // Joystick analog X
#define JSX_Treshold 200
uint16_t JSX_Zero = 1955;
#define JSY 33  // Joystick analog Y
#define JSY_Treshold 200
uint16_t JSY_Zero = 2000;
#define JSZ 35  // Joystick analog Z
#define JSZ_Treshold 200
uint16_t JSZ_Zero = 1900;
#define JSButton 13  // Joystick digital button
#define steps_per_mm 800
#define microsteps 32
#define Interuptintervall 50  // 100 ms
#define JogVorschubMAX 6000    // F=3000 (mm/min)
#define EncoderposMAX 255
#define achse_active_timeout 10000
#define ButtonDebounce 200
#define SerialWaitTimeout 20000
volatile uint32_t SerialWaitTimeoutTimer = 0;
volatile uint32_t DebounceTimer = 0;
volatile bool DebounceTimer_aktiv = false;
volatile bool achse_aktiv_counter = false;
volatile bool aktiveachsenullen = false;
volatile bool ABCaktiv = false;
volatile bool JS_single_axis = true;
volatile bool JSX_activ = false;
volatile bool JSY_activ = false;
volatile bool JSZ_activ = false;
volatile int timeout;
volatile int timeout_old;
volatile int JSX_pos = 0;       // Encoder Position
volatile int JSX_pos_last = 0;  // EncncTCP.h>oder Position
volatile int JSX_readout = 0;
volatile int JSY_pos = 0;       // Encoder Position
volatile int JSY_pos_last = 0;  // Encoder Position
volatile int JSY_readout = 0;
volatile int JSZ_pos = 0;       // Encoder Position
volatile int JSZ_pos_last = 0;  // Encoder Position
volatile int JSZ_readout = 0;
volatile int Encoder_pos = 0;  // Encoder Position
volatile int Encoder_pos_2send = 0;
volatile bool Encoder_state;     // Encoder State A High / Low
volatile bool Encoder_last = 1;  // Encoder last recorded State A
volatile bool sende_neuen_Encoder_jogbefehl = false;
volatile bool sende_neuen_Joystick_jogbefehl = false;
volatile int interruptCounter;
volatile char aktive_achse = '-';
volatile char letzte_aktive_achse = '-';
volatile bool doJoystickauslesen = false;
int Fortschritt = 0;
bool jobrunning = false;
bool jobdone = true;
bool beendeJob=false;
int Wiederholungen2do =  0;
int Wiederholungendone =  0;
bool repeat = true;
bool firstrun = true;
bool Joystickenabled = true;
bool OTA =false;
bool recalibrateTouch = false;

String cmd = "";
char Temp_String[100] = "";
const char nl[] = "\n";
const char jog_cancel = char(0x85);

volatile uint32_t achse_aktiv_timer;

#define LV_INDEV_LONG_PRESS_TIME 400

const unsigned int MAX_MESSAGE_LENGTH = 90;
/*Don't forget to set Sketchbook location in File/Preferencesto the path of your UI project (the parent foder of this INO file)*/
const byte numChars = 100;

//Position variables
float MPos_X = 0;
float MPos_Y = 0;
float MPos_Z = 0;
float MPos_A = 0;
float MPos_B = 0;
float MPos_C = 0;

char Dateien[20][numChars];
int DateienCounter = 0;
int NumberofDateien = 0;

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

static const char *kb_map[] = { "$", "!", "L", "M", "P", "H", "\n",
                                "G0 ", "G1 ", "F", "S", "G90 ", "G91", "\n",
                                "X", "Y", "Z", "A", "B", "C", "\n",
                                "1", "2", "3", "-", "\n",
                                "4", "5", "6", "~", "\n",
                                "7", "8", "9", "?", "\n",
                                LV_SYMBOL_BACKSPACE, "0", " ", LV_SYMBOL_OK, "" };  //*/

static const lv_btnmatrix_ctrl_t kb_ctrl[] = { 1, 1, 1, 1, 1, 1,
                                               1, 1, 1, 1, 1, 1,
                                               1, 1, 1, 1, 1, 1,
                                               1, 1, 1, 1,
                                               1, 1, 1, 1,
                                               1, 1, 1, 1,
                                               1, 1, 1, 1 };  //*/



hw_timer_t *timer = NULL;

volatile bool ok = false;
volatile bool setRoller = false;

//============
/*Change to your screen resolution*/
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 480;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

TFT_eSPI tft = TFT_eSPI(screenWidth, screenHeight); /* TFT instance */

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf) {
  Serial.printf(buf);
  Serial.flush();
}
#endif


void toggleABCaktiv() {
  //ABCaktiv=!ABCaktiv;
  detachInterrupt(digitalPinToInterrupt(JSButton));
  DebounceTimer_aktiv = true;
  DebounceTimer = millis();
  if (ABCaktiv) {
    _ui_state_modify(ui_ButtonX, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_ButtonY, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_ButtonZ, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_ButtonA, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_ButtonB, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_ButtonC, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    ABCaktiv = false;
  } else {
    _ui_state_modify(ui_ButtonX, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_ButtonY, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_ButtonZ, LV_STATE_USER_1, _UI_MODIFY_STATE_REMOVE);
    _ui_state_modify(ui_ButtonA, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_ButtonB, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    _ui_state_modify(ui_ButtonC, LV_STATE_USER_1, _UI_MODIFY_STATE_ADD);
    ABCaktiv = true;
  }
  //Serial.println("Joystickbutton pressed");
}

/*-----------------------------------------------------------------------------------
/////////////////---- Timerinterrupt: alle 100ms ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void IRAM_ATTR onTimer() {
  doJoystickauslesen = true;
}
//portENTER_CRITICAL_ISR(&timerMux);
void Joystickauslesen() {
  if (!(JS_single_axis && (JSY_activ || JSZ_activ))) {
    JSX_readout = analogRead(JSX);

    if ((JSX_readout - JSX_Zero) > JSX_Treshold) {
      JSX_activ = true;
      if (ABCaktiv) {
        aktive_achse = 'A';
      } else {
        aktive_achse = 'X';
      }
      achse_aktiv_timer = millis();
      achse_aktiv_counter = true;
      JSX_pos = (int)((float)(JSX_readout - JSX_Zero - JSX_Treshold) / (4096 - JSX_Zero - JSX_Treshold) * 255);
    } else if ((JSX_readout - JSX_Zero) < -JSX_Treshold) {
      JSX_activ = true;
      if (ABCaktiv) {
        aktive_achse = 'A';
      } else {
        aktive_achse = 'X';
      }

      achse_aktiv_timer = millis();
      achse_aktiv_counter = true;
      JSX_pos = (int)((float)(JSX_readout) / (JSX_Zero - JSX_Treshold) * 255) - 255;
    } else {
      JSX_activ = false;

      JSX_pos = 0;
    }
  }

  if (!(JS_single_axis && (JSX_activ || JSZ_activ))) {
    JSY_readout = analogRead(JSY);

    if ((JSY_readout - JSY_Zero) > JSY_Treshold) {
      JSY_activ = true;

      if (ABCaktiv) {
        aktive_achse = 'B';
      } else {
        aktive_achse = 'Y';
      }

      achse_aktiv_timer = millis();
      achse_aktiv_counter = true;
      JSY_pos = (int)((float)(JSY_readout - JSY_Zero - JSY_Treshold) / (4096 - JSY_Zero - JSY_Treshold) * 255);
    } else if ((JSY_readout - JSY_Zero) < -JSY_Treshold) {
      JSY_activ = true;
      if (ABCaktiv) {
        aktive_achse = 'B';
      } else {
        aktive_achse = 'Y';
      }

      achse_aktiv_timer = millis();
      achse_aktiv_counter = true;
      JSY_pos = (int)((float)(JSY_readout) / (JSY_Zero - JSY_Treshold) * 255) - 255;
    } else {
      JSY_pos = 0;
      JSY_activ = false;
    }
  }

  if (!(JS_single_axis && (JSX_activ || JSY_activ))) {
    JSZ_readout = analogRead(JSZ);

    if ((JSZ_readout - JSZ_Zero) > JSZ_Treshold) {
      JSZ_activ = true;
      if (ABCaktiv) {
        aktive_achse = 'C';
      } else {
        aktive_achse = 'Z';
      }

      achse_aktiv_timer = millis();
      achse_aktiv_counter = true;
      if (ABCaktiv) {
        JSZ_pos = (int)((float)(JSZ_readout - JSZ_Zero - JSZ_Treshold) / (4096 - JSZ_Zero - JSZ_Treshold) * 255);
        } else {
        JSZ_pos = (int)(-1 * (float)(JSZ_readout - JSZ_Zero - JSZ_Treshold) / (4096 - JSZ_Zero - JSZ_Treshold) * 255);
      }
    } else if ((JSZ_readout - JSZ_Zero) < -JSZ_Treshold) {
      JSZ_activ = true;
      if (ABCaktiv) {
        aktive_achse = 'C';
        achse_aktiv_timer = millis();
        achse_aktiv_counter = true;
        JSZ_pos = (int)((((float)(JSZ_readout) / (JSZ_Zero - JSZ_Treshold) * 255) - 255));
      } else {
        aktive_achse = 'Z';
        achse_aktiv_timer = millis();
        achse_aktiv_counter = true;
        JSZ_pos = (int)(-1 * (((float)(JSZ_readout) / (JSZ_Zero - JSZ_Treshold) * 255) - 255));
      }

      
    } else {
      JSZ_pos = 0;
      JSZ_activ = false;
    }
  }

  if (Encoder_pos != 0) {
    if(aktive_achse == 'Z'){
      Encoder_pos_2send = -Encoder_pos;
    }else{
      Encoder_pos_2send = Encoder_pos;
    }
    Encoder_pos = 0;
    sende_neuen_Encoder_jogbefehl = true;
  } else if (Encoder_pos_2send != 0) {
    Encoder_pos_2send = 0;
    sende_neuen_Encoder_jogbefehl = true;
  }
  //*
  if ((JSX_pos_last == 0) && (JSY_pos_last == 0) && (JSZ_pos_last == 0)) {
    sende_neuen_Joystick_jogbefehl = false;
  } else {
    sende_neuen_Joystick_jogbefehl = true;
  }
  JSX_pos_last = JSX_pos;
  JSY_pos_last = JSY_pos;
  JSZ_pos_last = JSZ_pos;
  // portEXIT_CRITICAL_ISR(&timerMux);
  //*/
}

/*-----------------------------------------------------------------------------------
/////////////////---- READING ENCODER ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void doEncoder() {                // found a falling Edge on channel A
  if (digitalRead(Enc_B) == 1) {  // check channel B to see which way encoder is turning
    Encoder_pos--;                // CCW
  } else {
    Encoder_pos++;  // CW
  }
}  // Do Encoder


/*-----------------------------------------------------------------------------------
/////////////////---- Jog-weite berechnen ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

float JogWeite(int Encoderschritte) {
  return float(JogVorschubMAX) / float(EncoderposMAX) * float(Encoderschritte) / 600;
  // Bei EncoderposMAX  ist JogVorschub JogVorschubMAX : Vorschub = mm/min 3000 mm/min = 50 mm/s = 5mm/100ms  -> JogWeite = 5mm
}
/*-----------------------------------------------------------------------------------
/////////////////---- Jog-Vorschub berechnen ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

int JogVorschub(int Encoderschritte) {
  return abs(JogVorschubMAX / EncoderposMAX * Encoderschritte);
  // JogVorschub F: Vorschub = mm/min 3000 mm/min = 50 mm/s = 5mm/100ms
}

/*-----------------------------------------------------------------------------------
/////////////////---- SEND G-Code ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void sendeEncoderJog(int value) {
  if (value != 0) {
    // String cmd = "$J=G91 F";
    // cmd += JogVorschub(value);
    // cmd += aktive_achse;
    // cmd += JogWeite(value);
    // cmd += "\n";



    String cmd = "$J=G91 F";
    cmd += float(abs(value))*0.01/Interuptintervall*1000*60;  //(ms) F in mm/min = Strecke/Interuptintervall(ms) *1000*60
    cmd +=" ";
    cmd += aktive_achse;
    // if (value>0){
    //   cmd += "0.01";
    // } else {
    //   cmd += "-0.01";
    // }
    cmd += (float(value)*0.01);
    cmd += "\n";
    sendeSerial(cmd, true);
  } else {
    //Serial.print("jogCancel\n");  //jog_cancel);
    sendeSerial("\x85\n", true);  //jog_cancel);
  }//*/

}  // doSendGCode
void sendeJoystickJog(int xvalue, int yvalue, int zvalue) {
  if (xvalue + yvalue + zvalue != 0) {
    String cmd = "$J=G91 F";
    cmd += JogVorschub(max(max(abs(xvalue), abs(yvalue)), abs(zvalue)));
    switch (aktive_achse) {
      case 'X':
        cmd += " X";
        cmd += JogWeite(xvalue);
        break;
      case 'Y':
        cmd += " Y";
        cmd += JogWeite(yvalue);
        break;
      case 'Z':
        cmd += " Z";
        cmd += JogWeite(zvalue);
        break;
      case 'A':
        cmd += " A";
        cmd += JogWeite(xvalue);
        break;
      case 'B':
        cmd += " B";
        cmd += JogWeite(yvalue);
        break;
      case 'C':
        cmd += " C";
        cmd += JogWeite(zvalue);
        break;
      default:
        break;
    }
    cmd += "\n";
    //Serial.print(cmd);
    sendeSerial(cmd, true);
  } else {
    //Serial.print("jogCancel\n");//jog_cancel);
    sendeSerial("\x85\n", true);  //jog_cancel);
  }

}  // doSendGCode


void SendGCode(lv_event_t *e) {
  //Serial.println(lv_textarea_get_text(ui_TextArea1));
  const char r[] = "\r";
  sendeSerial(lv_textarea_get_text(ui_TextArea1), false);
  sendeSerial(r, false);
}

/*-----------------------------------------------------------------------------------
/////////////////---- Set Config ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void setConfig1(lv_event_t *e) {
  //Serial.println("M63 P0");
  //Serial.println("M63 P1");
  //Serial.println("M63 P2");
  //Serial.println("M63 P3");
  sendeSerial("M63 P0\r", true);
  sendeSerial("M63 P1\r", true);
  sendeSerial("M63 P2\r", true);
  sendeSerial("M63 P3\r", true);
}

void setConfig2(lv_event_t *e) {
  //Serial.println("M63 P1");
  //Serial.println("M63 P2");
  //Serial.println("M63 P3");
  //Serial.println("M62 P0");
  sendeSerial("M63 P1\r", true);
  sendeSerial("M63 P2\r", true);
  sendeSerial("M63 P3\r", true);
  sendeSerial("M62 P0\r", true);
}

void setConfig3(lv_event_t *e) {
  //Serial.println("M63 P0");
  //Serial.println("M63 P2");
  //Serial.println("M63 P3");
  //Serial.println("M62 P1");
  sendeSerial("M63 P0\r", true);
  sendeSerial("M63 P2\r", true);
  sendeSerial("M63 P3\r", true);
  sendeSerial("M62 P1\r", true);
}

void setConfig4(lv_event_t *e) {
  //Serial.println("M63 P0");
  //Serial.println("M63 P1");
  //Serial.println("M63 P3");
  //Serial.println("M62 P2");
  sendeSerial("M63 P0\r", true);
  sendeSerial("M63 P1\r", true);
  sendeSerial("M63 P3\r", true);
  sendeSerial("M62 P2\r", true);
}

void setConfig5(lv_event_t *e) {
  //Serial.println("M63 P0");
  //Serial.println("M63 P1");
  //Serial.println("M63 P2");
  //Serial.println("M62 P3");
  sendeSerial("M63 P0\r", true);
  sendeSerial("M63 P1\r", true);
  sendeSerial("M63 P2\r", true);
  sendeSerial("M62 P3\r", true);
}
/*-----------------------------------------------------------------------------------
/////////////////---- toggle Spindel ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void toggleSpindel(lv_event_t *e) {
  if (lv_obj_has_state(ui_ButtonSpindle, LV_STATE_CHECKED)) {
    sendeSerial("M3 S1\n", true);
  } else{
    sendeSerial("M5\n", true);
  }
}
/*-----------------------------------------------------------------------------------
/////////////////---- toggle Absaugung ENCODER ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void toggleAbsaugung(lv_event_t *e) {
    if (lv_obj_has_state(ui_ButtonAbsaug, LV_STATE_CHECKED)) {
    sendeSerial("M8\n", true);
  } else{
    sendeSerial("M9\n", true);
  }
}
/*-----------------------------------------------------------------------------------
/////////////////---- load Dateien ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void loadFiles(lv_event_t *e) {
  DateienCounter = 0;
  if (lv_obj_has_flag(ui_Roller, LV_OBJ_FLAG_HIDDEN)) {
    sendeSerial("$LocalFS/List\r", true);
    setRoller = true;
    _ui_flag_modify(ui_Roller, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
  } else {
    _ui_flag_modify(ui_Roller, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
  }
}
/*-----------------------------------------------------------------------------------
/////////////////---- doHomeing ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void doHome_All(lv_event_t *e) {
  sendeSerial("$H\r", false);
}

void doHome_X(lv_event_t *e) {
  sendeSerial("$HX\r", false);
}

void doHome_Y(lv_event_t *e) {
  sendeSerial("$HY\r", false);
}

void doHome_Z(lv_event_t *e) {
  sendeSerial("$HZ\r", false);
}

void doHome_A(lv_event_t *e) {
  sendeSerial("$HA\r", false);
}

void doHome_B(lv_event_t *e) {
  sendeSerial("$HB\r", false);
}
/*-----------------------------------------------------------------------------------
/////////////////---- setToZERO ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void setactivetoZero() {
  aktiveachsenullen = true;
}


void settoZeroX(lv_event_t *e) {
  sendeSerial("G10 L20 P0 X0\r", true);
}

void settoZeroY(lv_event_t *e) {
  sendeSerial("G10 L20 P0 Y0\r", true);
}

void settoZeroZ(lv_event_t *e) {
  sendeSerial("G10 L20 P0 Z0\r", true);
}

void settoZeroA(lv_event_t *e) {
  sendeSerial("G10 L20 P0 A0\r", true);
}

void settoZeroB(lv_event_t *e) {
  sendeSerial("G10 L20 P0 B0\r", true);
}

void settoZeroC(lv_event_t *e) {
  sendeSerial("G10 L20 P0 C0\r", true);
}

void settoZeroAll(lv_event_t * e)
{
    sendeSerial("G10 L20 P0 X0 Y0 Z0 A0 B0 C0\r", true);
	// Your code here
}

/*-----------------------------------------------------------------------------------
/////////////////---- Buttons: select File ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void selectFile(lv_event_t *e) {
  lv_label_set_text(ui_LabelFile, Dateien[lv_roller_get_selected(ui_Roller)]);
}
/*-----------------------------------------------------------------------------------
/////////////////---- Buttons: set aktive Achse ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/


void setaktiveAchseX(lv_event_t *e) {
  aktive_achse = 'X';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (ABCaktiv){
    toggleABCaktiv();
  }
}

void setaktiveAchseY(lv_event_t *e) {
  aktive_achse = 'Y';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (ABCaktiv){
    toggleABCaktiv();
  }
}

void setaktiveAchseZ(lv_event_t *e) {
  aktive_achse = 'Z';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (ABCaktiv){
    toggleABCaktiv();
  }
}

void setaktiveAchseA(lv_event_t *e) {
  aktive_achse = 'A';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (!ABCaktiv){
    toggleABCaktiv();
  }
}

void setaktiveAchseB(lv_event_t *e) {
  aktive_achse = 'B';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (!ABCaktiv){
    toggleABCaktiv();
  }
}

void setaktiveAchseC(lv_event_t *e) {
  aktive_achse = 'C';
  achse_aktiv_timer = millis();
  achse_aktiv_counter = true;
  if (!ABCaktiv){
    toggleABCaktiv();
  }
}

/*-----------------------------------------------------------------------------------
/////////////////---- Run File ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void runselectedFile(lv_event_t *e) {
  Joystickenabled=false;
  timerAlarmDisable(timer);
  if (jobrunning) {
    beendeJob=true;
    repeat = false;
    _ui_state_modify(ui_ButtonRun, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
  } else {
    if (firstrun) {
      Wiederholungen2do = atoi(lv_label_get_text(ui_LabelWiederholungen));
      Wiederholungendone = 0;
      if (Wiederholungen2do == 0) {
        repeat = true;
      } else {
          repeat = false;
      }  
      firstrun = false;
    }
    _ui_state_modify(ui_ButtonRun, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
    //char Temp_String[40] = "";
    //Temp_String[100] = "";
    const char cmd[] = "$LocalFS/Run=";
    //const char nl[] = "\n";
    strcpy(Temp_String, cmd);
    strcat(Temp_String, lv_label_get_text(ui_LabelFile));
    strcat(Temp_String, nl);
    //Serial.print(Temp_String);
    sendeSerial(Temp_String, true);
    jobrunning = true;
    jobdone = false;
  }
}

void readJSCalibrationfile(){

  uint16_t calData[3];
  bool calDataOK = false;

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(JSCALIBRATION_FILE)) {
      Serial.println("lese Joystickkalibrierungs Datei");
      fs::File f = SPIFFS.open(JSCALIBRATION_FILE, "r");
      if (f) {
        if (f.readBytes((char *)calData, 7) == 7)
          calDataOK = true;
        f.close();
        Serial.println(calData[0]);
        Serial.println(calData[1]);
        Serial.println(calData[2]);
        JSX_Zero = calData[0];
        JSY_Zero = calData[1];
        JSZ_Zero = calData[2];
      }
  }else{
    calibrateJoystick();
    Serial.println("Datei nicht vorhanden: kalibriere neu");
    return;
  }
  if(!digitalRead(ZEROBUT) && digitalRead(JSButton)){
    calibrateJoystick();
    Serial.println("Neukalibrierung angefordert: kalibriere neu");
  }
}


/*-----------------------------------------------------------------------------------
///////////////// calibrate Joystick //////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void calibrateJoystick()
{
  uint16_t calData[3];
  bool calDataOK = true;

  calData[0] = analogRead(JSX);
  calData[1] = analogRead(JSY);
  calData[2] = analogRead(JSZ);
          JSX_Zero = calData[0];
        JSY_Zero = calData[1];
        JSZ_Zero = calData[2];

          Serial.println(calData[0]);
        Serial.println(calData[1]);
        Serial.println(calData[2]);

  // check file system exists
  if (!SPIFFS.begin()) {
    Serial.println("Formating file system");
    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists and size is correct
  if (SPIFFS.exists(JSCALIBRATION_FILE)) {
      // Delete old file
      SPIFFS.remove(JSCALIBRATION_FILE);
  }

    // store data
    fs::File f = SPIFFS.open(JSCALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calData, 7);
      f.close();
  }
}





/*-----------------------------------------------------------------------------------
///////////////// Display flushing //////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  tft.startWrite();
  tft.setAddrWindow(area->x1, area->y1, w, h);
  tft.pushColors((uint16_t *)&color_p->full, w * h, true);
  tft.endWrite();

  lv_disp_flush_ready(disp);
}
/*-----------------------------------------------------------------------------------
/////////////////Read the touchpad-///////////////////////////////////////////
-----------------------------------------------------------------------------------*/

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) { uint16_t touchX = 0, touchY = 0;

  bool touched = tft.getTouch(&touchX, &touchY, 600);

  if (!touched) {
    data->state = LV_INDEV_STATE_REL;
  } else {
    data->state = LV_INDEV_STATE_PR;
    /*Set the coordinates*/
    data->point.x = touchX;
    data->point.y = touchY;
  }
}


/*-----------------------------------------------------------------------------------
/////////////////---- sende Serial new ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void sendeSerial(char Data2send[], bool wait4ok) {
  ok = !wait4ok;
  Serial2.print(Data2send);
  Serial.print(Data2send);
  SerialWaitTimeoutTimer=millis();
  while (!ok) {
    empfangeSerial();
     if ( (millis() - SerialWaitTimeoutTimer ) > SerialWaitTimeout) {
       return;
     }
  }
}

void sendeSerial(String Data2send, bool wait4ok) {
  ok = !wait4ok;
  Serial2.print(Data2send);
  Serial.print(Data2send);
  SerialWaitTimeoutTimer=millis();
  while (!ok) {
    empfangeSerial();
     if ( (millis() - SerialWaitTimeoutTimer ) > SerialWaitTimeout) {
       return;
     }
  }
}

/*-----------------------------------------------------------------------------------
/////////////////---- receive Serial new ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
char Status[6] = "";

void empfangeSerial() {
  timerAlarmDisable(timer);
  char rc;
  char Temp[100] = "";
  char Check[20] = "";
  int i = 0;
  int n = 0;
  int state = 0;
  while (Serial2.available() > 0) {
    rc = Serial2.read();
    Serial.print(rc);
    switch (state) {
      case 0:
        switch (rc) {
          case ('['):
            state = 21;
            i = 0;
            break;
          case ('<'):
            state = 2;
            i = 0;
            break;
          case ('o'):
            state = 1;
            break;
          case ('>'):
            break;
          case (']'):
            break;
          case ('G'):
            state = 40;
            i = 0;
            break;
          //case ('\n'):
          //  break;
          default:
            //Serial.print(rc);
            break;
        }
        break;
      case 1:
        if (rc == 'k') {
          ok = true;
          //Serial.print("OK");
          state = 0;
        }
        break;
      case 2:
        if (rc == '|') {
          Status[i] = '\0';
          lv_label_set_text(ui_LabelSTATE, Status);
          //if (strcmp(Status,"Idle")==0){
          //  jobdone = true;
          //}
          strcpy(Status, "");
          state = 3;
          i = 0;
        } else {
          Status[i] = rc;
          i += 1;
        }
        break;
      case 3:
        if (rc == ':') {
          state = 4;
          i = 0;
        }
        break;
      case 4:
        if (rc == ',') {
          Temp[i] = '\0';
          MPos_X = atof(Temp);
          lv_label_set_text(ui_LabelX, Temp);
          state = 5;
          i = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 5:
        if (rc == ',') {
          Temp[i] = '\0';
          MPos_Y = atof(Temp);
          lv_label_set_text(ui_LabelY, Temp);
          state = 6;
          i = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 6:
        if (rc == ',') {
          Temp[i] = '\0';
          MPos_Z = atof(Temp);
          lv_label_set_text(ui_LabelZ, Temp);
          state = 7;
          i = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 7:
        if (rc == ',') {
          Temp[i] = '\0';
          MPos_A = atof(Temp);
          lv_label_set_text(ui_LabelA, Temp);
          state = 8;
          i = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 8:
        if (rc == ',') {
          Temp[i] = '\0';
          MPos_B = atof(Temp);
          lv_label_set_text(ui_LabelB, Temp);
          state = 9;
          i = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 9:
        if (rc == '|') {
          Temp[i] = '\0';
          MPos_C = atof(Temp);
          lv_label_set_text(ui_LabelC, Temp);
          state = 10;
          i = 0;
        } else if (rc == '>') {
          Temp[i] = '\0';
          MPos_C = atof(Temp);
          lv_label_set_text(ui_LabelC, Temp);
          state = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 10:
        switch (rc) {
          case 'B':  //Bf:15,128
            state = 11;
            break;
          case 'L':  //Ln:99999
            state = 12;
            break;
          case 'F':  // F:500 oder: FS:500,8000
            i = 0;
            state = 13;
            break;
          case 'P':  // Pn:XYZPDHRS
            state = 14;
            break;
          case 'O':  //Ov:100,100,100
            state = 15;
            break;
          case 'A':  //A:SCFM
            state = 16;
            break;
          case 'W':  //WCO:116.433,-63.066,82.001,56.092,41.691,-8.750
            state = 17;
            break;
          case 'S':  //SD:00.00,/littleFS/File
            state = 18;
            break;
          default:
            //Serial.print(rc);
            break;
        }
        break;
      case 11:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 12:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 13:
        if (rc == '|') {
          Temp[i] = '\0';
          lv_label_set_text(ui_LabelFeed, Temp);
          i = 0;
          state = 10;
        } else if (rc == ':') {
        } else if (rc == 'S') {
        } else if (rc == ',') {
          Temp[i] = '\0';
          lv_label_set_text(ui_LabelFeed, Temp);
          i = 0;
          state = 11;
        } else if (rc == '>') {
          Temp[i] = '\0';
          lv_label_set_text(ui_LabelFeed, Temp);
          i = 0;
          state = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 14:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 15:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 16:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 17:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 18:  //SD:00.00,/littleFS/File
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else if (rc == ':') {
          state = 19;
          i = 0;
        } else {
        }
        break;
      case 19:
        if (rc == ',') {
          Temp[i] = '\0';
          Fortschritt = atoi(Temp);
          lv_bar_set_value(ui_BarFortschritt, atoi(Temp), LV_ANIM_OFF);
          i = 0;
          state = 20;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 20:
        if (rc == '|') {
          state = 10;
        } else if (rc == '>') {
          state = 0;
        } else {
        }
        break;
      case 21:
        switch (rc) {
          case 'M':  //[MSG: /littlefs/Beschriftung_ALEX_TEST.nc file job succeeded]  --> Ende des Jobs auswerten! (allerdings läuft die Maschine noch weiter, bis alle Befehle abgearbeitet wurden!)
            if (!jobdone){
              state = 22;  
            } else {
              state = 32;
            }
            break;
          case 'V':  //VER
            state = 23;
            break;
          case 'G':  //[GC:G1 G54 G17 G21 G90 G94 M5 M8 T0 F1000 S24000]
            state = 24;
            break;
          case 'e':  //echo
            state = 25;
            break;
          case 'H':  //HLP
            state = 26;
            break;
          case 'O':  //
            state = 27;
            break;
          case 'T':  //
            state = 28;
            break;
          case '/':  ///littlefs/ Free:24.00 KB Used:168.00 KB Total:192.00 KB]
            state = 29;
            break;
          case 'F':  //FILE: Beschriftung_ALEX_TEST.nc|SIZE:10236]
            state = 30;
            break;
          default:
            state = 0;
            //Serial.print(rc);
            break;
        }
        break;
      case 22:
        Serial2.setTimeout(200);
        if (Serial2.findUntil("file job succeeded", "]")) {
          jobdone = true;
        }
        Serial2.setTimeout(1000);
        state = 0;
        break;
      case 29:  //FILE: Beschriftung_ALEX_TEST.nc|SIZE:10236]
        if (rc == ']') {
          state = 0;
          NumberofDateien = DateienCounter;
          i = 0;
        }
        break;
      case 30:  //FILE: Beschriftung_ALEX_TEST.nc|SIZE:10236]
        if (rc == ' ') {
          state = 31;
          i = 0;
        }
        break;
      case 31:
        if (rc == '|') {
          Temp[i] = '\0';
          //Dateien[n]=Temp;
          strcpy(Dateien[DateienCounter], Temp);
          DateienCounter += 1;
          i = 0;
          state = 0;
        } else {
          Temp[i] = rc;
          i += 1;
        }
        break;
      case 32:  //MSG... ]
        if (rc == ']') {
          state = 0;
        }
        break;
      case 40:
        if (rc == 'r') {
          //Serial.print("OK");
          state = 41;
        }else{
          state = 0;
        }
        break;
      case 41:
        if (rc == 'b') {
          //Serial.print("OK");
          state = 42;
        }else{
          state = 0;
        }
        break;
      case 42:
        if (rc == 'l') {
          ok = true;
          //Serial.print("OK");
          state = 0;
        }else{
          state = 0;
        }
        break;

      default:
        state=0;
        //Serial.print(rc);
        break;
    }
  }
  timerAlarmEnable(timer);
}



/*-----------------------------------------------------------------------------------
/////////////////---- SETUP ----/////////////////////////////////////////////////////
------------------------------------------------------------------------------------*/
void setup() {
  uint16_t calibrationData[5];
  uint8_t calDataOK = 0;
  pinMode(Enc_A, INPUT_PULLUP);     //
  pinMode(Enc_B, INPUT_PULLUP);     //
  pinMode(JSButton, INPUT_PULLUP);  //
  pinMode(ZEROBUT, INPUT_PULLUP);   //
  pinMode(ZEROBUTLED, OUTPUT);   //
  digitalWrite(ZEROBUTLED,LOW);

  Serial2.begin(S2SPEED, SERIAL_8N1, RXD2, TXD2);
  Serial.begin(115200); /* prepare for possible serial debug */


  //String LVGL_Arduino = "Hello Arduino! ";
  //LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  //Serial.println(LVGL_Arduino);
  //Serial.println("I am LVGL_Arduino");
  //  Serial.println();

  if (!digitalRead(ZEROBUT) && !digitalRead(JSButton)){
    OTA=true;
    Serial.println("Beide Knöpfe gedrückt");
  }
  if (digitalRead(ZEROBUT) && !digitalRead(JSButton)){
    Serial.println("Nur Joystickknopf gedrückt: Touch neu kalibrieren");
    recalibrateTouch = true;
  }
  readJSCalibrationfile();






  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  tft.begin();        /* TFT init */
  tft.setRotation(0); /* Landscape orientation, flipped */

// check file system
  if (!SPIFFS.begin()) {
    Serial.println("formating file system");

    SPIFFS.format();
    SPIFFS.begin();
  }

  // check if calibration file exists
  if (SPIFFS.exists(TOUCHCALIBRATION_FILE)) {
    fs::File f = SPIFFS.open(TOUCHCALIBRATION_FILE, "r");
    if (f) {
      if (f.readBytes((char *)calibrationData, 14) == 14)
        calDataOK = 1;
      f.close();
      Serial.println("Touchkalibrierungsdatei gelesen");
    }
  }
  if (calDataOK) {
    // calibration data valid
    tft.setTouch(calibrationData);
    Serial.println("Touchkalibrierungsdatei OK");
  } else {
    // data not valid. recalibrate
    Serial.print("Touchkalibrierung...");
    tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
    Serial.println(" ausgeführt");
    // store data
    fs::File f = SPIFFS.open(TOUCHCALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calibrationData, 14);
      f.close();
    }
  }
  if(recalibrateTouch){
    Serial.print("Touchkalibrierung...");
    tft.calibrateTouch(calibrationData, TFT_WHITE, TFT_RED, 15);
    Serial.println(" ausgeführt");
    // store data
    fs::File f = SPIFFS.open(TOUCHCALIBRATION_FILE, "w");
    if (f) {
      f.write((const unsigned char *)calibrationData, 14);
      f.close();
    }
  }

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * screenHeight / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = screenWidth;
  disp_drv.ver_res = screenHeight;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  ui_init();

  lv_obj_set_style_bg_color(ui_Keyboard1, lv_color_hex(0xFFBDBD), LV_PART_ITEMS | LV_STATE_DEFAULT);
  lv_obj_set_style_text_color(ui_Keyboard1, lv_color_hex(0x000000), LV_PART_ITEMS | LV_STATE_DEFAULT);
  lv_obj_set_style_text_opa(ui_Keyboard1, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
  lv_obj_set_style_text_align(ui_Keyboard1, LV_TEXT_ALIGN_CENTER, LV_PART_ITEMS | LV_STATE_DEFAULT);
  lv_obj_set_style_text_font(ui_Keyboard1, &lv_font_montserrat_24, LV_PART_ITEMS | LV_STATE_DEFAULT);


  lv_keyboard_set_map(ui_Keyboard1, LV_KEYBOARD_MODE_USER_1, kb_map, kb_ctrl);
  lv_keyboard_set_mode(ui_Keyboard1, LV_KEYBOARD_MODE_USER_1);




  attachInterrupt(digitalPinToInterrupt(Enc_A), doEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(ZEROBUT), setactivetoZero, FALLING);
  attachInterrupt(digitalPinToInterrupt(JSButton), toggleABCaktiv, FALLING);


  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, Interuptintervall * 1000, true);
  timerAlarmEnable(timer);

  if(!OTA){
    Serial.println("warte auf OK");
    sendeSerial("\r", true);
    Serial.println("OK!");
  }

  Serial2.println("$10=0");
  Serial2.println("$Report/Interval=100");
  Serial2.println("?");
  Serial.println("Setup done");

  lv_bar_set_range(ui_BarAchsentimeout, 0, achse_active_timeout);
}
/*-----------------------------------------------------------------------------------
/////////////////---- Main LOOP ----///////////////////////////////////////////
-----------------------------------------------------------------------------------*/
void loop() {

  lv_timer_handler(); /* let the GUI do its work */
  empfangeSerial();
  //delay(5);
  //Check to see if anything is available in the serial receive buffer
  if (doJoystickauslesen && Joystickenabled) {
    Joystickauslesen();
    doJoystickauslesen = false;
  }

  if (setRoller) {
    strcpy(Temp_String,"");
    for (int i = 0; i < NumberofDateien; i++) {
      strcat(Temp_String, Dateien[i]);
      strcat(Temp_String, nl);
    }
    lv_roller_set_options(ui_Roller, Temp_String, LV_ROLLER_MODE_NORMAL);
    setRoller = false;
  }


  if (achse_aktiv_counter) {
    timeout = achse_active_timeout - (millis() - achse_aktiv_timer);
    if (timeout < 0) {
      achse_aktiv_counter = false;
      //Serial.println("achse nicht mehr aktiv");
      _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
      digitalWrite(ZEROBUTLED,LOW);
    } else {
      if (letzte_aktive_achse != aktive_achse) {
       // digitalWrite(ZEROBUTLED,HIGH);
        timeout_old = timeout;
        switch (aktive_achse) {
          case 'X':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            break;
          case 'Y':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            break;
          case 'Z':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            break;
          case 'A':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            break;
          case 'B':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            break;
          case 'C':
            _ui_state_modify(ui_ButtonX, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonY, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonZ, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonA, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonB, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
            _ui_state_modify(ui_ButtonC, LV_STATE_CHECKED, _UI_MODIFY_STATE_ADD);
            break;
          default:
            break;
        }
        letzte_aktive_achse = aktive_achse;
      }
      if (abs(timeout_old - timeout) > 10) {
        lv_bar_set_value(ui_BarAchsentimeout, timeout, LV_ANIM_OFF);
        digitalWrite(ZEROBUTLED,HIGH);
        timeout_old = timeout;
      }
    }
  }
  //*/

  if (jobrunning) {
    if (jobdone) {
      jobrunning = false;
      if(beendeJob){
        itoa(Wiederholungen2do,Temp_String,10);
        lv_label_set_text(ui_LabelWiederholungen,Temp_String);
        firstrun=true;
        timerAlarmEnable(timer);
        Joystickenabled=true;
        _ui_state_modify(ui_ButtonRun, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
        beendeJob=false;
      } else {
        Wiederholungendone+=1;
        if(repeat){
          itoa(Wiederholungendone,Temp_String,10);
          lv_label_set_text(ui_LabelWiederholungen,Temp_String);          
          runselectedFile();
        }else{
          itoa((Wiederholungen2do-Wiederholungendone),Temp_String,10);
          lv_label_set_text(ui_LabelWiederholungen,Temp_String);
          if((Wiederholungen2do-Wiederholungendone)>0){
            runselectedFile();
          } else {
            timerAlarmEnable(timer);
            Joystickenabled=true;
            firstrun=true;
            itoa((Wiederholungen2do),Temp_String,10);
            lv_label_set_text(ui_LabelWiederholungen,Temp_String);
            _ui_state_modify(ui_ButtonRun, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
         }
        } 
      }
    }
  }

  if (aktiveachsenullen && achse_aktiv_counter) {
    cmd = "G10 L20 P0 ";
    cmd += aktive_achse;
    cmd += "0\r";
    sendeSerial(cmd, true);
    aktiveachsenullen = false;
  } else {
    aktiveachsenullen = false;
  }  //*/

  if (DebounceTimer_aktiv) {
    if ((millis() - DebounceTimer) > ButtonDebounce) {
      attachInterrupt(digitalPinToInterrupt(JSButton), toggleABCaktiv, FALLING);
      DebounceTimer_aktiv = false;
    }
  }


  if (sende_neuen_Encoder_jogbefehl && achse_aktiv_counter) {
    sendeEncoderJog(Encoder_pos_2send);
    achse_aktiv_timer = millis();
    sende_neuen_Encoder_jogbefehl = false;
  }
  //*
  if (sende_neuen_Joystick_jogbefehl) {
    sendeJoystickJog(JSX_pos, JSY_pos, JSZ_pos);
    sende_neuen_Joystick_jogbefehl = false;
  }

  while (Serial.available() > 0) {
    //Create a place to hold the incoming message
    static char message[MAX_MESSAGE_LENGTH];
    static unsigned int message_pos = 0;

    //Read the next available byte in the serial receive buffer
    char inByte = Serial.read();

    //Message coming in (check not terminating character) and guard for over message size
    if (inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1)) {
      //Add the incoming byte to our message
      message[message_pos] = inByte;
      message_pos++;
    }
    //Full message received...
    else {
      //Add null character to string
      message[message_pos] = '\0';

      //Print the message (or do other things)
      Serial2.print(message);
      Serial2.print('\r');

      //Reset for the next message
      message_pos = 0;
    }
  }
  //*/
}