
#include "ZeroLASER.h"
//#include "ftoa.h"

#include "Analog.h"



//====================================================================
// Global object for FT800 Implementation
//====================================================================
FT800IMPL_SPI FTImpl(FT_CS_PIN, FT_PDN_PIN, FT_INT_PIN);

//====================================================================
// Defines for compatibility with FTDI Chip's EVE Screen Designer.
// This allows for the direct import of the Display List (.dl) files
// exported from GUI Designer tool.
//====================================================================
#define cmd_dlstart         FTImpl.DLStart
#define CLEAR_COLOR_RGB     FTImpl.ClearColorRGB
#define CLEAR               FTImpl.Clear
#define SAVE_CONTEXT        FTImpl.SaveContext
#define COLOR_RGB           FTImpl.ColorRGB
#define LINE_WIDTH          FTImpl.LineWidth
#define RECTS               FT_RECTS
#define BEGIN               FTImpl.Begin
#define VERTEX2F            FTImpl.Vertex2f
#define END                 FTImpl.End
#define RESTORE_CONTEXT     FTImpl.RestoreContext
#define cmd_text            FTImpl.Cmd_Text
#ifdef DISPLAY
#undef DISPLAY
#endif
#define DISPLAY             FTImpl.DLEnd
#define cmd_swap            FTImpl.Finish
#define cmd_fgcolor         FTImpl.Cmd_FGColor
#define cmd_gradcolor       FTImpl.Cmd_GradColor
#define cmd_button          FTImpl.Cmd_Button

#define cmd_gradient        FTImpl.Cmd_Gradient
#define cmd_spinner         FTImpl.Cmd_Spinner
#define cmd_number          FTImpl.Cmd_Number
#define cmd_dial            FTImpl.Cmd_Dial
#define cmd_bgcolor         FTImpl.Cmd_BGColor
#define cmd_toggle          FTImpl.Cmd_Toggle
#define POINT_SIZE          FTImpl.PointSize


#define BITMAPS             FT_BITMAPS
#define POINTS              FT_POINTS
#define LINES               FT_LINES
#define LINE_STRIP          FT_LINE_STRIP
#define LINE_STRIP_R        FT_EDGE_STRIP_R
#define LINE_STRIP_L        FT_EDGE_STRIP_L
#define LINE_STRIP_A        FT_EDGE_STRIP_A
#define LINE_STRIP_B        FT_EDGE_STRIP_B
#define RECTS               FT_RECTS

static bool displayCalibrated = false;
static bool displayAttached;
static Screen displayScreen;
static bool  gantryXmin;
static bool  gantryYmin;
static bool  interlockTripped;
static bool  operatorESTOP;
static bool  powerModulePresent;
static bool  displayCalbrated;
static bool  exaustFan;
static bool  airAssist;
static bool  waterCooling;
static char  gantryXloc[8];
static char  gantryYloc[8];
static char  gantryStepSize[8];
static char  laserCurrent[8];
static char  laserPower[8];
static char  waterTempIn[8];
static char  waterTempOut[8];
static float realXYloc[2];

static bool  ManualMoveEnabled;
static bool  LaserCalEnabled;

static sTagXY sTagxy;

extern float JogStepSize;
extern void floatToAscii(float n, char *res, int fractional);


bool DisplayInit() {

  displayCalibrated   = false;
  displayAttached     = false;
  displayScreen       = SCRN_BOOTING;
  //  displayScreen       = SCRN_MANUAL_MOVE;
  gantryXmin          = false;
  gantryYmin          = false;
  interlockTripped    = false;
  operatorESTOP       = false;
  powerModulePresent  = false;
  displayCalbrated    = false;
  exaustFan           = false;
  airAssist           = false;
  waterCooling        = false;

  ManualMoveEnabled   = false;
  LaserCalEnabled     = false;



  DisplayUpdateLocation( 0.0, 0.0 );
  DisplayUpdateLaserPower( 0.0, 0.0 );
  DisplayUpdateWaterCooling( 0.0, 0.0 );

  //----------------------------------
  // Initialize FT800 library
  //----------------------------------
  uint32_t chipid = 0;

  FTImpl.Init(FT_DISPLAY_RESOLUTION);
  delay(20);
  chipid = FTImpl.Read32(FT_ROM_CHIPID);

  //----------------------------------
  // Identify the chip
  //----------------------------------
  if (FT800_CHIPID != chipid) {
#ifdef DISPLAY_DEBUG
    DISPLAY_SERIAL_DEVICE.print("Error in chip id read ");
    DISPLAY_SERIAL_DEVICE.println(chipid, HEX);
#endif
    displayAttached = false;
    return false;
  }

  //----------------------------------
  // Set the Display & audio pins
  //----------------------------------
  FTImpl.SetDisplayEnablePin(FT_DISPENABLE_PIN);
  FTImpl.SetAudioEnablePin(FT_AUDIOENABLE_PIN);
  FTImpl.DisplayOn();
  FTImpl.AudioOn();
#ifdef DISPLAY_DEBUG
  DISPLAY_SERIAL_DEVICE.print(F("Display Enabled ["));
  DISPLAY_SERIAL_DEVICE.print(FT_DISPLAYWIDTH);
  DISPLAY_SERIAL_DEVICE.print("x");
  DISPLAY_SERIAL_DEVICE.print(FT_DISPLAYHEIGHT);
  DISPLAY_SERIAL_DEVICE.println("]");
#endif
#ifdef DISPLAY_CUSTOM_FONTS
  CustomFonts();
#endif
  displayAttached = true;
  return true;
}

//======================================================
// 'Fadein' effect by changing the display PWM from
// 0 to 100 and finally 128
//======================================================
void DisplayFadeIn() {
  int32_t i;
  for (i = 0; i <= 100 ; i += 3) {
    FTImpl.Write(REG_PWM_DUTY, i);
    delay(2); //sleep for 2 ms
  }
  // Finally make the PWM 100%
  i = 128;
  FTImpl.Write(REG_PWM_DUTY, i);
}

//======================================================
// 'fadeout' by changing the display PWM from 100 to 0
//======================================================
void DisplayFadeOut() {
  int32_t i;
  for (i = 100; i >= 0; i -= 3) {
    FTImpl.Write(REG_PWM_DUTY, i);
    delay(2); //sleep for 2 ms
  }
}

//======================================================
// Called by mainline code.
//======================================================
//uint32_t DisplayGetTags() {
//  FTImpl.GetTagXY(sTagxy);
////  FTImpl.ClearTag(sTagxy.tag);
//  return sTagxy.tag;
//}

uint16_t StepSizeDialAngle = 0;

void DisplaySetStepSizeDial( uint16_t val ) {
  StepSizeDialAngle = val;
}


static uint8_t sk = 0;

uint8_t DisplayButtonPressed() {

  if((sk == BUTTON_MOVE_HOME)    || \
     (sk == BUTTON_MOVE_PLUS_X)  || \
     (sk == BUTTON_MOVE_MINUS_X) || \
     (sk == BUTTON_MOVE_PLUS_Y)  || \
     (sk == BUTTON_MOVE_MINUS_Y)) {
      if( blocks_queued() ) {
        FTImpl.PlaySound(255, FT_WARBLE);
        return 0;
      }   
  }
  return sk;
}

/********API to return the assigned TAG value when penup,for the primitives/widgets******/
uint8_t DisplayReadButtons()
{
  static uint8_t Read_tag = 0, temp_tag = 0, ret_tag = 0;
  static uint32_t KeyTimer = 0;
  Read_tag = FTImpl.Read(REG_TOUCH_TAG);
  ret_tag = 0;

  if( millis() < KeyTimer ) {
    return 0;
  }
  
  if (Read_tag != 0 && temp_tag != Read_tag) {
    temp_tag = Read_tag;                     // Load the Read tag to temp variable
    ret_tag =  temp_tag;
    sk = Read_tag;
    FTImpl.PlaySound(255, FT_SWITCH);
  }
  if (Read_tag == 0) {
    KeyTimer = millis() + 250;
    //     ret_tag =  temp_tag;
    temp_tag = 0;
    ret_tag =  0;
    sk = 0;
  }
  return ret_tag;
}

uint16_t DisplayReadDial( uint8_t tag )
{
  uint8_t  tagval = 0;
  uint32_t tracker = FTImpl.Read32(REG_TRACKER);
  uint16_t angleval = 0;

  tagval = tracker & 0xff;

  if (0 != tagval) {
    if (tag == tagval) {
      angleval = tracker >> 16;
    }
  }
  return angleval;
}

//======================================================
// Screen Change from mainline code...
//======================================================
void DisplaySetScreen( Screen scrn ) {
  displayScreen = scrn;
}

//======================================================
// Called by mainline code.
//======================================================
bool DisplayAttached() {
  return displayAttached;
}

//======================================================
// Called from mainline code to keep UI up to date.
//======================================================
void DisplayUpdateLimitSwitches( bool xmin, bool ymin, bool interlock ) {
  gantryXmin        = xmin;
  gantryYmin        = ymin;
  interlockTripped  = interlock;
}


// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
  int i = 0, j = len - 1, temp;
  while (i < j) {
    temp = str[i];
    str[i] = str[j];
    str[j] = temp;
    i++; j--;
  }
}

// Converts a given integer x to string str[].  d is the number
// of digits required in output. If d is more than the number
// of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d, bool spacepad)
{
  int i = 0;
  while (x) {
    str[i++] = (x % 10) + '0';
    x = x / 10;
  }

  // If number of digits required is more, then
  // add 0s at the beginning
  while (i < d) {
    str[i++] = ((spacepad == true) ? ' ' : '0');
  }

  if (str[0] == ' ') str[0] = '0';

  reverse(str, i);
  str[i] = '\0';
  return i;
}

// Converts a floating point number to string.
void floatToAscii(float n, char *res, int mantissa, int fractional)
{
  // Extract integer part
  int ipart = (int)n;

  // Extract floating part
  float fpart = n - (float)ipart;

  // convert integer part to string
  //  int i = intToStr(ipart, res, 0);
  int i = intToStr(ipart, res, mantissa, true);

  // check for display option after point
  if (fractional != 0) {
    res[i] = '.';  // add dot
    // Get the value of fraction part upto given no.
    // of points after dot. The third parameter is needed
    // to handle cases like 233.007
    fpart = fpart * pow(10, fractional);
    intToStr((int)fpart, res + i + 1, fractional, false);
  }
}
//======================================================
// Called from mainline code to keep UI up to date.
//======================================================
void DisplayUpdateLocation( float xloc, float yloc ) {
  realXYloc[0] = xloc;
  realXYloc[1] = yloc;  
  floatToAscii(xloc, gantryXloc, 3, 3);
  floatToAscii(yloc, gantryYloc, 3, 3);
  floatToAscii(JogStepSize, gantryStepSize, 3, 3);
}


//======================================================
// Called from mainline code to keep UI up to date.
//======================================================
void DisplayUpdateLaserPower( float mA, float pwm ) {
  floatToAscii(mA, laserCurrent, 2, 2);
  floatToAscii(pwm, laserPower,  2, 0);
}

//======================================================
// Called from mainline code to keep UI up to date.
//======================================================
void DisplayUpdateWaterCooling( float in, float out ) {
  floatToAscii(in, waterTempIn, 2, 1);
  floatToAscii(out, waterTempOut, 2, 1);

  //SerialUSB.print("WaterTemp: ");
  //SerialUSB.print( out );
  //SerialUSB.print(" [");
  //SerialUSB.print( waterTempOut );
  //SerialUSB.println("]");

}



bool canMoveMinusX() {
  return (((realXYloc[0] - JogStepSize) > 0.0 ) ? true : false);
}

bool canMoveMinusY() {
  return (((realXYloc[1] - JogStepSize) > 0.0 ) ? true : false);
}

bool canMoveHome() {
  return (( canMoveMinusX() && canMoveMinusY() ) ? true : false);
}


#ifdef DISPLAY_CUSTOM_FONTS
//======================================================
// Custom Fonts setup
//======================================================
static void DisplayCustomFonts() {
  uint32_t fontaddr = FT_RAM_G;
  uint16_t blocklen = FT_FONT_TABLE_SIZE;

  //----------------------------------------------------------------------------------------------------------
  // Copy the header from starting of the array into FT_RAM_G/
  //----------------------------------------------------------------------------------------------------------
  FTImpl.Writefromflash(fontaddr, fontDigital, FT_FONT_TABLE_SIZE);

  //----------------------------------------------------------------------------------------------------------
  // update the address of the font data - last 4 bytes of the index table contains the font data address
  //----------------------------------------------------------------------------------------------------------
  FTImpl.Write32(fontaddr + FT_FONT_TABLE_SIZE - 4, 1024); //FT_RAM_G + 1024 is the starting address of the font raw data, out of which first 32 characters are dummy

  //----------------------------------------------------------------------------------------------------------
  // download the custom font data - note that first 32 characters in ascii
  // table are control commands and hence need to take care of offset. Next
  // download the data at location 32*FNT_DIGITAL7_STRIDE*FNT_DIGITAL7_HEIGHT - skip the first 32 characters
  // each character is FNT_DIGITAL7_STRIDExFNT_DIGITAL7_HEIGHT bytes
  //----------------------------------------------------------------------------------------------------------
  fontaddr = (FT_RAM_G + 1024 + 32 * FNT_DIGITAL_STRIDE * FNT_DIGITAL_HEIGHT);//make sure space is left at the starting of the buffer for first 32 characters
  FTImpl.Writefromflash( fontaddr, &fontDigital[FT_FONT_TABLE_SIZE], 1L * (fontDigital_Size - FT_FONT_TABLE_SIZE));
}
#endif

//=========================================================================
// The Calibrate function will wait untill user presses all the three dots.
// Only way to come out of this api is to reset the coprocessor bit.
//=========================================================================
void DisplayCalibrateTouch() {
  //-------------------------------------------------------------------------
  // Construct the display list with grey as background color,
  // informative string "Please Tap on the dot" followed by
  // inbuilt calibration command
  //-------------------------------------------------------------------------
  FTImpl.DLStart();
  FTImpl.ClearColorRGB(64, 64, 64);
  FTImpl.Clear(1, 1, 1);
  FTImpl.ColorRGB(0xff, 0xff, 0xff);
  FTImpl.Cmd_Text((FT_DISPLAYWIDTH / 2), (FT_DISPLAYHEIGHT / 2), 27, FT_OPT_CENTER, "Please Tap on the dot");
  FTImpl.Cmd_Calibrate(0);
  //-------------------------------------------------------------------------
  // Wait for the completion of calibration - either finish can be used for
  // flush and check can be used
  //-------------------------------------------------------------------------
  FTImpl.Finish();
  displayCalibrated = true;
}

void clearButtonTags() {
  FTImpl.ClearTag(BUTTON_BOOT_CONTINUE);
  FTImpl.ClearTag(BUTTON_WARN_OK);
  FTImpl.ClearTag(BUTTON_MAIN_MENU);
  FTImpl.ClearTag(BUTTON_MANUAL_MOVE);
  FTImpl.ClearTag(BUTTON_LASER_CAL);
  FTImpl.ClearTag(BUTTON_MONITOR);
  FTImpl.ClearTag(BUTTON_MOVE_HOME);
  FTImpl.ClearTag(BUTTON_MOVE_PLUS_X);
  FTImpl.ClearTag(BUTTON_MOVE_MINUS_X);
  FTImpl.ClearTag(BUTTON_MOVE_PLUS_Y);
  FTImpl.ClearTag(BUTTON_MOVE_MINUS_Y);
  FTImpl.ClearTag(BUTTON_LASER_PULSE);
  FTImpl.ClearTag(BUTTON_EMG_STOP);
}





void DisplayScreen() {

  // clearButtonTags();

  switch ( displayScreen ) {
    case SCRN_BOOTING:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("BootScreen");
#endif
      //===BootScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      //      SAVE_CONTEXT();
      cmd_gradient(226, 16, 0x00007f, 226, 186, 0x003c78);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(235, 42, 31, 1536, "ZeroLASER");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(231, 76, 22, 1536, "Version 1.0a");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(234, 251, 27, 1536, "Initializing");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x003870);
      cmd_gradcolor(0xffffff);

      //      COLOR_RGB(255,255,255);
      //      cmd_spinner(233,141,0,0);

      FTImpl.Tag(BUTTON_BOOT_CONTINUE);
      cmd_button(151, 190, 160, 42, 30, ((DisplayButtonPressed() == BUTTON_BOOT_CONTINUE) ? FT_OPT_FLAT : 0), "Continue");


      //      RESTORE_CONTEXT();
      //     SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_spinner(233, 141, 0, 0);
      //     RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    case SCRN_WARNING:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("WarningScreen");
#endif
      //===WarningScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(176, 1728);
      VERTEX2F(7472, 4240);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 0, 0);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(176, 96);
      VERTEX2F(7472, 1648);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(283, 57, 31, 1536, "DANGER");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(36);
      COLOR_RGB(255, 255, 0);
      BEGIN(LINES);
      VERTEX2F(1600, 1360);
      VERTEX2F(2192, 288);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(36);
      COLOR_RGB(255, 255, 0);
      BEGIN(LINES);
      VERTEX2F(2224, 304);
      VERTEX2F(2784, 1344);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(36);
      COLOR_RGB(255, 255, 0);
      BEGIN(LINES);
      VERTEX2F(1616, 1376);
      VERTEX2F(2752, 1376);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 0);
      cmd_text(138, 61, 31, 1536, "!");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(234, 131, 30, 1536, "INVISIBLE LASER RADIATION");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(291, 159, 28, 1536, "AVOID EYE OR SKIN EXPOSURE");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 0);
      cmd_fgcolor(0xff0000);
      cmd_gradcolor(0xffffff);

      //      FTImpl.ClearTag((uint8_t)BUTTON_BOOT_CONTINUE);
      FTImpl.Tag(BUTTON_WARN_OK);
      cmd_button(194, 224, 88, 38, 30, ((DisplayButtonPressed() == BUTTON_WARN_OK) ? FT_OPT_FLAT : 0), "OK");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(286, 178, 26, 1536, "TO DIRECT OR SCATTERED RADIATION");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(288, 206, 28, 1536, "CARBON DIOXIDE LASER");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(375, 243, 20, 1536, "40W MAX OUTPUT at 10.6uM");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(103, 244, 20, 1536, "CLASS IV LASER PRODUCT");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(22);
      COLOR_RGB(255, 0, 0);
      BEGIN(LINES);
      VERTEX2F(1328, 3056);
      VERTEX2F(7248, 3056);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      POINT_SIZE(200);
      COLOR_RGB(255, 0, 0);
      BEGIN(POINTS);
      VERTEX2F(1312, 3056);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(22);
      COLOR_RGB(255, 0, 0);
      BEGIN(LINES);
      VERTEX2F(768, 2592);
      VERTEX2F(1904, 3504);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(22);
      COLOR_RGB(255, 0, 0);
      BEGIN(LINES);
      VERTEX2F(736, 3488);
      VERTEX2F(2000, 2640);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      LINE_WIDTH(22);
      COLOR_RGB(255, 0, 0);
      BEGIN(LINES);
      VERTEX2F(1328, 2480);
      VERTEX2F(1344, 3616);
      END();
      RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    case SCRN_MAIN_MENU:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("MainMenuScreen");
#endif
      //===MainMenuScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      SAVE_CONTEXT();
      cmd_gradient(238, 32, 0x244b7f, 238, 10, 0x3f8ce5);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 57);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(0, 704);
      VERTEX2F(7696, 4320);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(242, 22, 30, 1536, "Main Menu");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 221);
      cmd_text(240, 20, 30, 1536, "Main Menu");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(230, 230, 230);
      cmd_fgcolor(0x003870);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MANUAL_MOVE);
      cmd_button(157, 66, 175, 50, 28, ((DisplayButtonPressed() == BUTTON_MANUAL_MOVE) ? FT_OPT_FLAT : 0), "Manual Move");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(230, 230, 230);
      cmd_fgcolor(0x003870);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_LASER_CAL);
      cmd_button(156, 131, 175, 50, 28, ((DisplayButtonPressed() == BUTTON_LASER_CAL) ? FT_OPT_FLAT : 0), "Laser Cal");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(230, 230, 230);
      cmd_fgcolor(0x003870);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MONITOR);
      cmd_button(156, 197, 175, 50, 28, ((DisplayButtonPressed() == BUTTON_MONITOR) ? FT_OPT_FLAT : 0), "Monitor");

      RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    case SCRN_MANUAL_MOVE:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("ManualMoveScreen");
#endif
      //===ManualMoveScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      SAVE_CONTEXT();
      cmd_gradient(239, 32, 0x244b7f, 239, 10, 0x3f8ce5);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 57);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(0, 704);
      VERTEX2F(7696, 4320);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(243, 22, 30, 1536, "Manual Move");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 221);
      cmd_text(241, 20, 30, 1536, "Manual Move");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x002448);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MAIN_MENU);
      cmd_button(13, 5, 44, 31, 31, ((DisplayButtonPressed() == BUTTON_MAIN_MENU) ? FT_OPT_FLAT : 0), "<");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(4656, 1024);
      VERTEX2F(6496, 1456);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_number(317,78,30,1536,Xloc_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(377,78,30,1536,Xloc_Fraction);
      cmd_text(349, 75, 30, 1536, gantryXloc);

      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_text(349,75,31, 1536, ".");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(433, 77, 28, 1536, "mm");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(268, 79, 30, 1536, "X");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      POINT_SIZE(144);
      //      COLOR_RGB(255,0,0);
      if ( X_MIN_HIT() ) {
        COLOR_RGB(255, 0, 0);
      } else {
        COLOR_RGB(85, 0, 0);
      }

      BEGIN(POINTS);
      VERTEX2F(3888, 1248);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(4672, 1872);
      VERTEX2F(6512, 2304);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_number(318,131,30,1536,Yloc_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(378,131,30,1536,Yloc_Fraction);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_text(350,128,31, 1536, ".");
      cmd_text(350, 128, 30, 1536, gantryYloc);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(434, 130, 28, 1536, "mm");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(269, 132, 30, 1536, "Y");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      POINT_SIZE(144);
      //      COLOR_RGB(85,0,0);
      if ( Y_MIN_HIT() ) {
        COLOR_RGB(255, 0, 0);
      } else {
        COLOR_RGB(85, 0, 0);
      }
      BEGIN(POINTS);
      VERTEX2F(3888, 2096);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 32, 64);      

      cmd_fgcolor( (( canMoveMinusX() == true ) ? 0xFFFFFFFF : 0x005bb6) );
      cmd_gradcolor(0xffffff);
      
      if( canMoveMinusX() == true ) FTImpl.Tag(BUTTON_MOVE_MINUS_X);
      cmd_button(9, 127, 60, 60, 31, ((DisplayButtonPressed() == BUTTON_MOVE_MINUS_X) ? FT_OPT_FLAT : 0), "-X");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 32, 64);
      cmd_fgcolor( (( canMoveHome() == true ) ? 0xFFFFFFFF : 0x005bb6) );
      cmd_gradcolor(0xffffff);

      if( canMoveHome() == true ) FTImpl.Tag(BUTTON_MOVE_HOME);
      cmd_button(81, 126, 60, 60, 31, ((DisplayButtonPressed() == BUTTON_MOVE_HOME) ? FT_OPT_FLAT : 0), "H");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 32, 64);
      cmd_fgcolor(0x005bb6);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MOVE_PLUS_X);
      cmd_button(153, 126, 60, 60, 31, ((DisplayButtonPressed() == BUTTON_MOVE_PLUS_X) ? FT_OPT_FLAT : 0), "+X");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 32, 64);
      cmd_fgcolor( (( canMoveMinusY() == true ) ? 0xFFFFFFFF : 0x005bb6) );
      cmd_gradcolor(0xffffff);

      if( canMoveMinusY() == true ) FTImpl.Tag(BUTTON_MOVE_MINUS_Y);
      cmd_button(81, 56, 60, 60, 31, ((DisplayButtonPressed() == BUTTON_MOVE_MINUS_Y) ? FT_OPT_FLAT : 0), "-Y");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 32, 64);
      cmd_fgcolor(0x005bb6);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MOVE_PLUS_Y);
      cmd_button(81, 195, 60, 60, 31, ((DisplayButtonPressed() == BUTTON_MOVE_PLUS_Y) ? FT_OPT_FLAT : 0), "+Y");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 49, 98);
      cmd_fgcolor(0x005bb6);

      //      cmd_dial(276,213,38,0,10000);
      FTImpl.Tag(DIAL_STEPSIZE);
      cmd_dial(276, 213, 38, 0, StepSizeDialAngle );
      FTImpl.Cmd_Track(276, 213, 1, 1, DIAL_STEPSIZE);


      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(394, 175, 27, 1536, "mm/Step");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5392, 3168);
      VERTEX2F(7232, 3648);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_number(362,214,30,1536,123);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(422,214,30,1536,123);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_text(394,211,31, 1536, ".");
      cmd_text(394, 211, 30, 1536, gantryStepSize);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x0068d0);
      cmd_bgcolor(0x002040);
      
      FTImpl.Tag(TOGGLE_MOVE_ENABLE);
      cmd_toggle(369, 14, 92, 28, 256, ((ManualMoveEnabled) ? 65535: 0), "Enabled?Disabled");

      RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    case SCRN_MANUAL_LASER:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("ManualLaserScreen");
#endif
      //===ManualLaserScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      SAVE_CONTEXT();
      cmd_gradient(238, 32, 0x244b7f, 238, 10, 0x3f8ce5);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 57);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(-16, 704);
      VERTEX2F(7680, 4320);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(242, 22, 30, 1536, "Laser Cal");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 221);
      cmd_text(240, 20, 30, 1536, "Laser Cal");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x002448);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MAIN_MENU);
      cmd_button(12, 5, 44, 31, 31, ((DisplayButtonPressed() == BUTTON_MAIN_MENU) ? FT_OPT_FLAT : 0), "<");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5168, 3552);
      VERTEX2F(6528, 3968);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_text(372,233,31, 1536, ".");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(347,235,30,1536,20);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(390,235,30,1536,0);
      cmd_text(372, 233, 30, 1536, waterTempOut);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(427, 236, 29, 1536, "C");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(250, 235, 30, 1536, "Coolant");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5152, 2720);
      VERTEX2F(6512, 3136);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_text(371,181,31, 1536, ".");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(346,183,30,1536,20);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(389,183,30,1536,0);
      cmd_text(371, 181, 30, 1536, laserCurrent);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(435, 183, 29, 1536, "mA");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(262, 183, 30, 1536, "Power");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_fgcolor(0xf9c801);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_LASER_PULSE);
      cmd_button(30, 152, 132, 93, 30, ((DisplayButtonPressed() == BUTTON_LASER_PULSE) ? FT_OPT_FLAT : 0), "PULSE");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(85, 170, 0);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(592, 1056);
      VERTEX2F(2496, 2096);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(96, 86, 29, 1536, "Safety");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(97, 111, 29, 1536, "Interlock");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(229, 225, 205);
      cmd_text(96, 110, 29, 1536, "Interlock");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(229, 225, 205);
      cmd_text(95, 85, 29, 1536, "Safety");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x0068d0);
      cmd_bgcolor(0x002040);

      FTImpl.Tag(TOGGLE_LASER_ENABLE);
      cmd_toggle(369, 14, 92, 28, 0, ((LaserCalEnabled) ? 65535: 0), "Enabled?Disabled");
      
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 56, 112);
      cmd_fgcolor(0x005bb6);
      cmd_dial(238, 111, 38, 0, 10000);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5136, 1600);
      VERTEX2F(6496, 2016);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_text(370,111,31, 1536, ".");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(345,113,30,1536,20);
      cmd_text(370, 111, 30, 1536, laserPower);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_number(388, 113, 30, 1536, 0);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(425, 114, 29, 1536, "%");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(365, 73, 29, 1536, "Duty Cycle");
      RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    case SCRN_MONITOR:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("MonitorScreen");
#endif
      //===MonitorScreen===
      cmd_dlstart();
      CLEAR_COLOR_RGB(0, 0, 0);
      CLEAR(1, 1, 1);
      SAVE_CONTEXT();
      cmd_gradient(238, 32, 0x244b7f, 238, 10, 0x3f8ce5);
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 57);
      LINE_WIDTH(16);
      BEGIN(RECTS);
      VERTEX2F(-16, 704);
      VERTEX2F(7680, 4320);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(242, 22, 30, 1536, "Monitor");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 221);
      cmd_text(240, 20, 30, 1536, "Monitor");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0x002448);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_MAIN_MENU);
      cmd_button(12, 5, 44, 31, 31, ((DisplayButtonPressed() == BUTTON_MAIN_MENU) ? FT_OPT_FLAT : 0), "<");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(4656, 1024);
      VERTEX2F(6496, 1456);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_number(317,78,30,1536,Xloc_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(377,78,30,1536,Xloc_Fraction);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_text(349,75,31, 1536, ".");
      cmd_text(349, 75, 30, 1536, gantryXloc);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(433, 77, 28, 1536, "mm");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(268, 79, 30, 1536, "X");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      POINT_SIZE(144);

      if ( X_MIN_HIT() ) {
        COLOR_RGB(255, 0, 0);
      } else {
        COLOR_RGB(85, 0, 0);
      }
      //COLOR_RGB(255,0,0);

      BEGIN(POINTS);
      VERTEX2F(3888, 1248);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(4672, 1872);
      VERTEX2F(6512, 2304);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_number(318,131,30,1536,Yloc_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(378,131,30,1536,Yloc_Fraction);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_text(350,128,31, 1536, ".");
      cmd_text(350, 128, 30, 1536, gantryYloc);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(434, 130, 28, 1536, "mm");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(269, 132, 30, 1536, "Y");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      POINT_SIZE(144);

      if ( Y_MIN_HIT() ) {
        COLOR_RGB(255, 0, 0);
      } else {
        COLOR_RGB(85, 0, 0);
      }

      //      COLOR_RGB(85,0,0);
      BEGIN(POINTS);
      VERTEX2F(3888, 2096);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5168, 3552);
      VERTEX2F(6528, 3968);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_text(372,233,31, 1536, ".");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(347,235,30,1536,temp_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(390,235,30,1536,temp_Fraction);
      cmd_text(372, 233, 30, 1536, waterTempOut);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(427, 236, 29, 1536, "C");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(250, 235, 30, 1536, "Coolant");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(28, 176, 189);
      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(5152, 2720);
      VERTEX2F(6512, 3136);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      //      cmd_text(371,181,31, 1536, ".");
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(346,183,30,1536,mA_Mantissa);
      //      RESTORE_CONTEXT();
      //      SAVE_CONTEXT();
      //      COLOR_RGB(255,255,255);
      //      cmd_number(389,183,30,1536,mA_Fraction);
      cmd_text(371, 181, 30, 1536, laserCurrent);

      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(435, 183, 29, 1536, "mA");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_text(262, 183, 30, 1536, "Power");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(255, 255, 255);
      cmd_fgcolor(0xd00000);
      cmd_gradcolor(0xffffff);

      FTImpl.Tag(BUTTON_EMG_STOP);
      cmd_button(30, 152, 132, 93, 30, ((DisplayButtonPressed() == BUTTON_EMG_STOP) ? FT_OPT_FLAT : 0), "ESTOP");

      RESTORE_CONTEXT();
      SAVE_CONTEXT();

      if ( INTERLOCK_SET() ) {
        COLOR_RGB(85, 170, 0);
      } else {
        COLOR_RGB(255, 0, 0);
      }
      //      COLOR_RGB(85,170,0);

      LINE_WIDTH(80);
      BEGIN(RECTS);
      VERTEX2F(592, 1056);
      VERTEX2F(2496, 2096);
      END();
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(96, 86, 29, 1536, "Safety");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(0, 0, 0);
      cmd_text(97, 111, 29, 1536, "Interlock");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(229, 225, 205);
      cmd_text(96, 110, 29, 1536, "Interlock");
      RESTORE_CONTEXT();
      SAVE_CONTEXT();
      COLOR_RGB(229, 225, 205);
      cmd_text(95, 85, 29, 1536, "Safety");
      RESTORE_CONTEXT();
      DISPLAY();
      cmd_swap();
      break;

    default:
#ifdef DISPLAY_DEBUG_SCREEN
      DISPLAY_SERIAL_DEVICE.println("ERROR: Unknown Screen Selected!");
#endif
      break;
  }

}



uint16_t StepSizeDialClick( uint16_t raw_val ) {

  uint16_t val = 0;
  uint16_t check_val = 2 * STEPSIZE_DIAL_DIV;

  if ( raw_val < (STEPSIZE_TICK_0 + 4096)) {
    JogStepSize = JOGSTEPSIZE_0;
    return STEPSIZE_TICK_0;
  }
  else if ( (raw_val > (STEPSIZE_TICK_1 - 4096)) && (raw_val < (STEPSIZE_TICK_1 + 4096)) )  {
    JogStepSize = JOGSTEPSIZE_1;
    return STEPSIZE_TICK_1;
  }
  else if ( (raw_val > (STEPSIZE_TICK_2 - 4096)) && (raw_val < (STEPSIZE_TICK_2 + 4096)) )  {
    JogStepSize = JOGSTEPSIZE_2;
    return STEPSIZE_TICK_2;
  }
  else if ( (raw_val > (STEPSIZE_TICK_3 - 4096)) && (raw_val < (STEPSIZE_TICK_3 + 4096)) )  {
    JogStepSize = JOGSTEPSIZE_3;
    return STEPSIZE_TICK_3;
  }
  else if ( (raw_val > (STEPSIZE_TICK_4 - 4096)) && (raw_val < (STEPSIZE_TICK_4 + 4096)) )  {
    JogStepSize = JOGSTEPSIZE_4;
    return STEPSIZE_TICK_4;
  }
  JogStepSize = JOGSTEPSIZE_5;
  return STEPSIZE_TICK_5;
}



void DisplayUpdate() {

  //  char val_str[32];
  //  uint32_t touchTag = 0;
  uint16_t dialAngle;
  uint16_t clickedDialVal;

  if ( DisplayAttached() == false ) return;

  if ( displayCalibrated == false ) {
    DisplayCalibrateTouch();
  }

  /* Read the touch screen xy and tag from GetTagXY API */
  //    FTImpl.GetTagXY(sTagxy);

  //----------------------------------------------------------
  // Update parameters for DisplayLists...
  //----------------------------------------------------------
  DisplayUpdateLimitSwitches( X_MIN_HIT(), Y_MIN_HIT(), INTERLOCK_SET() );
  DisplayUpdateLocation( real_position[X_AXIS], real_position[Y_AXIS] );
  DisplayUpdateLaserPower( AnalogMeasurement( ANALOG_CHANNEL_LASER ), LaserPwmPercent() );
  DisplayUpdateWaterCooling( AnalogMeasurement( ANALOG_CHANNEL_THERM0 ), AnalogMeasurement( ANALOG_CHANNEL_THERM1 ) );

  //----------------------------------------------------------
  // Set DisplayList and Render...
  //----------------------------------------------------------
  DisplayScreen();

  //----------------------------------------------------------
  // One-Shot the touch events...
  //----------------------------------------------------------
  //    touchTag = DisplayGetTags();

  //    SerialUSB.println(touchTag);

  //    if( touchTag > (uint32_t)0 ) {
  //      if( ButtonDownCount == 1 ) {
  //        if( ButtonTag != touchTag ) {
  //          ButtonTag = touchTag;
  //        }
  //      }
  //      ButtonDownCount++;
  //    } else {
  //      ButtonDownCount = 0;
  //      ButtonTag       = 0;
  //    }

  uint8_t touch_tag = DisplayReadButtons();



  //    if( (touchTag != lastTouchTag) && (ButtonHandled == false) ) {
  //      SerialUSB.print("TAG Event:");
  //      SerialUSB.println(touchTag);
  //      ButtonHandled = false;
  //    } else {
  //
  //      touchTag = lastTouchTag;
  //    }

  //    if( ButtonDownCount == 1 ) {

  //----------------------------------------------------------
  // Handle any touch events...
  //----------------------------------------------------------
  switch ( touch_tag ) {

    case TOGGLE_MOVE_ENABLE:
      ManualMoveEnabled   = !ManualMoveEnabled;
      break;

    case TOGGLE_LASER_ENABLE:
      LaserCalEnabled     = !LaserCalEnabled;
      break;
      
    case DIAL_STEPSIZE:

      dialAngle = DisplayReadDial( DIAL_STEPSIZE );

      //       SerialUSB.print("Tracking DIAL_STEPSIZE:");
      //       SerialUSB.println( dialAngle );

      clickedDialVal = StepSizeDialClick( dialAngle );

      DisplaySetStepSizeDial( clickedDialVal );

      //       SerialUSB.print("Clicked Dial To:");
      //       SerialUSB.println( clickedDialVal );

      //       SerialUSB.print("New Jog Interval:");
      //       SerialUSB.println( JogStepSize );

      break;

    case BUTTON_BOOT_CONTINUE:
      //        SerialUSB.println("BUTTON_BOOT_CONTINUE");
      DisplaySetScreen( SCRN_WARNING );
      //        while( DisplayButtonPressed() == BUTTON_BOOT_CONTINUE ) delay(10);
      break;

    case BUTTON_WARN_OK:
      //        SerialUSB.println("BUTTON_WARN_OK");
      DisplaySetScreen( SCRN_MAIN_MENU );
      break;

    case BUTTON_MAIN_MENU:
      //        SerialUSB.println("BUTTON_MAIN_MENU");
      DisplaySetScreen( SCRN_MAIN_MENU );
      break;

    case BUTTON_MANUAL_MOVE:
      //        SerialUSB.println("BUTTON_MANUAL_MOVE");
      DisplaySetScreen( SCRN_MANUAL_MOVE );
      break;

    case BUTTON_LASER_CAL:
      //        SerialUSB.println("BUTTON_LASER_CAL");
      DisplaySetScreen( SCRN_MANUAL_LASER );
      break;

    case BUTTON_MONITOR:
      //        SerialUSB.println("BUTTON_MONITOR");
      DisplaySetScreen( SCRN_MONITOR );
      break;

    case BUTTON_MOVE_HOME:
      //        SerialUSB.println("BUTTON_MOVE_HOME");
      HomeAllAxis();
      break;

    case BUTTON_MOVE_PLUS_X:
      //       SerialUSB.println("BUTTON_MOVE_PLUS_X");
      ManualJog( X_AXIS, true );
      break;

    case BUTTON_MOVE_MINUS_X:
      //        SerialUSB.println("BUTTON_MOVE_MINUS_X");
      ManualJog( X_AXIS, false );
      break;

    case BUTTON_MOVE_PLUS_Y:
      //        SerialUSB.println("BUTTON_MOVE_PLUS_Y");
      ManualJog( Y_AXIS, true );
      break;

    case BUTTON_MOVE_MINUS_Y:
      //        SerialUSB.println("BUTTON_MOVE_MINUS_Y");
      ManualJog( Y_AXIS, false );
      break;

    case BUTTON_LASER_PULSE:
      SerialUSB.println("BUTTON_LASER_PULSE");
      // One Shot - Requires operator 'UnLock' for each pulse.
      if( LaserCalEnabled == true) { LaserCalEnabled = false; }

      break;

    case BUTTON_EMG_STOP:
      //        SerialUSB.println("BUTTON_EMG_STOP");
      ErmergencyStop = true;
      LaserEnable( false );
      digitalWrite( XY_ENABLE_PIN, HIGH );
      SerialUSB.println(F("ESTOP!"));
      break;

    default:
      if ( touch_tag > 0) {
        SerialUSB.print("TAG Event:");
        SerialUSB.println(touch_tag);
      }
      break;

  }

  //    }


  //  lastTouchTag = touchTag;

  //    sTagXY sTagxy;
  //    FTImpl.GetTagXY(sTagxy);
  //    int tagval = sTagxy.tag;

  //    if(tagval == ESTOP_BUTTON) {
  //      if( ErmergencyStop == false ) {
  //        ErmergencyStop = true;
  //        LaserEnable( false );
  //        digitalWrite( XY_ENABLE_PIN, HIGH );
  //        SerialUSB.println(F("ESTOP!"));
  //      }
  //    }


  //    dtostrf(real_position[0], 4, 3, false, val_str);
  //    cmd_text(342,73,FNT_DIGITAL, FT_OPT_CENTER, val_str );




}











