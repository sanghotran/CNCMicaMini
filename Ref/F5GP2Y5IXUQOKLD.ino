/**********************************************************************
  DRUM PLOTTER Version 4
  Code by lingib
  Last update 16 January 2017

  Changes:
  Biarc curves added 15 November 2016.
  Interpreter rewritten 2 December 2016 [1]
  Move_to() & draw_line() functions reworked 16 December 2016 [2]
  Draw_line() and move_to functions rewritten 16 January 2017 [3}
  Minor bug fix 16 January 2017 [4]
  Step counters added 16 January 2017[5]

  [1] The interpreter now uses string functions rather than pointers. This means
  that command lines may now start with a string (e.g. menu) which makes expansion
  extremely easy.

  [2] Slight bumps when drawing circles eliminated by replacing (int) with
  round() and determining the octant in the reverse direction.

  [3} Version 4 now uses a faster draw_line() algorithm and a matching a different 
  move_to() function. 
  
  [4] Every so often a my arc routines would produce a random circle!! 
  This was traced to Inkscape producing some very large I,J biarc values ... sometimes 
  over 5000 (5 meters)!! The problem with such large numbers is that the square of these
  is used to calculate the radius ... but I,J values over 181, when squared, will exceed the 
  size of an integer which is 32767 producing a wrap-around with funny side-effects.
  For all intents and purposes a line between two point with such a large circle-radius will 
  be straight. Rather than use 'double' I simply ignore all I,J values greater than 100.
  
  [5] Step counters for each motor direction have also been added to the menu. 
  These were included to help debug a stepper motor problem and have been left in as
  they provide some interesting data. For example my last 4-layer drawing required in excess of 
  2 million motor steps.
   

  COPYRIGHT
  This code is free software: you can redistribute it and/or
  modify it under the terms of the GNU General Public License as published
  by the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This software is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License. If
  not, see <http://www.gnu.org/licenses/>.
***************************************************************************/

// -------------------------------
// GLOBALS
// -------------------------------

// ----- constants
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
#define DEG_TO_RAD 0.017453292519943295769236907684886
#define RAD_TO_DEG 57.295779513082320876798154814105

// ----- motor definitions
#define MOTOR1 PORTB            //pins 8,9,10,11
#define MOTOR2 PORTC            //pins A0,A1,A2,A3
#define UP false                //clockwise rotation
#define DOWN true               //counter-clockwise rotation
#define LEFT true               //counter-clockwise rotation
#define RIGHT false             //clockwise rotation
float X_STEPS_PER_MM = 51.2;    //for GT-2 pulley
float Y_STEPS_PER_MM = 7.6;     //for 90mm diameter drum
int STEPX = 0;                  //X-axis rotor position
int STEPY = 0;                  //Y-axis rotor position

// ----- step counters for debugging
double Xleft = 0;
double Xright = 0;
double Yup = 0;
double Ydown = 0;

// ----- plotter definitions
#define PEN 3
#define BAUD 9600
#define XOFF 0x13               //pause transmission (19 decimal)
#define XON 0x11                //resume transmission (17 decimal)
float SCALE_FACTOR = 1;         //drawing scale (1 = 100%)
int LAST_X = 0;                 //current X co-ordinate
int LAST_Y = 0;                 //current Y co-ordinate
bool DEBUG_ON = false;          //true=on; false=off

// ----- gcode definitions
#define STRING_SIZE 128         //string size
char BUFFER[STRING_SIZE + 1];
char INPUT_CHAR;
int INDEX = 0;
String INPUT_STRING;

String  SUB_STRING;
int   START,                    //used for sub_string extraction
      FINISH;
float X,                        //gcode float values held here
      Y,
      I,
      J;

// ----- gcode circle definitions
float LINE_MAX = 2;             //maximum line length (mm) when plotting arc

// -----------------------
// SETUP
// -----------------------
void setup()
{
  // ----- motor1 (X-axis)
  int pattern = DDRB;               //get PORTB data directions
  pattern = pattern | B00001111;    //preserve MSN data direction &
  DDRB = pattern;                   //make pins 8,9,10,11 outputs

  // ----- motor2 (Y-axis)
  pattern = DDRC;                   //get PORTC data directions
  pattern = pattern | B00001111;    //preserve MSN data direction &
  DDRC = pattern;                   //make pins A0,A1,A2,A3 outputs

  // ----- pen-lift
  pinMode(PEN, OUTPUT);             //D3
  TCCR2A = _BV(COM2B1) | _BV(COM2B0) | _BV(WGM20);   //PWM
  TCCR2B = _BV(WGM22) | _BV(CS22) | _BV(CS21) | _BV(CS20);  //div 1024
  OCR2A = 156;                      //20mS period
  OCR2B = 148;                      //2mS (pen-up)

  /*
    The above pen-lift comprises a standard servo which requires
    1mS or 2mS pulses with a fixed period of 20mS for pen-down or pen-up.

    The Arduino "bit value" macro, #define _BV(x) (1 << x), is used to
    set the Timer2 mode to "phase-correct PWM" with a variable "top-limit".
    In this mode the timer counts up to the value entered into register OCR2A
    then back down to zero.

    The following values were used to obtain a 20mS period at pin D3:
      clock:        16 MHz
    prescaler:      1024
    top-limit (OCR2A):  156
    period:       16MHz/1024/(156*2) = 50Hz (20mS)

    If you enter a value into register OCR2B that is LESS than the value in
    register OCR2A then timer2 will will pass through the value in OCR2B
    twice ... once on the way up ... and once on the way down. The duration
    of the output pulse on pin D3 is the time that the count in OCR2A is
    greater than the value in OCR2B.

    A value of 148 entered into OCR2B creates a 1mS pulse:
    period:       156-148)*20mS/156 = 1mS (pen-up)

    A value of 140 entered into OCR2B creates a 2mS pulse):
    period:     156-140)*20mS/156 = 2mS (pen-down)
  */

  // ----- plotter setup
  memset(BUFFER, '\0', sizeof(BUFFER));     //fill with string terminators
  INPUT_STRING.reserve(STRING_SIZE);
  INPUT_STRING = "";

  // ----- establish serial link
  Serial.begin(BAUD);

  // ----- flush the buffers
  Serial.flush();                           //clear TX buffer
  while (Serial.available()) Serial.read(); //clear RX buffer

  // ----- display commands
  menu();
}

//--------------------------------------------------------------------------
// MAIN LOOP
//--------------------------------------------------------------------------
void loop() {
  while (Serial.available()) {
    INPUT_CHAR = (char)Serial.read();         //read character
    Serial.write(INPUT_CHAR);                 //echo character to the screen
    BUFFER[INDEX++] = INPUT_CHAR;             //add char to buffer
    if (INPUT_CHAR == '\n') {                 //check for line feed
      Serial.flush();                         //clear TX buffer
      Serial.write(XOFF);                     //pause transmission
      INPUT_STRING = BUFFER;                  //convert to string
      process();                              //interpret string and perform task
      memset(BUFFER, '\0', sizeof(BUFFER));   //fill buffer with string terminators
      INDEX = 0;                              //point to buffer start
      INPUT_STRING = "";                      //empty string
      Serial.flush();                         //clear TX buffer
      Serial.write(XON);                      //resume transmission
    }
  }
}

//--------------------------------------------------------------------------
// PROCESS
//--------------------------------------------------------------------------
void process() {

  // ----- convert string to upper case
  INPUT_STRING.toUpperCase();

  // ------ verify input
  if (DEBUG_ON) {
    Serial.print(INPUT_STRING);
  }

  // ----------------------------------
  // G00   linear move with pen_up
  // ----------------------------------
  if (INPUT_STRING.startsWith("G00")) {

    // ------ verify input
    if (DEBUG_ON) {
      Serial.println("G00");
    }

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("X"); Serial.println(X, 6);
      }
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("Y"); Serial.println(Y, 6);
        Serial.println("");
      }
    }

    pen_up();
    move_to(X, Y);
  }

  // ----------------------------------
  // G01   linear move with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G01")) {     //linear move with pen-up

    // ------ verify input
    if (DEBUG_ON) {
      Serial.println("G01");
    }

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      X = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("X"); Serial.println(X, 6);
      }
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = START + 8;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 1);
      Y = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("Y"); Serial.println(Y, 6);
        Serial.println("");
      }
    }

    pen_down();
    move_to(X, Y);
  }

  // ----------------------------------
  // G02   clockwise arc with pen_down
  // ----------------------------------
  if (INPUT_STRING.startsWith("G02")) {

    // ------ verify input
    if (DEBUG_ON) {
      Serial.println("G02");
    }

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("X"); Serial.println(X, 6);
      }
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("Y"); Serial.println(Y, 6);
      }
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("I"); Serial.println(I, 6);
      }
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("J"); Serial.println(J, 6);
        Serial.println("");
      }
    }

    pen_down();
    draw_arc_cw(X, Y, I, J);
  }

  // ------------------------------------------
  // G03   counter-clockwise arc with pen_down
  // ------------------------------------------
  if (INPUT_STRING.startsWith("G03")) {

    // ------ verify input
    if (DEBUG_ON) {
      Serial.println("G03");
    }

    // ----- extract X
    START = INPUT_STRING.indexOf('X');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('X'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      X = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("X"); Serial.println(X, 6);
      }
    }

    // ----- extract Y
    START = INPUT_STRING.indexOf('Y');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('Y'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      Y = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("Y"); Serial.println(Y, 6);
      }
    }

    // ----- extract I
    START = INPUT_STRING.indexOf('I');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('I'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      I = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("I"); Serial.println(I, 6);
      }
    }

    // ----- extract J
    START = INPUT_STRING.indexOf('J');
    if (!(START < 0)) {
      FINISH = INPUT_STRING.indexOf('.', INPUT_STRING.indexOf('J'));
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH + 7);
      J = SUB_STRING.toFloat();

      // ------ verify input
      if (DEBUG_ON) {
        Serial.print("J"); Serial.println(J, 6);
        Serial.println("");
      }
    }

    pen_down();
    draw_arc_ccw(X, Y, I, J);
  }

  // ----------------------------------
  // MENU
  // ----------------------------------
  if (INPUT_STRING.startsWith("MENU")) {
    menu();
  }

  // ----------------------------------
  // T1   position the pen over 0,0
  // ----------------------------------
  if (INPUT_STRING.startsWith("T1")) {

    // ----- variables
    int step;           //loop counter

    // ----- instructions
    Serial.println(F(""));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    Position the pen over the 0,0 co-ordinate:"));
    Serial.println(F("  ----------------------------------------------"));
    Serial.println(F("    X-axis:             Y-axis:"));
    Serial.println(F("   'A'  'S'            'K'  'L'"));
    Serial.println(F("   <-    ->            <-    ->"));
    Serial.println(F("    Exit = 'E'"));

    // ----- flush the buffer
    while (Serial.available() > 0) Serial.read();

    // ----- control motors with 'A', 'S', 'K', and 'L' keys

    char keystroke = ' ';
    while (keystroke != 'E') {  //press 'E' key to exit

      // ----- check for keypress
      if (Serial.available() > 0) {
        keystroke = (char) Serial.read();
      }

      // ----- select task
      switch (keystroke) {
        case 'a':
        case 'A': {
            // ----- move pen 5mm left
            for (step = 0; step < 512; step++) {
              moveX(LEFT);
            }
            keystroke = ' ';    //otherwise motor will continue to rotate
            break;
          }
        case 's':
        case 'S': {
            // ------ move pen 5mm right
            for (step = 0; step < 512; step++) {
              moveX(RIGHT);
            }
            keystroke = ' ';
            break;
          }
        case 'k':
        case 'K': {
            // ----- move pen 5mm up
            for (step = 0; step < 72; step++) {
              moveY(UP);
            }
            keystroke = ' ';
            break;
          }
        case 'l':
        case 'L': {
            // ----- move pen 5mm down
            for (step = 0; step < 72; step++) {
              moveY(DOWN);
            }
            keystroke = ' ';
            break;
          }
        case 'e':
        case 'E': {
            // ----- exit
            Serial.println(F(" "));
            Serial.println(F("  Calibration complete ..."));
            keystroke = 'E';
            break;
          }
        // ----- default for keystroke
        default: {
            break;
          }
      }
    }

    LAST_X = 0;
    LAST_Y = 0;
    Xleft = 0;
    Xright = 0;
    Yup = 0;
    Ydown = 0;
  }

  // ----------------------------------
  // T2   set scale factor
  // ----------------------------------
  if (INPUT_STRING.startsWith("T2")) {
    Serial.println("T2");

    START = INPUT_STRING.indexOf('S');
    if (!(START < 0)) {
      FINISH = START + 6;
      SUB_STRING = INPUT_STRING.substring(START + 1, FINISH);
      SCALE_FACTOR = SUB_STRING.toFloat();
      Serial.print(F("Drawing now ")); Serial.print(SCALE_FACTOR * 100); Serial.println(F("%"));
    }
    else {
      Serial.println(F("Invalid scale factor ... try again. (1 = 100%)"));
    }
    // ------ verify input
    if (DEBUG_ON) {
      Serial.print("Scale factor: "); Serial.println(SCALE_FACTOR);
      Serial.println("");
    }
  }

  // ----------------------------------
  // T3   pen up
  // ----------------------------------
  if (INPUT_STRING.startsWith("T3")) {
    pen_up();
  }

  // ----------------------------------
  // T4   pen down
  // ----------------------------------
  if (INPUT_STRING.startsWith("T4")) {
    pen_down();
  }

  // ----------------------------------
  // T5   ABC test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T5")) {
    abc();
  }

  // ----------------------------------
  // T6   target test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T6")) {
    test_pattern();
  }

  // ----------------------------------
  // T7   radial line test pattern
  // ----------------------------------
  if (INPUT_STRING.startsWith("T7")) {
    test_pattern_2();
  }
  // ----------------------------------
  // T8   view step counters
  // ----------------------------------
  if (INPUT_STRING.startsWith("T8")) {
    Serial.print("  X left:"); 
    Serial.print(Xleft,0); 
    Serial.print("  X right:"); 
    Serial.print(Xright,0); 
    Serial.print("  Y up:"); 
    Serial.print(Yup,0); 
    Serial.print("  Y down:"); 
    Serial.println(Ydown,0);
  }
  // ----------------------------------
  // T9   clear step counters
  // ----------------------------------
  if (INPUT_STRING.startsWith("T9")) {
    Xleft = 0;
    Xright = 0;
    Yup = 0;
    Ydown = 0;
    Serial.print("  X left:"); 
    Serial.print(Xleft,0); 
    Serial.print("  X right:"); 
    Serial.print(Xright,0); 
    Serial.print("  Y up:"); 
    Serial.print(Yup,0); 
    Serial.print("  Y down:"); 
    Serial.println(Ydown,0);
  }
}

//---------------------------------------------------------------------------
// MENU
// The Arduino F() flash macro is used to conserve RAM.
//---------------------------------------------------------------------------
void menu() {
  Serial.println(F(""));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("                         MENU"));
  Serial.println(F("  ------------------------------------------------------"));
  Serial.println(F("    MENU ............... menu"));
  Serial.println(F("    G00 X## Y## ........ goto XY (pen-up)"));
  Serial.println(F("    G01 X## Y## ........ goto XY (pen-down)"));
  Serial.println(F("    T1 ................. move pen to 0,0"));
  Serial.println(F("    T2 S##.## .......... set drawing Scale (1=100%)"));
  Serial.println(F("    T3 ................. pen up"));
  Serial.println(F("    T4 ................. pen down"));
  Serial.println(F("    T5 ................. draw ABC pattern"));
  Serial.println(F("    T6 ................. draw test pattern 1"));
  Serial.println(F("    T7 ................. draw test pattern 2"));
  Serial.println(F("    T8 ................. view step counter"));
  Serial.println(F("    T9 ................. clear step counter"));
  Serial.println(F("  ------------------------------------------------------"));
}

// -------------------------------
// move to
// -------------------------------
void move_to(float x, float y) {
  int x1 = LAST_X,
      y1 = LAST_Y,
      x2 = round(x * SCALE_FACTOR * X_STEPS_PER_MM),
      y2 = round(y * SCALE_FACTOR * Y_STEPS_PER_MM);

  draw_line(x1, y1, x2, y2);

  LAST_X = x2;
  LAST_Y = y2;
}

// ------------------------------------------------------------------------
// draw line
// This code is faster than that for the original CNC Drum Plotter. The
// algorithm automatically maps all "octants" to "octant 0" and automatically
// swaps the XY coordinates if dY is greater than dX. A swap flag determines
// which motor moves for any combination X,Y inputs. The swap algorithm is 
// further optimised by realising that dY is always positive in quadrants
// 0,1 and that dX is always positive in "quadrants" 0,3.
// ------------------------------------------------------------------------
void draw_line(int x1, int y1, int x2, int y2) {

  // ----- screen co-ordinates
  int x = x1;
  int y = y1;

  // ----- find longest and shortest axis
  int dy = y2 - y1;  //vertical distance  
  int dx = x2 - x1;  //horizontal distance
  int longest = max(abs(dy), abs(dx));  //longest axis
  int shortest = min(abs(dy), abs(dx));  //shortest axis

  // ----- scale Bresenham values by 2*longest
  //int error = 0;
  int error = -longest;  //add offset to so we can test at zero
  //int threshold = longest;
  int threshold = 0;  //test now done at zero
  int maximum = (longest << 1);
  int slope = (shortest << 1);  // slope equals (shortest*2/longest*2)

  // ----- swap flag
  //The XY axes are automatically swapped by using "longest" in 
  //the "for loop". XYswap is used to decode the motors.
  boolean XYswap = true;
  if (abs(dx)>=abs(dy)) XYswap=false;

  // ----- pretend we are always in octant 0
  for (int i = 0; i < longest; i++) {

    // ----- move left/right along X axis
    if (XYswap) {  //swap
      if (dy<0) {
        //y--;
        moveY(DOWN);
      } else {
        //y++;
        moveY(UP);
      }
    } else {  //no swap
      if (dx<0) {
        //x--;
        moveX(LEFT);
      } else {
        //x++;
        moveX(RIGHT);
      }
    }

    // ----- move up/down Y axis 
    error += slope;
    if (error > threshold) {
      error -= maximum;

      // ----- move up/down along Y axis
      if (XYswap) {  //swap
        if (dx<0) {
          //x--;
          moveX(LEFT);
        } else {
          //x++;
          moveX(RIGHT);
        }
      } else {  //no swap
        if (dy<0) {
          //y--;
          moveY(DOWN);
        } else {
          //y++;
          moveY(UP);
        }
      }
    }
    //point(x, height-y); //for use with "Processing3"
  }
}

//------------------------------------------------
// MOVE X
// Controls the BJY-48 (X-axis) stepping-motor
//------------------------------------------------
void moveX(bool direction) {

  // ----- local variables
  int pattern;        //used for bitwise motor control

  if (direction) {
    STEPX++;
    Xright++;        //step counter for debugging
    if (STEPX > 3) { //wrap-around
      STEPX = 0;
    }
  } else {
    STEPX--;
    Xleft++;         //step counter for debugging
    if (STEPX < 0) { //wrap-around
      STEPX = 3;
    }
  }

  switch (STEPX) {
    case 0:
      pattern = MOTOR1;
      pattern = pattern & B11110000;  // erase motor current pattern
      pattern = pattern | B00000011;  // create new motor pattern
      MOTOR1 = pattern;
      break;
    case 1:
      pattern = MOTOR1;
      pattern = pattern & B11110000;
      pattern = pattern | B00000110;
      MOTOR1 = pattern;
      break;
    case 2:
      pattern = MOTOR1;
      pattern = pattern & B11110000;
      pattern = pattern | B00001100;
      MOTOR1 = pattern;
      break;
    case 3:
      pattern = MOTOR1;
      pattern = pattern & B11110000;
      pattern = pattern | B00001001;
      MOTOR1 = pattern;
      break;
    default:
      pattern = MOTOR1;
      pattern = pattern & B11110000;
      pattern = pattern | B00000000;
      MOTOR1 = pattern;
      break;
  }

  delay(2);     //allow motor time to move
}

//------------------------------------------------
// MOVE Y
// Controls the BJY-48 (Y-axis) stepping-motor
//------------------------------------------------
void moveY(bool direction) {

  // ----- local variables
  int pattern;        //used for bitwise motor control

  if (direction) {
    STEPY++;
    Yup++;           //step counter for debugging  
    if (STEPY > 3) { //wrap-around
      STEPY = 0;
    }
  } else {
    STEPY--;
    Ydown++;         //step counter for debugging
    if (STEPY < 0) { //wrap-around
      STEPY = 3;
    }
  }

  switch (STEPY) {
    case 0:
      pattern = MOTOR2;
      pattern = pattern & B11110000;  // erase motor current pattern
      pattern = pattern | B00000011;  // create new motor pattern
      MOTOR2 = pattern;
      break;
    case 1:
      pattern = MOTOR2;
      pattern = pattern & B11110000;
      pattern = pattern | B00000110;
      MOTOR2 = pattern;
      break;
    case 2:
      pattern = MOTOR2;
      pattern = pattern & B11110000;
      pattern = pattern | B00001100;
      MOTOR2 = pattern;
      break;
    case 3:
      pattern = MOTOR2;
      pattern = pattern & B11110000;
      pattern = pattern | B00001001;
      MOTOR2 = pattern;
      break;
    default:
      pattern = MOTOR2;
      pattern = pattern & B11110000;
      pattern = pattern | B00000000;
      MOTOR2 = pattern;
      break;
  }

  delay(14);      //match drum speed to print-head
}

//----------------------------------------------------------------------------
// DRAW ARC CLOCKWISE (G02)
//----------------------------------------------------------------------------
void draw_arc_cw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float thisX = LAST_X,     //current X co-ordinate
          thisY = LAST_Y,     //current Y co-ordinate
          nextX = x,        //next X co-ordinate
          nextY = y,        //next Y co-ordinate
          newX,           //interpolated X co-ordinate
          newY,           //interpolated Y co-ordinate
          I = i,          //horizontal distance thisX from circle center
          J = j,          //vertical distance thisY from circle center
          circleX = thisX + I,    //circle X co-ordinate
          circleY = thisY + J,    //circle Y co-ordinate
          delta_x,          //horizontal distance between thisX and nextX
          delta_y,          //vertical distance between thisY and nextY
          chord,          //line_length between lastXY and nextXY
          radius,         //circle radius
          alpha,          //interior angle of arc
          beta,           //fraction of alpha
          arc,            //subtended by alpha
          current_angle,      //measured CCW from 3 o'clock
          next_angle;       //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius)); //see construction lines
    arc = alpha * radius;         //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > LINE_MAX) {
      segments = (int)(arc / LINE_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    // atan2() angles between 0 and PI are CCW +ve from 3 o'clock.
    // atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI; //angles now 360..0 degrees CW

    // ----- display co-ordinates
    if (DEBUG_ON) {
      Serial.print(" startX:"); Serial.print(thisX, 2);
      Serial.print(" startY:"); Serial.print(thisY, 2);
      Serial.print(" start_angle:"); Serial.println((current_angle * RAD_TO_DEG), 2);
    }

    // ----- plot intermediate CW co-ordinates
    next_angle = current_angle;             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle -= beta;             //move CW around circle
      if (next_angle < 0) next_angle += 2 * PI; //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);  //standard circle formula
      newY = circleY + radius * sin(next_angle);

      // ----- display co-ordinates
      if (DEBUG_ON) {
        Serial.print(" segment:"); Serial.print(segment);
        Serial.print(" newX:"); Serial.print(newX);
        Serial.print(" newY:"); Serial.print(newY);
        Serial.print(" current_angle:"); Serial.println((next_angle * RAD_TO_DEG), 2);
      }

      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}

//----------------------------------------------------------------------------
// DRAW ARC COUNTER-CLOCKWISE (G03)
// We know the start and finish co-ordinates which allows us to calculate the
// chord length. We can also calculate the radius using the biarc I,J values.
// If we bisect the chord the center angle becomes 2*asin(chord/(2*radius)).
// The arc length may now be calculated using the formula arc_length = radius*angle.
//----------------------------------------------------------------------------
void draw_arc_ccw(float x, float y, float i, float j) {

  // ----- inkscape sometimes produces some crazy values for i,j
  if ((i < -100) || (i > 100) || (j < -100) || (j > 100)) {
    move_to(x, y);
  } else {

    // ----- variables
    float thisX = LAST_X,         //current X co-ordinate
          thisY = LAST_Y,         //current Y co-ordinate
          nextX = x,              //next X co-ordinate
          nextY = y,              //next Y co-ordinate
          newX,                   //interpolated X co-ordinate
          newY,                   //interpolated Y co-ordinate
          I = i,                  //horizontal distance thisX from circle center
          J = j,                  //vertical distance thisY from circle center
          circleX = thisX + I,    //circle X co-ordinate
          circleY = thisY + J,    //circle Y co-ordinate
          delta_x,                //horizontal distance between thisX and nextX
          delta_y,                //vertical distance between thisY and nextY
          chord,                  //line_length between lastXY and nextXY
          radius,                 //circle radius
          alpha,                  //interior angle of arc
          beta,                   //fraction of alpha
          arc,                    //subtended by alpha
          current_angle,          //measured CCW from 3 o'clock
          next_angle;             //measured CCW from 3 o'clock

    // ----- calculate arc
    delta_x = thisX - nextX;
    delta_y = thisY - nextY;
    chord = sqrt(delta_x * delta_x + delta_y * delta_y);
    radius = sqrt(I * I + J * J);
    alpha = 2 * asin(chord / (2 * radius)); //see construction lines
    arc = alpha * radius;               //radians

    // ----- sub-divide alpha
    int segments = 1;
    if (arc > LINE_MAX) {
      segments = (int)(arc / LINE_MAX);
      beta = alpha / segments;
    } else {
      beta = alpha;
    }

    // ----- calculate current angle
    // atan2() angles between 0 and PI are CCW +ve from 3 o'clock.
    // atan2() angles between 2*PI and PI are CW -ve relative to 3 o'clock
    current_angle = atan2(-J, -I);
    if (current_angle <= 0) current_angle += 2 * PI; //angles now 360..0 degrees CW

    // ----- display co-ordinates
    if (DEBUG_ON) {
      Serial.print(" startX:"); Serial.print(thisX, 2);
      Serial.print(" startY:"); Serial.print(thisY, 2);
      Serial.print(" start_angle:"); Serial.println((current_angle * RAD_TO_DEG), 2);
    }

    // ----- plot intermediate CCW co-ordinates
    next_angle = current_angle;             //initialise angle
    for (int segment = 1; segment < segments; segment++) {
      next_angle += beta;             //move CCW around circle
      if (next_angle > 2 * PI) next_angle -= 2 * PI; //check if angle crosses zero
      newX = circleX + radius * cos(next_angle);  //standard circle formula
      newY = circleY + radius * sin(next_angle);

      // ----- display co-ordinates
      if (DEBUG_ON) {
        Serial.print(" segment:"); Serial.print(segment);
        Serial.print(" newX:"); Serial.print(newX);
        Serial.print(" newY:"); Serial.print(newY);
        Serial.print(" current_angle:"); Serial.println((next_angle * RAD_TO_DEG), 2);
      }

      move_to(newX, newY);
    }

    // ----- draw final line
    move_to(nextX, nextY);
  }
}
//---------------------------------------------------------------------------
// PEN_UP
// Raise the pen
// Changing the value in OCR2B changes the pulse-width to the SG-90 servo
//---------------------------------------------------------------------------
void pen_up() {
  OCR2B = 148;                //1mS pulse
  delay(100);                 //give pen-lift time to respond
}

//---------------------------------------------------------------------------
// PEN_DOWN
// Lower the pen
// Changing the value in OCR2B changes the pulse-width to the SG-90 servo
//---------------------------------------------------------------------------
void pen_down() {
  OCR2B = 140;                //2mS pulse
  delay(100);                 //give pen-lift time to respond
}

//----------------------------------------------------------------------------
// ABC test pattern
//----------------------------------------------------------------------------
void abc() {

  // ------ letter C
  pen_up();
  move_to(71.607584, 21.035879);
  pen_down();
  move_to(70.163261, 20.392706);
  move_to(68.665833, 19.931058);
  move_to(67.123530, 19.654409);
  move_to(65.471170, 19.558347);
  move_to(60.824111, 20.352237);
  move_to(57.604313, 22.327055);
  move_to(55.521083, 25.458294);
  move_to(54.702496, 29.861135);
  move_to(55.523176, 34.275230);
  move_to(57.604313, 37.395213);
  move_to(60.826497, 39.380152);
  move_to(65.471170, 40.177231);
  move_to(67.123530, 40.081169);
  move_to(68.665833, 39.804521);
  move_to(70.163261, 39.342872);
  move_to(71.607584, 38.699700);
  move_to(71.607584, 34.586572);
  move_to(70.133934, 35.457974);
  move_to(68.798946, 36.010859);
  move_to(67.396000, 36.346751);
  move_to(65.883816, 36.463436);
  move_to(63.362672, 35.969576);
  move_to(61.571020, 34.706372);
  move_to(60.460591, 32.773568);
  move_to(60.000312, 29.861135);
  move_to(60.459740, 26.961684);
  move_to(61.571020, 25.029205);
  move_to(63.362672, 23.766000);
  move_to(65.883816, 23.272141);
  move_to(67.396000, 23.388826);
  move_to(68.798946, 23.724718);
  move_to(70.133934, 24.277603);
  move_to(71.607584, 25.149006);
  move_to(71.607584, 21.035879);
  pen_up();

  // ------ top inside-loop in letter 'B'
  pen_up();
  move_to(43.041974, 32.124019);
  pen_down();
  move_to(44.193140, 32.287491);
  move_to(44.878907, 32.656463);
  move_to(45.321578, 33.273441);
  move_to(45.504526, 34.227172);
  move_to(45.322608, 35.168069);
  move_to(44.878907, 35.784570);
  move_to(44.190670, 36.163294);
  move_to(43.041974, 36.330325);
  move_to(40.206713, 36.330325);
  move_to(40.206713, 32.124019);
  move_to(43.041974, 32.124019);
  pen_up();

  // ----- bottom inside-loop in letter 'B'
  pen_up();
  move_to(43.215018, 23.431875);
  pen_down();
  move_to(44.684832, 23.634884);
  move_to(45.531148, 24.084119);
  move_to(46.084429, 24.845298);
  move_to(46.316505, 26.054160);
  move_to(46.088504, 27.238072);
  move_to(45.544461, 27.984270);
  move_to(44.697894, 28.432828);
  move_to(43.215018, 28.636513);
  move_to(40.206713, 28.636513);
  move_to(40.206713, 23.431875);
  move_to(43.215018, 23.431875);
  pen_up();

  // ----- outside of letter 'B'
  pen_up();
  move_to(47.980391, 30.579932);
  pen_down();
  move_to(49.467494, 29.872216);
  move_to(50.536123, 28.809558);
  move_to(51.189538, 27.438932);
  move_to(51.441274, 25.641517);
  move_to(50.881551, 23.051631);
  move_to(49.497855, 21.355344);
  move_to(47.408388, 20.394118);
  move_to(43.587730, 19.944368);
  move_to(35.081941, 19.944368);
  move_to(35.081941, 39.817832);
  move_to(42.775754, 39.817832);
  move_to(46.788467, 39.403201);
  move_to(48.765745, 38.566589);
  move_to(50.084134, 37.024736);
  move_to(50.629298, 34.559950);
  move_to(50.441596, 33.165564);
  move_to(49.950432, 32.084086);
  move_to(49.146555, 31.229561);
  move_to(47.980391, 30.579932);
  pen_up();

  // ----- outside of letter 'A'
  pen_up();
  move_to(26.057020, 23.564986);
  pen_down();
  move_to(18.043741, 23.564986);
  move_to(16.779187, 19.944368);
  move_to(11.627794, 19.944368);
  move_to(18.988829, 39.817832);
  move_to(25.098621, 39.817832);
  move_to(32.459656, 19.944368);
  move_to(27.308262, 19.944368);
  move_to(26.057020, 23.564986);
  pen_up();

  // ----- inside of letter 'A'
  pen_up();
  move_to(19.321606, 27.252160);
  pen_down();
  move_to(24.765843, 27.252160);
  move_to(22.050380, 35.158949);
  move_to(19.321606, 27.252160);
  pen_up();

  // home --------------
  move_to(0.0000, 0.0000);
}

/***************************************************************************
  TEST_PATTERN
 ***************************************************************************/
void test_pattern() {

  // circle ------------
  pen_up();
  move_to(136.738441, 145.187821);
  pen_down();
  move_to(134.380298, 133.732203);
  move_to(127.595170, 123.920361);
  move_to(117.521703, 117.417222);
  move_to(105.521361, 115.111091);
  move_to(93.521020, 117.417222);
  move_to(83.447553, 123.920361);
  move_to(76.662425, 133.732203);
  move_to(74.304282, 145.187821);
  move_to(76.662425, 156.643438);
  move_to(83.447553, 166.455281);
  move_to(93.521020, 172.958420);
  move_to(105.521361, 175.264551);
  move_to(117.521703, 172.958420);
  move_to(127.595170, 166.455281);
  move_to(134.380298, 156.643438);
  move_to(136.738441, 145.187821);
  move_to(136.738441, 145.187821);
  pen_up();

  // back-slash -----------
  pen_up();
  move_to(37.813081, 210.330315);

  pen_down();
  move_to(174.084903, 79.190066);
  pen_up();

  // slash -------------
  pen_up();
  move_to(37.527994, 79.190066);
  pen_down();
  move_to(173.799816, 210.330315);
  pen_up();

  // square ------------
  pen_up();
  move_to(37.656509, 210.457146);
  pen_down();
  move_to(173.929525, 210.457146);
  move_to(173.929525, 79.022220);
  move_to(37.656509, 79.022220);
  move_to(37.656509, 210.457146);
  pen_up();

  // home --------------
  move_to(0.0000, 0.0000);
}

/***************************************************************************
  TEST_PATTERN_2
 ***************************************************************************/
void test_pattern_2() {

  // ----- move to the centre of the square
  pen_up();
  move_to(100, 100);

  // ----- draw octant 0 radials
  pen_down();
  move_to(150, 100);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 125);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 1 radials
  pen_down();
  move_to(125, 150);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(100, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 2 radials
  pen_down();
  move_to(75, 150);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 150);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 3 radials
  pen_down();
  move_to(50, 125);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 100);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 4 radials
  pen_down();
  move_to(50, 75);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(50, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 5 radials
  pen_down();
  move_to(75, 50);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(100, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 6 radials
  pen_down();
  move_to(125, 50);
  pen_up();
  move_to(100, 100);

  pen_down();
  move_to(150, 50);
  pen_up();
  move_to(100, 100);

  // ----- draw octant 7 radials
  pen_down();
  move_to(150, 75);
  pen_up();
  move_to(100, 100);
  pen_up();

  // ----- draw box
  move_to(50, 50);
  pen_down();
  move_to(50, 150);
  move_to(150, 150);
  move_to(150, 50);
  move_to(50, 50);
  pen_up();

  // home --------------
  move_to(0.0000, 0.0000);

}

