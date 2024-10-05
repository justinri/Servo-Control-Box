/*
 * Code by: Justin Rittenhouse  
*/

/*
 * LCD Webiste: https://www.sunfounder.com/learn/Sensor-Kit-v2-0-for-Arduino/lesson-1-display-by-i2c-lcd1602-sensor-kit-v2-0-for-arduino.html
 * Encoder Website: http://exploreembedded.com/wiki/index.php?title=Interactive_Menus_for_your_project_with_a_Display_and_an_Encoder
 */

// include the library code
#include<SPI.h>                 // Needed for Multimoto H-Bridge shield & MAX6675 TC Module
#include <Wire.h>               // Needed for LCD
#include <LiquidCrystal_I2C.h>  // Needed for LCD
#include <stdbool.h>            // Needed for booleans
#include <PinChangeInt.h>       //--(used for button interrupt for pause function)--
#include <Keypad.h>             // For Key pad


/**********************************************************/
//Defining Pins for Multimoto
//--(L9958 DIRection pins)--
#define DIR_M1 2
#define DIR_M2 3
#define DIR_M3 4
#define DIR_M4 7

//--(L9958 PWM pins)--
#define PWM_M1 9
#define PWM_M2 10 //--(Timer1)--
#define PWM_M3 5
#define PWM_M4 6 //--(Timer0)--

//--(L9958 slave select pins for SPI)--
#define SS_M1 11
#define SS_M2 12
#define SS_M3 13
#define SS_M4 14

//--(L9958 Enable for all 4 motors, define pin 8)--
#define ENABLE_MOTORS 8
//--(Multimoto/motors)--
int pwm1;
int pwm2;
int pwm3;
int pwm4;
bool dir1;
bool dir2;
bool dir3;
bool dir4;

int ANALOG_PIN12 = A12; //--(Motor 4 feedback)--
int ANALOG_PIN13 = A13; //--(Motor 3 feedback)--
int ANALOG_PIN14 = A14; //--(Motor 2 feedback)--
int ANALOG_PIN15 = A15; //--(Motor 1 feedback)--

/**********************************************************/
// For Keypad
char keypressed = 'None';
double tempFeedback = 0;
int value0;
int value1;
int value2;
int value3;
int i=0;  // For foor loops
static int Display = 0;
long val;
long readVal;
bool activated;
const byte ROWS = 4; //--(four rows)--
const byte COLS = 4; //--(four columns)--
char keys[ROWS][COLS] = {
  {'1', '2', '3', 'A'},
  {'4', '5', '6', 'B'}, //--(define the symbols on the buttons of the keypad)--
  {'7', '8', '9', 'C'},
  {'*', '0', '#', 'D'}
};
byte rowPins[ROWS] = {26, 27, 28, 29};
byte colPins[COLS] = {30, 31, 32, 33};

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );  //--(Initialize an instance of class NewKeypad)--

/**********************************************************/
// Time sweeping varibles
int startEnd[4] = {0, 0, 0, 0};
int startPot[4] = {0, 0, 0, 0};
int endPot[4]   = {0, 0, 0, 0};
bool startSweep[4] = {0, 0, 0, 0};
int oneTwoWay[4] = {0, 0, 0, 0};
int tmpWayServo = 0;                // tmp variable to tell what servo needs to move, sweeping
bool ifFailed = 0;                  // If the voltage could not be reached, break out of for if statement.
int multiSweep[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};   // Allows for 12 stop points
int numSweepPoints = 0;


/**********************************************************/
// Start up display for LCD
int StartTime = 1000;                      //the value to show start up display
int showStart = 0;

// initialize the library with the numbers of the interface pins
LiquidCrystal_I2C lcd(0x27,20,4); // set the LCD address to 0x27 for a 20 chars and 4 line display

/*********************************************************/
// Creating custom chars
byte upArrow[8] = {
  B00100,
  B01110,
  B10101,
  B00100,
  B00100,
  B00100,
  B00100,
  B00100,
};

byte downArrow[8] = {
  B00000,
  B00100,
  B00100,
  B00100,
  B00100,
  B10101,
  B01110,
  B00100,
};

/*********************************************************/
// Encoder pin connections and set up
static int pinA = 18; // Our first hardware interrupt pin is digital (pin 2 UNO), (pin 18 MEGA)
static int pinB = 19; // Our second hardware interrupt pin is digital (pin 3 UNO), (pin 19 MEGA)
static int selectSwitch = 24; //The select switch for our encoder. (pin 4 UNO), (pin 24 MEGA)
volatile byte aFlag = 0; // let's us know when we're expecting a rising edge on pinA to signal that the encoder has arrived at a detent
volatile byte bFlag = 0; // let's us know when we're expecting a rising edge on pinB to signal that the encoder has arrived at a detent (opposite direction to when aFlag is set)
volatile byte encoderPos = 0; //this variable stores our current value of encoder position. Change to int or uin16_t instead of byte if you want to record a larger range than 0-255
volatile byte oldEncPos = 0; //stores the last encoder position value so we can compare to the current reading and see if it has changed (so we know when to print to the serial monitor)
volatile byte reading = 0; //somewhere to store the direct values we read from our interrupt pins before checking to see if we have moved a whole detent
int arrowLoc = 0;          //Ensures the arrow starts at zero.
int movArrow = 0;          // Moves the arrow up and down on the LCD
int inMenu = 100;            // 100 is the main menu.. 
bool entMenu = false;
int scrollSize = 4;
bool scroll[4] = {false, false, false, false};

/* inMenu varbiles
 * 100 = Main Menu
 * 00 = Servo Controls Menu
 * 11 = Servo 1 Menu 
 * 12 = Servo 2 Menu
 * 13 = Servo 3 Menu 
 * 14 = Servo 4 Menu 
 * 51 = Sweep Servo 1 Menu
 * 52 = Sweep Servo 2 Menu
 * 53 = Sweep Servo 3 Menu
 * 54 = Sweep Servo 4 Menu
 * 110 = Removed for proprietary reasons
 * 120 = Removed for proprietary reasons
 * 130 = Removed for proprietary reasons
 */

/*********************************************************/
// Servo control variables
int menuFlag = 0;
double Mtr1Target = 5000; // [mV]
double Mtr1Feedback = 0;  // Also used in status
double Mtr2Target = 5000; // [mV]
double Mtr2Feedback = 0;  // Also used in status
double Mtr3Target = 5000; // [mV]
double Mtr3Feedback = 0;  // Also used in status
double Mtr4Target = 5000; // [mV]
double Mtr4Feedback = 0;  // Also used in status
unsigned int offset = 5; //--(Global variable. If motors takes too long to find positioning, increase number, probably by 5's)--

/*********************************************************/
//--(Interrupt switch-PAUSE we are using pin 53 here)--
#define PAUSE 53
bool pauseFlag = 0;
bool pauseFlagWait = 0;
byte pauseFlagSoak = 0;
unsigned int z; //--(Pause function)--

/*********************************************************/
// Servo Status variables
unsigned long previousMillisDelay = 0; //--(millis() returns an unsigned long)--
unsigned long interval = 500;          // [ms] Updates LCD
bool fromStatus = false;               // Allows for changing pot setting from status menu
int tmpNumStatus;                      // For the status in the sub menu for each servo, 1-4

/*********************************************************/
// For adjusting global varibles
bool gloMinMax[2] = {false, false}; // [0] = min, [1] = max

/*********************************************************/
void setup(){
  analogReference(EXTERNAL); //--(Enable external STABLE voltage reference 5VDC-REQUIRED)--
  //--(Interrupt Switch-PAUSE)--
  pinMode(PAUSE, INPUT_PULLUP);
  PCintPort::attachInterrupt(PAUSE, &pauseFlip, FALLING);

  //For Multimoto
  unsigned int configWord;

  pinMode(SS_M1, OUTPUT); //--(This is port assignment)--
  digitalWrite(SS_M1, HIGH);
  pinMode(SS_M2, OUTPUT);
  digitalWrite(SS_M2, HIGH);
  pinMode(SS_M3, OUTPUT);
  digitalWrite(SS_M3, HIGH);
  pinMode(SS_M4, OUTPUT);
  digitalWrite(SS_M4, HIGH); //--(HIGH = not selected)--

  //--(L9958 DIRection pins)--
  pinMode(DIR_M1, OUTPUT);
  pinMode(DIR_M2, OUTPUT);
  pinMode(DIR_M3, OUTPUT);
  pinMode(DIR_M4, OUTPUT);

  //--(L9958 PWM pins)--
  pinMode(PWM_M1, OUTPUT);
  digitalWrite(PWM_M1, LOW);
  pinMode(PWM_M2, OUTPUT);
  digitalWrite(PWM_M2, LOW); //--(Timer1)--
  pinMode(PWM_M3, OUTPUT);
  digitalWrite(PWM_M3, LOW);
  pinMode(PWM_M4, OUTPUT);
  digitalWrite(PWM_M4, LOW); //--(Timer0)--

  //--(L9958 Enable for all 4 motors)--
  pinMode(ENABLE_MOTORS, OUTPUT);

  /******* Set up L9958 chips *********
    ' L9958 Config Register
    ' Bit
    '0 - RES
    '1 - DR - reset
    '2 - CL_1 - curr limit
    '3 - CL_2 - curr_limit
    '4 - RES
    '5 - RES
    '6 - RES
    '7 - RES
    '8 - VSR - voltage slew rate
    '9 - ISR - current slew rate
    '10 - ISR_DIS - current slew disable
    '11 - OL_ON - open load enable
    '12 - RES
    '13 - RES
    '14 - 0 - always zero
    '15 - 0 - always zero
  */

  //--(set to max current limit and disable ISR slew limiting)--
  configWord = 0b0000010000001100; //--(two byte binary word for SPI config, see table above)--
  SPI.beginTransaction(SPISettings(14000000, LSBFIRST, SPI_MODE1)); //--(Serial Peripheral Interface bus)--

  //--(Motor 1)--
  digitalWrite(SS_M1, LOW);
  SPI.transfer(lowByte(configWord)); //--(low byte half of config)--
  SPI.transfer(highByte(configWord)); //--(high byte half of config)--
  digitalWrite(SS_M1, HIGH);
  //--(Motor 2)--
  digitalWrite(SS_M2, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M2, HIGH);
  //--(Motor 3)--
  digitalWrite(SS_M3, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M3, HIGH);
  //--(Motor 4)--
  digitalWrite(SS_M4, LOW);
  SPI.transfer(lowByte(configWord));
  SPI.transfer(highByte(configWord));
  digitalWrite(SS_M4, HIGH);

  // LCD Set up
  lcd.init(); //initialize the lcd
  lcd.backlight(); //open the backlight 
  lcd.clear();

  // Encoder set up
  pinMode(pinA, INPUT_PULLUP); // set pinA as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(pinB, INPUT_PULLUP); // set pinB as an input, pulled HIGH to the logic voltage (5V or 3.3V for most cases)
  pinMode(selectSwitch, INPUT_PULLUP);
  attachInterrupt(4,PinA,RISING); // set an interrupt on PinA, looking for a rising edge signal and executing the "PinA" Interrupt Service Routine (below)
  attachInterrupt(5,PinB,RISING); // set an interrupt on PinB, looking for a rising edge signal and executing the "PinB" Interrupt Service Routine (below)

  // Creating special Chars
  lcd.createChar(0, upArrow);   // byte 0
  lcd.createChar(1, downArrow); // byte 1

  Serial.begin(9600); // Not needed in real code
}

/*********************************************************/
void loop(){
  // Starting up display for LCD
  if (showStart == 0){
  lcd.clear();
  lcd.setCursor(0,0);  lcd.print("Justin is Awesome!");
  lcd.setCursor(9,3); lcd.print("True Story!");
  delay(StartTime);
  lcd.clear(); //Clears the LCD screen and positions the cursor in the upper-left corner.

  digitalWrite(ENABLE_MOTORS, LOW);  //--(H-Bridge-enable = LOW)--
  showStart++;
  }

//
//  digitalWrite(ENABLE_MOTORS, LOW);  //--(H-Bridge-enable = LOW)--
  pwm1 = 0;
  analogWrite(PWM_M1, pwm1);  digitalWrite(DIR_M1, dir1);
  analogWrite(PWM_M2, pwm1);  digitalWrite(DIR_M2, dir1);
  analogWrite(PWM_M3, pwm1);  digitalWrite(DIR_M3, dir1);
  analogWrite(PWM_M4, pwm1);  digitalWrite(DIR_M4, dir1);
  

  // Creates main menu for selecting motors
  if (showStart == 1){
  lcd.clear();
  lcd.setCursor(3,0); lcd.print("Servo Controls");  // set the cursor to column 4, line 0 to allow room for arrow
  lcd.setCursor(3,1); lcd.print("Removed for proprietary reasons");  // set the cursor to column 4, line 1 to allow room for arrow
  lcd.setCursor(3,2); lcd.print("Removed for proprietary reasons");  // set the cursor to column 4, line 2 to allow room for arrow
  lcd.setCursor(3,3); lcd.print("Removed for proprietary reasons");  // set the cursor to column 4, line 3 to allow room for arrow
      
  showStart++;
  }

/*********************************************************/
  // Going back to main menu via the B button
  if (keypressed != 'None'){keypressed = 'None';} // Resetting keypressed if needed.
  keypressed = kpd.getKey();
  if (keypressed == 'B'){
    // For Main Menu
    if (inMenu == 11 || inMenu == 12 || inMenu == 13 || inMenu == 14 || inMenu == 04){
        scroll[0] = false; scroll[1] = false; arrowLoc = 0; inMenu = 00;
        inMenu = Servo_Controls(&arrowLoc, &entMenu, inMenu);
        
    } else {
        showStart = 1; scroll[0] = false; scroll[1] = false; arrowLoc = 0; inMenu = 100;
    }
  }


Unfortunately the rest of the code and additional function files were not included for proprietary reasons


