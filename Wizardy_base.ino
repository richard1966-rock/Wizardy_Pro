// Wizardy Pro V2
// Richard van Wijk - April 2026 
//
// Wizardy Pro is build with a Arduino Pro Mini Atmega328P 5V 16Mhz board.
// Sketch to be uploaded with FTDI FT232RL USB To TTL Serial IC Adapter Converter
// or other USB to TTL interface (e.g. Arduino Uno with chip removed)
// Wizardy Pro V2 features 4 analog / digital inputs and 8 digital inputs. The inputs are all multiplexed
// Wizardy Pro V2 features 4 potmeters that allow for external parameter adjustment. All potmeters are multiplexed to a single analog input
// Wizardy Pro V2 allow to set the 4 Q-Outputs independent of the state of the 2 motor outputs.
// Wizardy Pro V2 features 4 outputs for servo motors. Q1, Q2, Q3 are either digital outputs or Servo outputs. Servo 4 is independent. Q4 is independent.
// Make sure to switch off output logic in case Q1/2/3 is used as servo output. Check void "showOutputs" and "setOutputs" and definitions set pinmode - disable resp. Qx as output.
//
// Reading of inputs / POTS are done every cycle, however, display is only refreshed if changes happen to input/output/pots. This is to gain maximum loop speed
//
// Servo's can be adjusted in a generic way using ServoCh. Also, to gain speed, if angle is not changed, the void will be exited immediately.
//
// The modified TB6612 lib is based on: https://github.com/sparkfun/SparkFun_TB6612FNG_Arduino_Library
// Display 128x64 SSD1306 OLED - AdaFruit outputr: https://learn.adafruit.com/adafruit-gfx-graphics-library/
//
#include <Adafruit_SSD1306.h>
#include "Whizzbizz_TB6612.h" // Output lib for TB6612FNG enabling control over each output individually
#include <Servo.h>

// One servo = the Servo object + the last angle written to it
struct ServoCh {
  Servo servo;
  uint16_t last = 0xFFFF;      // 0xFFFF = nothing written yet
};

// TB6612 pins definition
#define AIN1 2   
#define AIN2 3   
#define PWMA 5
#define BIN1 10  
#define BIN2 11  
#define PWMB 6
#define STBY 14

// Inputs, Outputs and potmeters
#define INA            A1    // Multiplexed input for inputs 1-4 (analog). Max Input voltage 10V, resistor bridge dividing input voltage by 2.
#define INB            16    // Multiplexed input for inputs 5-8 (digital). Max Input voltage 9V, Capped with zener at 5V for Arduino
#define INC            A6    // Multiplexed input for inputs 9-12 (digital (A-D conversion in SW)). Max Input voltage 9V, Capped with zener at 5V for Arduino
#define INPOT          A7    // Multiplexed input for the 4 potmeters
#define S1             13    // Multiplexer Control bit 1
#define S2             12    // Multiplexer Control bit 2
#define MINREG         0     // Minimal analogue speed potmeter value
#define MAXREG         1023  // Maximal analogue potmeter value
#define DEFAULT_TRESH  450   // Detects >4.5 volt as HIGH
#define TRESHOLD_MAX   50    // Maximal level of trigger/detection level or theshold
#define DELAY_MAX      5000  // Maximal delay after sensor trigger or pulse width of monoflop in ms (0 ~ 5 sec)
#define SPEED_STEPS    30    // Step value for smooth motor speed
#define MAX_VALUE      255   // Used to output logic on the motor-/outputr-outputs
#define DIG_THRES      200   // Used to decide whether on analog input there is a zero or a one
#define Q1             8     // This pin is used either as an digital output 1 or as servo 1 control
#define Q2             7     // This pin is used either as an digital output 2 or as servo 2 control
#define Q3             4     // This pin is used either as an digital output 3 or as servo 3 control
#define Q4             17    // Independent digital output 4
#define SERVO4         9     // Independent servo 4 control

// Scan speed / response tuning
#define FAST_ADC              1     // (not used in this version of the sketch)
#define POT_READ_INTERVAL_MS  50    // (not used in this version of the sketch)
#define DISPLAY_INTERVAL_MS   100   // Minimum time between two screen refreshes. A refresh blocks the loop for ~25 ms, and only happens when something on screen changed

bool logging = true;          // Serial logging during development...
int currentOutputSpeed;
bool currentOutputDir = true; // true=forward(CW), false=backwards(CCW)
uint16_t potValue1;           // Value read from potentiometer 1
uint16_t potValue2;           // Value read from potentiometer 2
uint16_t potValue3;           // Value read from potentiometer 3
uint16_t potValue4;           // Value read from potentiometer 4
uint8_t sensorTresh1;         // Analog trigger threshold value of potentiometer 1
uint8_t sensorTresh2;         // Analog trigger threshold value of potentiometer 2
uint8_t sensorTresh3;         // Analog trigger threshold value of potentiometer 3
uint8_t sensorTresh4;         // Analog trigger threshold value of potentiometer 4
int timeDelay;                // Smooth motor delay or pulse width of monoflop in ms

// Initializing TB6612FNG driver and SSD1306 screen and SERVO('s)
Output Out1 = Output(AIN1, AIN2, PWMA, 1, STBY);
Output Out2 = Output(BIN1, BIN2, PWMB, 1, STBY);
Adafruit_SSD1306 display(128, 64, &Wire, -1);

ServoCh Servo4;                                       // Add Servo# to this line with ,'s

// Inputs
bool input1, input2, input3, input4, input5, input6;
bool input7, input8, input9, input10, input11, input12;
bool* inputs[12] = {
  &input1, &input2, &input3, &input4, &input5, &input6,
  &input7, &input8, &input9, &input10, &input11, &input12
};

// Custom input names, max 4 characters each
const char* inputNames[12] = {
  "IN1a", "IN2a", "IN3a", "IN4a",
  "IN1d", "IN2d", "IN3d", "IN4d",
  "IN5d", "IN6d", "IN7d", "IN8d"
};

// Outputs
bool output1, output2, output3, output4;
bool* outputs[4] = { &output1, &output2, &output3, &output4 };

// Potmeters (1..50)
uint8_t potvalue1 = 1, potvalue2 = 1, potvalue3 = 1, potvalue4 = 1;
uint16_t* pots[4] = { &potValue1, &potValue2, &potValue3, &potValue4 };

const char* functionName = "WIZARDY PRO";  // max 10 chars at size 2

// Layout (pixels)
const uint8_t INPUT_COLS    = 4;
const uint8_t INPUT_CELL_W  = 128 / INPUT_COLS;  // 32
const uint8_t INPUT_PITCH   = 10;                // 8 px text + underline
const uint8_t FUNC_Y        = 30;                // size 2 text, 14 px high
const uint8_t POT_Y         = 46;                // two rows, 8 px apart
const uint8_t POT_ROW_H     = 8;
const uint8_t OUT_Y         = 62;                // 2 px thick blocks
const uint8_t OUT_W         = 28;                // 4 x 32 px cells, 2 px margin

// ----------------------------------------------------------------------------------------------------------------

void setup() {

  if (logging) Serial.begin(38400);

  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.display();
  
//Assign servo

  servoAttach(Servo4, SERVO4, 67);            // call structure for this void: (ServoName, OutputPin, start Angle)

// Set pin-modes and read sensor defaults...
  pinMode(INA, INPUT);    // Multiplexed input for 4 analog signals
  pinMode(INB, INPUT);    // Multiplexed input for 4 digital signals
  pinMode(INC, INPUT);    // Multiplexed input for 4 digital signals
  pinMode(INPOT, INPUT);  // Multiplexed input for 4 potmeters
  pinMode(S1, OUTPUT);    // Multiplexer control signal 1
  pinMode(S2, OUTPUT);    // Multiplexer control signal 2
  pinMode(Q1, OUTPUT);    // Either output 1 or Servo 1. Disable this line in case of servo usage
  pinMode(Q2, OUTPUT);    // Either output 2 or Servo 1. Disable this line in case of servo usage
  pinMode(Q3, OUTPUT);    // Either output 3 or Servo 1. Disable this line in case of servo usage
  pinMode(Q4, OUTPUT);    // Output 4

}

void loop() {

  ReadInputs();
  updateScreen();         // this line should be last of the loop. It updates the display in case of changes

}

// ========== Display Functions ==========

void drawInputs() {
  int16_t x1, y1;
  uint16_t w, h;

  display.setTextSize(1);
  for (uint8_t i = 0; i < 12; i++) {
    uint8_t col = i % INPUT_COLS;
    uint8_t row = i / INPUT_COLS;

    char label[5];                          // Reserves 4 chars + terminator in RAM (5 bytes)
    strncpy(label, inputNames[i], 4);       // Copy max 4 characters in label with InputNames[i] as a source :: enforce the 4-character limit
    label[4] = '\0';                        // strncpy does add a terminator, this line ensures it has a terminator

    display.getTextBounds(label, 0, 0, &x1, &y1, &w, &h);

    int16_t x = col * INPUT_CELL_W + (INPUT_CELL_W - w) / 2;
    int16_t y = row * INPUT_PITCH;

    display.setCursor(x, y);
    display.print(label);

    if (*inputs[i]) {
      display.drawFastHLine(x, y + 8, w, SSD1306_WHITE);  // 1 px underline
    }
  }
}

void drawFunctionName() {
  display.setFont();          // default font
  display.setTextSize(1);

  const uint8_t PITCH = 8;    // 6 px glyph + 2 px extra spacing
  const int16_t Y = 34;       // top of text, centred in the gap (y=29..45)

  uint8_t n = strlen(functionName);
  int16_t total = n * PITCH - (PITCH - 5);   // last glyph has 5 px of ink
  int16_t x = (128 - total) / 2;

  for (uint8_t i = 0; i < n; i++) {           //Function name is printed letter by letter with predefined pitch
    display.setCursor(x + i * PITCH, Y);
    display.write(functionName[i]);
  }
}

void drawPots() {
  display.setTextSize(1);

  const int16_t LEFT_X  = 4;
  const int16_t RIGHT_X = 128 - 36;  // "Px: " + 2 digits = 6 chars x 6 px = 36 px -> x=92

  for (uint8_t i = 0; i < 4; i++) {
    bool top   = (i % 2 == 1);       // P2, P4 on the top row
    bool right = (i >= 2);           // P3, P4 in the right column

    int16_t x = right ? RIGHT_X : LEFT_X;
    int16_t y = POT_Y + (top ? 0 : POT_ROW_H);

    display.setCursor(x, y);
    display.print('P');
    display.print(i + 1);
    display.print(": ");
    display.print(map(*pots[i], MINREG, MAXREG, 1, TRESHOLD_MAX));
  }
}

void drawOutputs() {
  for (uint8_t i = 0; i < 4; i++) {
    if (*outputs[i]) {
      display.fillRect(i * 32 + 2, OUT_Y, OUT_W, 2, SSD1306_WHITE);
    }
  }
}

void drawScreen() {
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  drawInputs();
  drawFunctionName();
  drawPots();
  drawOutputs();
  display.display();
}

void updateScreen() {
  // Calls drawScreen() only when something visible has changed (inputs, outputs, displayed pot values)
  // AND at least DISPLAY_INTERVAL_MS has passed since the last check.
  // display.display() sends the full 1 KB buffer over I2C and blocks for ~25 ms, so while nothing changes the loop runs at full speed.
  static unsigned long lastCheck = 0;
  static uint16_t lastIO   = 0xFFFF;       // 12 input bits + 4 output bits, as last drawn
  static uint32_t lastPots = 0xFFFFFFFF;   // 4 displayed pot values, 6 bits each (max 24 bits used, so 0xFFFFFFFF = "never drawn" forces the first draw)

  unsigned long now = millis();
  if (now - lastCheck < DISPLAY_INTERVAL_MS) return;
  lastCheck = now;

  uint16_t io = 0;
  for (uint8_t i = 0; i < 12; i++) if (*inputs[i])  io |= 1u << i;
  for (uint8_t i = 0; i < 4;  i++) if (*outputs[i]) io |= 1u << (12 + i);

  uint32_t pv = 0;
  for (uint8_t i = 0; i < 4; i++) pv = (pv << 6) | map(*pots[i], MINREG, MAXREG, 1, TRESHOLD_MAX);

  if (io == lastIO && pv == lastPots) return;   // nothing changed: skip the 25 ms refresh
  lastIO = io;
  lastPots = pv;
  drawScreen();
}

// ========== END OF DISPLAY FUNCTIONS ==========

void ReadInputs() {
  // Reads potmeters and sets thesholds.
  // Reads inputs and decides logic value of input A based on threshold of potmeters
  // All inputs are multiplexed by using Arduino ports 12 and 13. The Multiplexer selector inputs are inverted compared to the Arduino port 12 and 13

  PORTB |= (1 << PORTB4) | (1 << PORTB5); //set multiplexers to select input 0

  potValue1 = analogRead(INPOT);
  sensorTresh1 = round10(map(potValue1, MINREG, MAXREG, 1, TRESHOLD_MAX)); 
  input1 = (analogRead(INA) > (sensorTresh1*10));
  input5 = digitalRead(INB);
  input9 = (analogRead(INC) > DIG_THRES);


  PORTB = (PORTB & ~((1 << PORTB4) | (1 << PORTB5))) | (1 << PORTB4); //set multiplexers to select input 1

  potValue2 = analogRead(INPOT);
  sensorTresh2 = round10(map(potValue2, MINREG, MAXREG, 1, TRESHOLD_MAX)); 
  input2 = (analogRead(INA) > (sensorTresh2*10));
  input6 = digitalRead(INB);
  input10 = (analogRead(INC) > DIG_THRES);

  PORTB = (PORTB & ~((1 << PORTB4) | (1 << PORTB5))) | (1 << PORTB5); //set multiplexers to select input 2

  potValue3 = analogRead(INPOT);
  sensorTresh3 = round10(map(potValue3, MINREG, MAXREG, 1, TRESHOLD_MAX)); 
  input3 = (analogRead(INA) > (sensorTresh3*10));
  input7 = digitalRead(INB);
  input11 = (analogRead(INC) > DIG_THRES);

  PORTB &= ~((1 << PORTB4) | (1 << PORTB5)); //set multiplexers to select input 3

  potValue4 = analogRead(INPOT);
  sensorTresh4 = round10(map(potValue4, MINREG, MAXREG, 1, TRESHOLD_MAX)); 
  input4 = (analogRead(INA) > (sensorTresh4*10));
  input8 = digitalRead(INB);
  input12 = (analogRead(INC) > DIG_THRES);

}

// ========== OUTPUT CONTROL (Q OUTPUTS) ==========

void setOutputs(bool OUT1, bool OUT2, bool OUT3, bool OUT4) {
  PORTB = (PORTB & ~(1 << PORTB0)) | ((!OUT1) << PORTB0);    //Set Q1 according to OUT1 => Switch off in case Q1 is used for SERVO
  PORTD = (PORTD & ~(1 << PORTD7)) | ((!OUT2) << PORTD7);    //Set Q2 according to OUT2 => Switch off in case Q2 is used for SERVO
  PORTD = (PORTD & ~(1 << PORTD4)) | ((!OUT3) << PORTD4);    //Set Q3 according to OUT3 => Switch off in case Q3 is used for SERVO
  PORTC = (PORTC & ~(1 << PORTC3)) | ((!OUT4) << PORTC3);    //Set Q4 according to OUT4
}

// ========== END OF OUTPUT CONTROL ==========

// ========== SERVO CONTROL FUNCTIONS ==========

// Generic: move any servo, skipping the write when the angle has not changed
void servoControl(ServoCh &ch, uint16_t angle) {
  if (angle == ch.last) return;
  ch.last = angle;
  ch.servo.write(angle);
}

// Generic: write the rest angle first, then attach (no twitch at power-up)
void servoAttach(ServoCh &ch, uint8_t pin, uint16_t restAngle) {
  servoControl(ch, restAngle);
  ch.servo.attach(pin);
}

// ========== END OF SERVO CONTROL FUNCTIONS ==========

// ========== MISCELANEOUS FUNCTIONS ==========

void logSerial(String message) {
  // Handled overhere so logging to serial can easily be switched off...
  if (logging) Serial.println(message);
  //return;
}

unsigned long round10(unsigned long n) {
  return (n/10 + (n%10>4)) * 10;
}

// ========== END OF MISCELANEOUS FUNCTIONS ==========
