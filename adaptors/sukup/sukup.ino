/*
  AutoTill 2
  Electro-Hydraulic PWM Controller
  Developed by Trevor Stanhope
  Receives serial commands to adjust the hydraulics with user-inputted sensitivity settings
*/

/* --- Definitions --- */
#define REFERENCE_PIN 5
#define CONTROL_PIN 6 // 255 corresponds to reaction at max negative offset

/* --- Constants --- */
const unsigned long BAUD = 9600;
const unsigned int INPUT_LOW = 0;
const unsigned int INPUT_HIGH = 100;
const unsigned int PWM_LOW = 2; // 0.10 V
const unsigned int PWM_HIGH = 210; // 8.00 V
const unsigned int PWM_MAX = 255; // 9.70 V

/* --- Variables --- */
volatile int DUTY = 127; // effective range of 0 to 255
volatile int PWM_CONTROL = 127; // neutral at 127
volatile int PWM_REFERENCE = PWM_MAX; // neutral at 127

void setup(void) {
    pinMode(CONTROL_PIN, OUTPUT);
    pinMode(REFERENCE_PIN, OUTPUT);
    Serial.begin(BAUD);
}

void loop(void) {
    DUTY = Serial.parseInt();
    PWM_CONTROL = map(DUTY, PWM_LOW, PWM_HIGH, INPUT_LOW, INPUT_HIGH); // TODO
    PWM_REFERENCE = PWM_MAX; 
    analogWrite(CONTROL_PIN, PWM_CONTROL);
    analogWrite(REFERENCE_PIN, PWM_REFERENCE);
}
