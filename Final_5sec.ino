#include <ECE3.h>

uint16_t sensorValues[8]; // right -> left, 0 -> 7

// Motor Constants
// left side
const int left_nslp_pin = 31; 
const int left_dir_pin  = 29;
const int left_pwm_pin  = 40;
// right side
const int right_nslp_pin = 11; 
const int right_dir_pin  = 30;
const int right_pwm_pin  = 39;

// Light debug pin
const int yellowled = 51;

// Minimum Constants
const int min[8] = {619, 596, 596, 596, 605, 712, 783, 807};
// Maximum Constants
const int max[8] = {1881, 1904, 1904, 1904, 1895, 1788, 1717, 1693};

// IR Emitters Constants
const int IR_LED_odd  = 45;
const int IR_LED_even = 61;

// Base Speed
int leftSpd   = 0;
int rightSpd  = 0;
int baseSpeed = 70;

// PID
// if kp is negative, speed left motor, slow down right motor
int kp        = 1200;
int kd        = 0;
int prevError = 0;

// Weighting for sensors (left negative, right positive)
int photoWeight[8] = {-17, -16, -12, -12, 12, 12, 16, 17};

// Variables to calculate error
int error;
int sensorSum;
int result; 

bool isOnLine();
int lineCount      = 0;
int phantomDetect  = 0;

// Turning state
bool turning        = false;
long turnStartLeft  = 0;
long turnStartRight = 0;
// You must calibrate this value!
const long TURN_COUNTS_225 = 380; // <-- adjust after testing

// Stop-by-encoder state
bool stopByEncoder   = false;
long stopStartLeft   = 0;
long stopStartRight  = 0;
// You must calibrate this too
const long STOP_COUNTS = 1000;   // <-- adjust after testing

bool donut   = false;
long donutStartLeft   = 0;
long donutStartRight  = 0;
int donutCount = 0;
const long DONUT_COUNTS1 = 150;   // <-- adjust after testing
const long DONUT_COUNTS2 = 358 + DONUT_COUNTS1;   // <-- adjust after testing
const long DONUT_COUNTS3 = 1088 + DONUT_COUNTS2;   // <-- adjust after testing

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  ECE3_Init();

  // left motor
  pinMode(left_nslp_pin, OUTPUT);
  pinMode(left_dir_pin,  OUTPUT);
  pinMode(left_pwm_pin,  OUTPUT);
  digitalWrite(left_dir_pin,  LOW);
  digitalWrite(left_nslp_pin, HIGH);

  // right motor
  pinMode(right_nslp_pin, OUTPUT);
  pinMode(right_dir_pin,  OUTPUT);
  pinMode(right_pwm_pin,  OUTPUT);
  digitalWrite(right_dir_pin,  LOW);
  digitalWrite(right_nslp_pin, HIGH);  

  // IR emitters
  pinMode(IR_LED_odd,  OUTPUT);
  pinMode(IR_LED_even, OUTPUT);
  digitalWrite(IR_LED_odd,  HIGH);
  digitalWrite(IR_LED_even, HIGH);
  
  // LED 
  pinMode(yellowled, OUTPUT);
  digitalWrite(yellowled, LOW);

  Serial.begin(9600);
  delay(2000);
}

void loop() {
  // Read sensors
  ECE3_read_IR(sensorValues);

   // ------------------------- TURNING PHASE -------------------------
  if (turning && lineCount == 1) {
    long leftDelta  = labs(getLeftCounts()  - turnStartLeft);
    long rightDelta = labs(getRightCounts() - turnStartRight);
    long turnDelta  = (leftDelta + rightDelta) / 2;  // average or just one side

    if (turnDelta < TURN_COUNTS_225) {
      // still turning: spin in place
      digitalWrite(left_dir_pin,  HIGH);  // backward
      digitalWrite(right_dir_pin, LOW);   // forward

      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255);
      return; // skip rest of loop while turning
    } else {
      // done turning, go forward again

      digitalWrite(left_dir_pin,  LOW);
      digitalWrite(right_dir_pin, LOW);

      // adjust weights AFTER turn
      photoWeight[0] = -15;
      photoWeight[1] = -15;
      photoWeight[2] = -3;
      photoWeight[3] = -6;
      photoWeight[4] =  6;
      photoWeight[5] =  12;
      photoWeight[6] =  13;
      photoWeight[7] =  15;

      baseSpeed = 55;
      kp        = 990;
      turning   = false;
      // continue to PID + line logic in next loop iteration
    }
  }
  // ------------------------- DONUT PHASE -------------------------
  if (donut) {
    long leftDonutDelta  = labs(getLeftCounts()  - donutStartLeft);
    long rightDonutDelta = labs(getRightCounts() - donutStartRight);
    long donutDelta      = (leftDonutDelta + rightDonutDelta) / 2;
    
    if (donutDelta < DONUT_COUNTS1) {
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255); 
      return;
    } else if (donutDelta < DONUT_COUNTS2) {
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 0); 
      return;
    } else if (donutDelta < DONUT_COUNTS3) {
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255); 
      return;
    } else {
      // "Brake" / stop car with a little reverse/forward wiggle
      digitalWrite(left_dir_pin,  HIGH);  // backward
      digitalWrite(right_dir_pin, LOW);   // forward
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255);
      delay(110);

      digitalWrite(left_dir_pin,  LOW);   // forward (or your actual direction)
      digitalWrite(right_dir_pin, LOW);
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255);
      delay(60);

      analogWrite(left_pwm_pin,  0);
      analogWrite(right_pwm_pin, 0);
      digitalWrite(yellowled, HIGH); // indicate finished

      // Stop forever
      while (true) {
        // do nothing
      }
    }
  }
  // ------------------------- NORMAL PID LINE FOLLOWING -------------------------
  // Only do PID if we’re not currently turning
  if (!turning) {
    sensorSum = 0;

    for (int i = 0; i < 8; i++) {
      result = sensorValues[i] - min[i];
      result *= 1000;
      result /= (max[i] - min[i]);

      if (result < 0) result = 0;

      result *= photoWeight[i];
      sensorSum += result;
    }

    error = sensorSum / 8;

    int diffSum = error - prevError;
    int pidSum  = kp * error / 100000 + kd * diffSum / 10000;

    // Constrain speeds to valid PWM range
    leftSpd  = constrain(baseSpeed - pidSum, 0, 255);
    rightSpd = constrain(baseSpeed + pidSum, 0, 255);

    analogWrite(left_pwm_pin,  leftSpd);
    analogWrite(right_pwm_pin, rightSpd); 
  
    prevError = error;
  }

  // ------------------------- LINE DETECTION / STATE MACHINE -------------------------
  if (!turning) {
    if (isOnLine()) {
      phantomDetect += 1;
    } else {
      phantomDetect = 0;
    }
    
    // 1st cross: start 225° turn
    if (phantomDetect >= 2 && lineCount == 0) { 
      lineCount      += 1;
      phantomDetect   = 0;
      turning         = true;
      // IMPORTANT: record encoder start counts for turn
      turnStartLeft   = getLeftCounts();
      turnStartRight  = getRightCounts();
      return;
    } 
    // 2nd cross: start timed run
    else if (phantomDetect >= 2 && lineCount == 1) {
      lineCount += 1;

      // Adjust weights for straight run
      photoWeight[0] = -16;
      photoWeight[1] = -14;
      photoWeight[2] = -12;
      photoWeight[3] = -9;
      photoWeight[4] =  9;
      photoWeight[5] =  12;
      photoWeight[6] = 14;
      photoWeight[7] = 16;

      digitalWrite(yellowled, HIGH);

      baseSpeed      = 50;
      phantomDetect  = 0;
      donut = true;
      // IMPORTANT: record encoder start counts for turn
      donutStartLeft   = getLeftCounts();
      donutStartRight  = getRightCounts();
      return;
    }  
  }
}

/////////////////////////////////////////////////////////////////
bool isOnLine() {
  int count = 0;
  const int threshold = 2450; // <-- tune this

  for (int i = 0; i <= 7; i++) {
    if (sensorValues[i] > threshold) {
      count++;
    }
  }
  return (count >= 5); // at least 3 sensors see black
}


long getLeftCounts() {
  // ECE3 encoder read
  return getEncoderCount_left();
}

long getRightCounts() {
  // ECE3 encoder read
  return getEncoderCount_right();
}
