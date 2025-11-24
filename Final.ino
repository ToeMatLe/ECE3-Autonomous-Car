#include <ECE3.h>
uint16_t sensorValues[8]; // right -> left, 0 -> 7

// Motor Constants
// left side
const int left_nslp_pin=31; 
const int left_dir_pin=29;
const int left_pwm_pin=40;
// right side
const int right_nslp_pin=11; 
const int right_dir_pin=30;
const int right_pwm_pin=39;

//light debugg pins
const int yellowled = 51;

// Minimum Constants
const int min[8] = {734, 619, 664, 596, 619, 664, 711, 734};
// Maximum Constants
const int max[8] = {1766, 1881, 1836, 1904, 1881, 1836, 1789, 1766};

// IR Emitters Constants
const int IR_LED_odd = 45;
const int IR_LED_even = 61;

//Base Speed
int leftSpd = 0;
int rightSpd = 0;
int baseSpeed = 29;

// PID (arbutarity 50)
// if kp is negative, speed left motor, slow down right motor
int kp = 820;
int kd = -12;
int prevError = 0;

// Define the weighting of the pins using (15-14-12-8)/8 weighting
// Thought Process is that you add up the photopin values, left as negative, right as positive

// slow down speed, more room for correction so all 4 starting positions will work
int photoWeight[8] = {-15, -14, -12, -8, 8, 12, 15, 19};

// Variables to caclulate error in sensor fusion
int error;
int sensorSum;
int result; 

bool isOnLine();
int lineCount = 0;
int phantomDetect = 0;

// int TURN_TIME = 1200;  // ms you measured for ~225°
bool turning = false;
long turnStartLeft = 0;
long turnStartRight = 0;
// You must calibrate this value!
const long TURN_COUNTS_225 = 605; // <-- adjust after testing

bool stopByEncoder = false;
long stopStartLeft = 0;
long stopStartRight = 0;
// You must calibrate this too
const long STOP_COUNTS = 1000;   // <-- adjust after testing

///////////////////////////////////////////////////////////////////////////////////////////////////////////
void setup() {
  ECE3_Init();
// left
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
// right
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);  
// ir emmiters
  pinMode(IR_LED_odd, OUTPUT);
  pinMode(IR_LED_even, OUTPUT);
  digitalWrite(IR_LED_odd, HIGH);
  digitalWrite(IR_LED_even, HIGH);
  
//LED 
  pinMode(yellowled, OUTPUT);
  digitalWrite(yellowled, LOW);

  Serial.begin(9600);
  delay(1000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  // --- PID computation as before ---
  result = 0;
  error = 0;
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    result = sensorValues[i] - min[i];
    result *= 1000;
    result /= (max[i]-min[i]);
    if (result < 0) result = 0;
    result *= photoWeight[i];
    sensorSum += result;
  }
  error = sensorSum/8;

  int diffSum = error - prevError;
  int pidSum = kp*error/100000 + kd*diffSum/10000;

  leftSpd  = baseSpeed - pidSum;
  rightSpd = baseSpeed + pidSum;
  analogWrite(left_pwm_pin,  leftSpd);
  analogWrite(right_pwm_pin, rightSpd); 
  
  prevError = error;
  
  // --- If we are currently turning, keep turning until TURN_TIME elapsed ---
  if (turning && lineCount == 1) {
  // how far have we turned? use one wheel or the average
  long leftDelta  = labs(getLeftCounts()  - turnStartLeft);
  long rightDelta = labs(getRightCounts() - turnStartRight);
  long turnDelta  = (leftDelta + rightDelta) / 2;  // or just use one side

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

    // adjust weights
    photoWeight[0] = -15;
    photoWeight[1] = -8;
    photoWeight[2] = -6;
    photoWeight[3] = -10;
    photoWeight[4] =  10;
    photoWeight[5] =  14;
    photoWeight[6] =  16;
    photoWeight[7] =  18;
    baseSpeed = 35;
    turning = false;
  }
}
if (stopByEncoder) {
    long leftStopDelta  = labs(getLeftCounts()  - stopStartLeft);
    long rightStopDelta = labs(getRightCounts() - stopStartRight);
    long stopDelta      = (leftStopDelta + rightStopDelta) / 2;

    if (stopDelta >= STOP_COUNTS) {
      // Stop car
      digitalWrite(left_dir_pin,  HIGH);  // backward
      digitalWrite(right_dir_pin, LOW);   // forward
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255);
      delay(100);
      digitalWrite(left_dir_pin,  LOW);  // backward
      digitalWrite(right_dir_pin, LOW);   // forward
      analogWrite(left_pwm_pin,  255);
      analogWrite(right_pwm_pin, 255);
      delay(50);
      analogWrite(left_pwm_pin,  0);
      analogWrite(right_pwm_pin, 0);
      digitalWrite(yellowled, HIGH); // indicate finished

      // Stop forever
      while (true) {
        // do nothing
      }
    }
    return;
  }

  // --- Handle line detection only when not turning ---
 if (!turning && isOnLine()) {
  // stop briefly on the line
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    if (isOnLine()) {
      phantomDetect += 1;
    } else {
      phantomDetect = 0;
    }
    
    // start turn
    if (phantomDetect >= 2 && lineCount == 0){ // Hits cc 225 turn
      lineCount += 1;
      phantomDetect = 0;
      turning = true;
    } else if(phantomDetect >= 3 && lineCount == 1) { // start timed run
      // Change weights back to normal, delay to check
      lineCount += 1;
      photoWeight[0] = -15;
      photoWeight[1] = -10;
      photoWeight[2] = -12;
      photoWeight[3] = -8;
      photoWeight[4] = 8;
      photoWeight[5] = 12;
      photoWeight[6] = 14;
      photoWeight[7] = 15;
      analogWrite(left_pwm_pin,  50);
      analogWrite(right_pwm_pin, 50);
      delay(15);
      digitalWrite(yellowled, HIGH);
      baseSpeed = 25;
      phantomDetect = 0;
    }  else if(phantomDetect >= 6 && lineCount == 2) { // end timed run
      // Change weights back to normal, delay to check
      lineCount += 1;
      photoWeight[0] = -0;
      photoWeight[1] = -14;
      photoWeight[2] = -10;
      photoWeight[3] = -8;
      photoWeight[4] = 8;
      photoWeight[5] = 12;
      photoWeight[6] = 14;
      photoWeight[7] = 30;
      digitalWrite(yellowled, LOW);
      analogWrite(left_pwm_pin,  50);
      analogWrite(right_pwm_pin, 50);
      delay(15);
      baseSpeed = 100;
      phantomDetect = 0;
      stopByEncoder = true;
      stopStartLeft  = getLeftCounts();
      stopStartRight = getRightCounts();
    }
    return; // next loop iteration will enter the "turning" block
  }
}
/////////////////////////////////////////////////////////////////
bool isOnLine() {
  if (sensorValues[0] > 2450 && sensorValues[1] > 2450 && sensorValues[2] > 2450 && sensorValues[3] > 2450 &&
      sensorValues[4] > 2450 && sensorValues[5] > 2450 && sensorValues[6] > 2450 && sensorValues[7] > 2450) {
    return true;
  } else {
    return false;
  }
}
long getLeftCounts() {
  // TODO: replace with your ECE3 encoder read function
  return getEncoderCount_left();

}

long getRightCounts() {
  // TODO: replace with your ECE3 encoder read function
  return getEncoderCount_right();

}
