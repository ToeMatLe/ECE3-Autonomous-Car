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

// Minimum Constants
const int min[8] = {734, 619, 664, 596, 619, 664, 711, 734};
// Maximum Constants
const int max[8] = {1766, 1881, 1836, 1904, 1881, 1836, 1789, 1766};

// Phototransisters Constants (Left -> Right)
const int photoPins[8] = {65, 48, 64, 47, 52, 68, 53, 69};

// IR Emitters Constants
const int IR_LED_odd = 45;
const int IR_LED_even = 61;

//Base Speed
int leftSpd = 0;
int rightSpd = 0;
int baseSpeed = 25;

// PID (arbutarity 50)
// if kp is negative, speed left motor, slow down right motor
int kp = 800;
int kd = 12;
int prevError = 0;

// Define the weighting of the pins using (15-14-12-8)/8 weighting
// Thought Process is that you add up the photopin values, left as negative, right as positive
int photoWeight[8] = {-15, -14, -12, -8, 8, 12, 14, 15};

// Variables to caclulate error in sensor fusion
int error;
int sensorSum;
int result; 

bool isOnLine();
int lineCount = 0;
int phantomDetect = 0;
const int TURN_TIME = 700;  // ms you measured for ~225°
bool turning = false;
int turnStart = 0;

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
  
  Serial.begin(9600);
  delay(1000);
}

void loop() {
  ECE3_read_IR(sensorValues);
	
  error = 0;
  sensorSum = 0;
  for (int i = 0; i < 8; i++) {
    result = sensorValues[i] - min[i];
    result *= 1000;
    result /= max[i];
    if (result < 0) result = 0;
    result *= photoWeight[i];
    sensorSum += result;
  }
  error = sensorSum/8;

  int diffSum = error - prevError;
  int pidSum = kp*error/100000 + kd*diffSum/10000;

  leftSpd = baseSpeed - pidSum;
  rightSpd = baseSpeed + pidSum;
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);  
  
  prevError = error;
  
// If we are currently turning, keep turning until TURN_TIME elapsed
  if (turning && lineCount == 1) {
    if (millis() - turnStart < TURN_TIME) {
      // spin in place: left backward, right forward
      digitalWrite(left_dir_pin, HIGH);   // adjust if your wiring is opposite
      digitalWrite(right_dir_pin, LOW);

      analogWrite(left_pwm_pin,  140);
      analogWrite(right_pwm_pin, 140);
      return; // skip rest of loop while turning
    } else {
      digitalWrite(left_dir_pin, LOW);  
      digitalWrite(right_dir_pin, LOW);
      //Give right weighting less power
      photoWeight[0] = -15;
      photoWeight[1] = -14;
      photoWeight[2] = -12;
      photoWeight[3] = -15;
      photoWeight[4] = 15;
      photoWeight[5] = 14;
      photoWeight[6] = 20;
      photoWeight[7] = 24;
      turning = false;
    }
  }

  // If we're NOT already turning, check for line hit
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
    if (phantomDetect >= 2 && lineCount == 0){
      lineCount += 1;
      phantomDetect = 0;
      turning = true;
      turnStart = millis();
    } else if(phantomDetect >= 2 && lineCount == 1) {
      // Change weights back to normal, delay to check
      phantomDetect = 0;
      lineCount += 1;
      kp = 700;
      photoWeight[0] = -15;
      photoWeight[1] = -14;
      photoWeight[2] = -12;
      photoWeight[3] = -8;
      photoWeight[4] = 8;
      photoWeight[5] = 12;
      photoWeight[6] = 14;
      photoWeight[7] = 15;
      delay(3000);
    }
    return; // next loop iteration will enter the "turning" block
  }
}
///////////////////////////////////////////////////////////////////////////////////////////////
bool isOnLine() {
	for (int i = 0; i < 8; i++) {
    //delay(1);
    if (sensorValues[i] < 2450) {
      return false;
    }
  }
  return true;
}
