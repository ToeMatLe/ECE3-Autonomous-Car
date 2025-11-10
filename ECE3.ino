#include <ECE3.h>
uint16_t sensorValues[8]; // right -> left, 0 -> 7

// Motor Constants
// left side
const int left_nslp_pin=31; // nslp ==> awake & ready for PWM
const int left_dir_pin=29;
const int left_pwm_pin=40;
// right side
const int right_nslp_pin=11; // nslp ==> awake & ready for PWM
const int right_dir_pin=30;
const int right_pwm_pin=39;

// Minimum Constants
int min1 = 734;
int min2 = 619;
int min3 = 664;
int min4 = 596;
int min5 = 619;
int min6 = 664;
int min7 = 711;
int min8 = 734;
const int min[8] = {min1, min2, min3, min4,
                          min5, min6, min7, min8};
// Maximum Constants
int max1 = 1766;
int max2 = 1881;
int max3 = 1836;
int max4 = 1904;
int max5 = 1881;
int max6 = 1836;
int max7 = 1789;
int max8 = 1766;
const int max[8] = {max1, max2, max3, max4,
                          max5, max6, max7, max8};

// Phototransisters Constants (Left -> Right)
int photoPin1 = 65; 
int photoPin2 = 48;
int photoPin3 = 64;
int photoPin4 = 47;
int photoPin5 = 52;
int photoPin6 = 68;
int photoPin7 = 53;
int photoPin8 = 69;
const int photoPins[8] = {photoPin1, photoPin2, photoPin3, photoPin4,
                          photoPin5, photoPin6, photoPin7, photoPin8};

// IR Emitters Constants
const int IR_LED_odd = 45;
const int IR_LED_even = 61;

//Base Speed
int leftSpd = 0;
int rightSpd = 0;

// PID (arbutarity 50)
// if kp is negative, speed left motor, slow down right motor
const int kp = 0.01;
const int kd = 0.1;
int error = 0; // Some value from fusion output we will use?
int prevError = 0;


// Define the weighting of the pins using (15-14-12-8)/8 weighting
// Thought Process is that you add up the photopin values, left as negative, right as positive
int photoWeight1 = -15;
int photoWeight2 = -14;
int photoWeight3 = -12;
int photoWeight4 = -8;
int photoWeight5 = 8;
int photoWeight6 = 12;
int photoWeight7 = 14;
int photoWeight8 = 15;
const int photoWeight[8] = {photoWeight1, photoWeight2, photoWeight3, photoWeight4,
                          photoWeight5, photoWeight6, photoWeight7, photoWeight8};

///////////////////////////////////
void setup() {
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
  
}

void loop() {
  ECE3_read_IR(sensorValues);

  int sensorSum = 0;
  int min = min[i];
  int max = max[i]
  for (int i = 0; i < 8; i++) {
    sensorValues[i] -= min;
    if (sensorValues[i] < 0) sensorValues[i] = 0;
    sensorValues[i] *= 1000;
    sensorValues[i] /= max;
    sensorValues[i] *= photoWeight[i];
    sensorSum += sensorValues[i];
  }
  error = sensorSum/8;
  Serial.print(error);

  int diffSum = error - prevError;
  int pidSum = kp*error + kd*diffSum;

  leftSpd += pidSum;
  rightSpd -= pidSum;
  analogWrite(left_pwm_pin,leftSpd);
  analogWrite(right_pwm_pin,rightSpd);  

  prevError = error;
  }
