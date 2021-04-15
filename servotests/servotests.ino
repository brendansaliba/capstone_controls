/* Servo motor with Arduino example code. Position and sweep. More info: https://www.makerguides.com/ */
// Include the servo library:
#include <Servo.h>

// Create a new servo object:
Servo escRight, escLeft, tiltRight, tiltLeft, escRear, aileronServo, elevatorServo, throttleServo, rudderServo;

// LMAO FUCK THIS SHIT
float dt, tiltRaw2;
float HERTZ = 2000;
unsigned long current_time, prev_time;

// Define the servo pin:
#define escRightPin 22
#define escLeftPin 24
#define tiltRightPin 30
#define tiltLeftPin 34
#define aileronPin 52
#define elevatorPin 46
#define throttlePin 49
#define rudderPin 37 //50
#define escRearPin 26
#define tiltPin 42

float THIINGY = 2;

float aileronRaw; // channel 1
float elevatorRaw; // channel 2
float throttleRaw; // channel 3
float rudderRaw; // channel 4
float tiltRaw; // channel 5 (SG Switch 3pos)

// Create a variable to store the servo position:
float rightTiltSetting_PWM, leftTiltSetting_PWM;

void setup() {
  // Attach the Servo variable to a pin:

  Serial.begin(500000);
//  escRight.attach(escRightPin);
//  escLeft.attach(escLeftPin);
  tiltRight.attach(tiltRightPin);
  tiltLeft.attach(tiltLeftPin);
//  escRear.attach(escRearPin);
//  aileronServo.attach(aileronPin);
//  throttleServo.attach(throttlePin);
//  rudderServo.attach(rudderPin);
//  elevatorServo.attach(elevatorPin);
  
  rightTiltSetting_PWM = 1020;
  leftTiltSetting_PWM = 2000;
  setupPins();

  tiltRaw2 = 1000;

//  aileronServo.writeMicroseconds(1500); // rudder (min 1000 (arm forward), max 1800 (arm aft))
//  rudderServo.writeMicroseconds(1500); // elevator (min 1250 (arm forward), max 1750 (arm aft))
//  elevatorServo.writeMicroseconds(1500); // right aileron (min 1100 (arm forward), max 1900 (arm aft))
////  throttleServo.writeMicroseconds(1000); // left aileron (min 1100 (arm aft), max 1800 (arm forward))
  tiltRight.writeMicroseconds(rightTiltSetting_PWM); // right tilt (min 1020 (horizontal), max 2000 (vertical))
  tiltLeft.writeMicroseconds(leftTiltSetting_PWM);

  delay(1000);
}

void loop() {
  // The fourth dimension
  prev_time = current_time;      
  current_time = micros();      
  dt = (current_time - prev_time)/1000000.0;

  //Check Tilt Condition
  tiltRaw2 = pulseIn(tiltPin, HIGH);

  //Check for flight condition
  if (tiltRaw2 > 1500){ // titRaw may have some stupid stuff with the throttle.... check later maybe?
    rightTiltSetting_PWM = floatFaderLinear(rightTiltSetting_PWM, 1020, 2000, THIINGY, 1, HERTZ); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
    leftTiltSetting_PWM = floatFaderLinear(leftTiltSetting_PWM, 1020, 2000, THIINGY, 0, HERTZ);
  }
  if (tiltRaw2 < 1500) { //go to min specified value in 2.5 seconds
    rightTiltSetting_PWM = floatFaderLinear(rightTiltSetting_PWM, 1020, 2000, THIINGY, 0, HERTZ); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
    leftTiltSetting_PWM = floatFaderLinear(leftTiltSetting_PWM, 1020, 2000, THIINGY, 1, HERTZ);  
  }

//  rightTiltSetting_PWM = floatFaderLinear(rightTiltSetting_PWM, 1020, 2000, 2, 1, 2000); //parameter, minimum value, maximum value, fadeTime (seconds), state (0 min or 1 max), loop frequency
//  leftTiltSetting_PWM = floatFaderLinear(leftTiltSetting_PWM, 1020, 2000, 2, 0, 2000);
//  
  tiltRight.writeMicroseconds(rightTiltSetting_PWM);
  tiltLeft.writeMicroseconds(leftTiltSetting_PWM);
//
//
  aileronRaw = pulseIn(aileronPin, HIGH);
  elevatorRaw = pulseIn(elevatorPin, HIGH);
  rudderRaw = pulseIn(rudderPin, HIGH);
  throttleRaw = pulseIn(throttlePin, HIGH);
  tiltRaw = pulseIn(tiltPin, HIGH);

  Serial.print("aileron: ");
  Serial.print(aileronRaw);
  Serial.print(" elevator: ");
  Serial.print(elevatorRaw);
  Serial.print(" rudder: ");
  Serial.print(rudderRaw);
  Serial.print(" throttle: ");
  Serial.print(throttleRaw);
  Serial.print(" TILT: ");
  Serial.println(tiltRaw);
//  // maintain loop rate at 2000 hz
  loopRate(HERTZ);
  
}


float floatFaderLinear(float param, float param_min, float param_max, float fadeTime, int state, int loopFreq){
  //DESCRIPTION: Linearly fades a float type variable between min and max bounds based on desired high or low state and time
  /*  
   *  Takes in a float variable, desired minimum and maximum bounds, fade time, high or low desired state, and the loop frequency 
   *  and linearly interpolates that param variable between the maximum and minimum bounds. This function can be called in controlMixer()
   *  and high/low states can be determined by monitoring the state of an auxillarly radio channel. For example, if channel_6_pwm is being 
   *  monitored to switch between two dynamic configurations (hover and forward flight), this function can be called within the logical 
   *  statements in order to fade controller gains, for example between the two dynamic configurations. The 'state' (1 or 0) can be used
   *  to designate the two final options for that control gain based on the dynamic configuration assignment to the auxillary radio channel.
   *  
   */
  float diffParam = (param_max - param_min)/(fadeTime*loopFreq); //difference to add or subtract from param for each loop iteration for desired fadeTime

  if (state == 1) { //maximum param bound desired, increase param by diffParam for each loop iteration
    param = param + diffParam;
  }
  else if (state == 0) { //minimum param bound desired, decrease param by diffParam for each loop iteration
    param = param - diffParam;
  }

  param = constrain(param, param_min, param_max); //constrain param within max bounds
  
  return param;
}

void loopRate(int freq) {
  //DESCRIPTION: Regulate main loop rate to specified frequency in Hz
  /*
   * It's good to operate at a constant loop rate for filters to remain stable and whatnot. Interrupt routines running in the
   * background cause the loop rate to fluctuate. This function basically just waits at the end of every loop iteration until 
   * the correct time has passed since the start of the current loop for the desired loop rate in Hz. 2kHz is a good rate to 
   * be at because the loop nominally will run between 2.8kHz - 4.2kHz. This lets us have a little room to add extra computations
   * and remain above 2kHz, without needing to retune all of our filtering parameters.
   */
  float invFreq = 1.0/freq*1000000.0;
  unsigned long checker = micros();
  
  //Sit in loop until appropriate time has passed
  while (invFreq > (checker - current_time)) {
    checker = micros();
  }
}

void setupPins() {
  pinMode(aileronPin, INPUT);
  pinMode(throttlePin, INPUT);
  pinMode(elevatorPin, INPUT);
  pinMode(rudderPin, INPUT);
  pinMode(tiltPin, INPUT);
}
