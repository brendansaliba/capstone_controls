#include <Servo.h>

// ----------------------------------- DEFINE THINGS
// -------------------------------------------------

// servo objects
Servo rearESC;
Servo mainESC1;
Servo mainESC2;
Servo tiltServo1;
Servo tiltServo2;

// raw control input variables
float aileronRaw; // channel 1
float elevatorRaw; // channel 2
float throttleRaw; // channel 3
float rudderRaw; // channel 4
float tiltRaw; // channel 5 (SG Switch 3pos)
float tiltRaw2; // channel 6 (same as channel 5?????)

// smoothed control inputs
float aileronInput;
float elevatorInput;
float throttleInput;
float rudderInput;
float tiltInput;
float tiltInput2;
float ESCTest;


// ----------------------------------- START HERE
// ----------------------------------------------
void setup() {
  // put your setup code here, to run once:
  // serial
  Serial.begin(9600); 
  
  initMovingAvgObjects();
  pinModeSetupInput();  
}


// ----------------------------------- THE BIG LOOPER
// --------------------------------------------------
void loop() {
  readRawControlInputs();
  setupMotorControllers();

  rearESC.write(throttleInput+1500); // test esc output pin 9
  
  // ESCTest = pulseIn(8, HIGH); // reading test esc output
  Serial.print(throttleInput+1500);
  Serial.println();
  delay(10);
}


// ----------------------------------- FUNCTIONS
// ---------------------------------------------
void pinModeSetupInput() {
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  pinMode(6, INPUT);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
}

void pinModeSetupOutput() {
  // setup the output pins
}

void readRawControlInputs() {
  aileronRaw = pulseIn(2, HIGH);
  elevatorRaw = pulseIn(3, HIGH);
  throttleRaw = pulseIn(4, HIGH);
  rudderRaw = pulseIn(5, HIGH);
  tiltRaw = pulseIn(6, HIGH);
  tiltRaw2 = pulseIn(7, HIGH);

  aileronInput = constrain(smoothInput1.reading(aileronRaw) - 1500, -500, 500);
  elevatorInput = constrain(smoothInput2.reading(elevatorRaw) - 1500, -500, 500);
  throttleInput = constrain(smoothInput3.reading(throttleRaw) - 1500, -500, 500);
  rudderInput = constrain(smoothInput4.reading(rudderRaw) - 1500, -500, 500);
  tiltInput = constrain(smoothInput5.reading(tiltRaw) - 1500, -500, 500);
  tiltInput2 = constrain(smoothInput6.reading(tiltRaw2) - 1500, -500, 500);
}

void setupMotorControllers() {
  rearESC.attach(9, 1000, 2000); // set to pin 9
}
