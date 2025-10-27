#define Idle 0
#define Photometry 1
#define Control 2

#include <PID_v1.h>


// PID params
double input, output, target, control;
int Kp = 10;
int Ki = 0;
int Kd = 100;
double minPIDOutput = 0;
double maxPIDOutput = 255;

// Photometry processing params
double PhotometryWindow = 5; // in ms
double PIDSampleTime = 0; // in ms
int MaxFreq = 100;
double signal = 0;
double PhotometrySum = 0;

// Set up pins in arduino
const byte InputPin = A0;
const byte OutputPin = 13;
const byte TargetPin = 3;

// Baseline related variables
unsigned long BaselineSumInSecond = 0;
unsigned long BaselineSumInWindow = 0;
int nBaselineSample = 0;
int nBaselineWindow = 0;

// Misc initialization stuff
char SerialInput = '0';
static int state = 0;
unsigned long Start = 0;
unsigned long End = 0;
unsigned long nSample = 1;

unsigned long lastInput = 0;
unsigned long dInput = 0;

// Setup PID
PID myPID(&input, &output, &target, Kp, Ki, Kd, REVERSE);

void setup() {
  Serial.begin(115200);
  pinMode(InputPin, INPUT);
  pinMode(OutputPin, OUTPUT);
  pinMode(TargetPin, INPUT);

  signal = analogRead(InputPin);
  target = 0; //analogRead(TargetPin);
  state = 0;

  // turn PID on
  myPID.SetMode(MANUAL);
  myPID.SetOutputLimits(minPIDOutput,maxPIDOutput);
  myPID.SetSampleTime(PIDSampleTime);
}

void loop() {
  switch (state) {
    // state 0: Idle state, PID off until start
    case Idle:
      if (SerialInput == '8') {
        // Turn PID on (set to automatic)
        myPID.SetMode(AUTOMATIC);
        Serial.println("Received 8: PID set to automatic");
        Start = millis();
        End = 0;
        state = 1;
      }
      break;

    // state 1: processing photometry
    case Photometry:
      signal = analogRead(InputPin);
      if (millis() - Start >= PhotometryWindow) {
        PhotometrySum += signal;
        input = PhotometrySum / nSample;
        PhotometrySum = 0;
        Start = millis();
        state = 2;
        nSample = 1;

        // Calculate baseline
        BaselineSumInSecond += input;
        nBaselineSample += 1;
        if (nBaselineSample >= (1000/PhotometryWindow)){
          BaselineSumInWindow += BaselineSumInSecond / nBaselineSample;
          nBaselineSample = 0;
          BaselineSumInSecond = 0;
          nBaselineWindow += 1;
        }
      } else {
        PhotometrySum += signal;
        nSample += 1;
      }
      break;

    // state 2: calculate PID
    case Control:
      // Update target every 60 second
      if (target == 0){
        target = input;
      }
      if (nBaselineWindow == 59){
        target = BaselineSumInWindow / nBaselineWindow;
        BaselineSumInWindow = 0;
        nBaselineWindow = 0;
      }

      myPID.Compute();

      // Determine control value (final output)
      control = 255-output;

      analogWrite(OutputPin,control);
      // Serial.print("Input:");
      //Serial.println(input);
      // Serial.print("Target:");
      // Serial.println(target);

      //Serial.print("Error: ");
      Serial.println((target - input)*Kp);
      //Serial.print("dInput: ");
      //Serial.println((input - lastInput)*Kd);
      lastInput = input;
      // Serial.print("Output: ");
      // Serial.println(output/255);
      state = 1;
      break;
  }


  if (SerialInput == '9') {
    if (End == 0) {
      // Turn PID off (set to manual)
      myPID.SetMode(MANUAL);
      Serial.println("Received 9: PID set to manual");
      state = 0;
      digitalWrite(OutputPin, HIGH);
      End = millis();
    }
  }

  if (Serial.available() > 0) {
    // read the incoming byte
    SerialInput = Serial.read();
  }
}
