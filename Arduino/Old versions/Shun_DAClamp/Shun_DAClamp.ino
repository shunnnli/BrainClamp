#include <PID_v1.h>

// PID params
double input, output, target;
int SampleTime = 10; // evaluate PID every 50ms
int Kp = 3;
int Ki = 2;
int Kd = 1;
double minPIDOutput = 0;
double maxPIDOutput = 100;


// Set up pins in arduino
const byte InputPin = A0;
const byte OutputPin = 2;
const byte TargetPin = 3;

char SerialInput = '0';

// Setup PID
PID myPID(&input, &output, &target, Kp,Ki,Kd, REVERSE);

void setup() {
  Serial.begin(115200);
  pinMode(InputPin, INPUT);
  pinMode(OutputPin, OUTPUT);
  pinMode(TargetPin, INPUT);

  input = analogRead(InputPin);
  target = analogRead(TargetPin);

  // turn PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(minPIDOutput,maxPIDOutput);
  myPID.SetSampleTime(SampleTime);
}

void loop() {
  if (SerialInput == '8'){
    // Turn PID on (set to automatic)
    myPID.SetMode(AUTOMATIC);
    Serial.println('Received 8: PID set to automatic');
  }

  if (SerialInput == '9'){
    // Turn PID off (set to manual)
    myPID.SetMode(MANUAL);
    Serial.println('Received 9: PID set to manual');
  }

  input = analogRead(InputPin);
  target = 0;//analogRead(TargetPin);

  myPID.Compute();
  // analogWrite(OutputPin,output);
  Serial.print("Input:");
  Serial.println(input);
  Serial.print("Target:");
  Serial.println(target);
  Serial.print("Output:");
  Serial.println(output);
  // Serial.println("----------");
}
