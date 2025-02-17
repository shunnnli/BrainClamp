#define Idle 0
#define Photometry 1
#define Control 2

#include <PID_v1.h>

// -----------------------
// Global Flags & Modes
// -----------------------
bool onlineTuningMode = false;  // false = offline mode; true = online tuning mode
bool useKalman = false;         // false = no Kalman filter; true = use Kalman filter

// -----------------------
// Photometry & Baseline Params
// -----------------------
double PhotometryWindow = 5; // in ms
double PIDSampleTime = 0;    // in ms
double PhotometrySum = 0;
double signal = 0;

unsigned long BaselineSumInSecond = 0;
unsigned long BaselineSumInWindow = 0;
int nBaselineSample = 0;
int nBaselineWindow = 0;

// -----------------------
// PID Variables
// -----------------------
double input;            // Filtered dopamine signal (after moving average / Kalman)
double target;           // Desired dopamine level (baseline)
double output_inhibit;     // PID output for inhibition (controls inhibition laser)
double output_excite;    // PID output for excitation (controls excitation laser)
double control_inhibit;    // Final control value for inhibition laser
double control_excite;   // Final control value for excitation laser

// Default PID parameters for inhibition (reverse action) & excitation (direct)
double Kp_inhibit = 10, Ki_inhibit = 10, Kd_inhibit = 100;
double Kp_excite = 10, Ki_excite = 10, Kd_excite = 100;
double minPIDOutput = 0;
double maxPIDOutput = 255;

// -----------------------
// PID Setup (using PID_v1 library)
// -----------------------
PID myPID_inhibit(&input, &output_inhibit, &target, Kp_inhibit, Ki_inhibit, Kd_inhibit, REVERSE);
PID myPID_excite(&input, &output_excite, &target, Kp_excite, Ki_excite, Kd_excite, DIRECT);

// -----------------------
// Pin Definitions
// -----------------------
const byte InputPin = A0;
const byte OutputPin_inhibit = 13;   // Inhibition laser pin
const byte OutputPin_excite = 12;   // Excitation laser pin
const byte TargetPin = 3;          // (Unused in this version)

// -----------------------
// Timing & State Variables
// -----------------------
char SerialInput = '0';
static int state = Idle;
unsigned long Start = 0;
unsigned long End = 0;
unsigned long nSample = 1;
unsigned long lastInput = 0;

// -----------------------
// Kalman Filter Variables & Function
// -----------------------
float Q = 0.022;     // Process noise covariance
float R = 0.617;     // Measurement noise covariance
float P = 1.0;       // Estimation error covariance
float K = 0.0;       // Kalman gain
float X_est = 0.0;   // Filtered value estimate

float kalmanFilter(float measurement) {
  // Predict update
  P = P + Q;
  // Kalman gain
  K = P / (P + R);
  // Update estimate with measurement
  X_est = X_est + K * (measurement - X_est);
  // Update error covariance
  P = (1 - K) * P;
  return X_est;
}

// -----------------------
// Setup
// -----------------------
void setup() {
  Serial.begin(115200);
  pinMode(InputPin, INPUT);
  pinMode(OutputPin_inhibit, OUTPUT);
  pinMode(OutputPin_excite, OUTPUT);
  pinMode(TargetPin, INPUT);

  // Initial sensor read & baseline setup
  signal = analogRead(InputPin);
  // Option: initialize X_est for Kalman with first reading
  X_est = signal;
  target = 0; // Will be set to baseline later
  state = Idle;

  // Turn PIDs OFF initially (manual mode)
  myPID_inhibit.SetMode(MANUAL);
  myPID_excite.SetMode(MANUAL);
  myPID_inhibit.SetOutputLimits(minPIDOutput, maxPIDOutput);
  myPID_excite.SetOutputLimits(minPIDOutput, maxPIDOutput);
  myPID_inhibit.SetSampleTime(PIDSampleTime);
  myPID_excite.SetSampleTime(PIDSampleTime);

  Serial.println("---------------------PhotometryClamp---------------------");
  Serial.println("Toggles: online tuning: 't' | Kalman filter: 'k'");
  Serial.println("Command: start clamping: 8  | Stop clamping: 9");
  Serial.println("---------------------PhotometryClamp---------------------");
}

// -----------------------
// Main Loop
// -----------------------
void loop() {
  // --- Mode Control via Serial Commands ---
  // 't' toggles online tuning mode, 'k' toggles Kalman filter use.

  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == 't') {
      onlineTuningMode = !onlineTuningMode;
      Serial.print("Online Tuning Mode: ");
      Serial.println(onlineTuningMode ? "ON" : "OFF");
    }
    else if (inChar == 'k') {
      useKalman = !useKalman;
      Serial.print("Kalman Filter: ");
      Serial.println(useKalman ? "ON" : "OFF");
    }
    // If in online tuning mode, expect a tuning command starting with 'T'
    else if (onlineTuningMode && inChar == 'T') {
      // Expect parameters as comma-separated values:
      // Format: T,<Kp_inhibit>,<Ki_inhibit>,<Kd_inhibit>,<Kp_excite>,<Ki_excite>,<Kd_excite>
      while (!Serial.available()) {} // wait for rest of message
      String paramStr = Serial.readStringUntil('\n');
      // Concatenate with initial 'T' removed.
      paramStr.trim();
      // Parse parameters
      int idx1 = paramStr.indexOf(',');
      int idx2 = paramStr.indexOf(',', idx1+1);
      int idx3 = paramStr.indexOf(',', idx2+1);
      int idx4 = paramStr.indexOf(',', idx3+1);
      int idx5 = paramStr.indexOf(',', idx4+1);
      if (idx1 > 0 && idx2 > idx1 && idx3 > idx2 && idx4 > idx3 && idx5 > idx4) {
        Kp_inhibit   = paramStr.substring(0, idx1).toFloat();
        Ki_inhibit   = paramStr.substring(idx1+1, idx2).toFloat();
        Kd_inhibit   = paramStr.substring(idx2+1, idx3).toFloat();
        Kp_excite  = paramStr.substring(idx3+1, idx4).toFloat();
        Ki_excite  = paramStr.substring(idx4+1, idx5).toFloat();
        Kd_excite  = paramStr.substring(idx5+1).toFloat();

        // Update PID tunings
        myPID_inhibit.SetTunings(Kp_inhibit, Ki_inhibit, Kd_inhibit);
        myPID_excite.SetTunings(Kp_excite, Ki_excite, Kd_excite);

        Serial.println("Online tuning parameters updated:");
        Serial.print("Inhib: Kp=");
        Serial.print(Kp_inhibit); Serial.print(" Ki=");
        Serial.print(Ki_inhibit); Serial.print(" Kd=");
        Serial.println(Kd_inhibit);
        Serial.print("Excite: Kp=");
        Serial.print(Kp_excite); Serial.print(" Ki=");
        Serial.print(Ki_excite); Serial.print(" Kd=");
        Serial.println(Kd_excite);
      }
    }
    // '8' starts control (PID set to AUTOMATIC)
    else if (inChar == '8') {
      myPID_inhibit.SetMode(AUTOMATIC);
      myPID_excite.SetMode(AUTOMATIC);
      Serial.println("Received 8: PIDs set to AUTOMATIC");
      Start = millis();
      End = 0;
      state = Photometry;
    }
    // '9' stops control (PID set to MANUAL)
    else if (inChar == '9') {
      if (End == 0) {
        myPID_inhibit.SetMode(MANUAL);
        myPID_excite.SetMode(MANUAL);
        Serial.println("Received 9: PIDs set to MANUAL");
        state = Idle;
        digitalWrite(OutputPin_inhibit, HIGH);
        digitalWrite(OutputPin_excite, HIGH);
        End = millis();
      }
    }
  }
  
  // -----------------------
  // State Machine
  // -----------------------
  switch (state) {
    // Idle state: waiting to start
    case Idle:
      // Nothing to do until command received
      break;

    // Photometry state: process sensor data & calculate moving average and baseline
    case Photometry:
      signal = analogRead(InputPin);
      // Optionally run signal through Kalman filter
      if (useKalman) {
        signal = kalmanFilter(signal);
      }
      
      if (millis() - Start >= PhotometryWindow) {
        PhotometrySum += signal;
        // Compute moving average from nSample readings
        input = PhotometrySum / nSample;
        PhotometrySum = 0;
        Start = millis();
        state = Control;
        nSample = 1;

        // Calculate baseline (moving average over 1 second)
        BaselineSumInSecond += input;
        nBaselineSample++;
        if (nBaselineSample >= (1000 / PhotometryWindow)) {
          BaselineSumInWindow += (BaselineSumInSecond / nBaselineSample);
          nBaselineSample = 0;
          BaselineSumInSecond = 0;
          nBaselineWindow++;
        }
      } else {
        PhotometrySum += signal;
        nSample++;
      }
      break;

    // Control state: run the PIDs, update target, and output control signals
    case Control:
      // Update target (baseline) every 60 seconds
      if (target == 0) {
        target = input; // First time: set baseline to current input
      }
      if (nBaselineWindow >= 59) {
        target = BaselineSumInWindow / nBaselineWindow;
        BaselineSumInWindow = 0;
        nBaselineWindow = 0;
      }
      
      // Compute both PIDs
      myPID_inhibit.Compute();
      myPID_excite.Compute();

      // Determine final control values:
      control_inhibit = 255 - output_inhibit;
      control_excite = 255 - output_excite;

      // Write outputs to respective pins
      analogWrite(OutputPin_inhibit, (int)control_inhibit);
      analogWrite(OutputPin_excite, (int)control_excite);
      // Serial.print("Input:");
      //Serial.println(input);
      // Serial.print("Target:");
      // Serial.println(target);

      // For debugging, print the error from inhibition PID
      //Serial.print("Error: ");
      Serial.println((target - input) * Kp_inhibit);
      //Serial.print("dInput: ");
      //Serial.println((input - lastInput)*Kd);
      // lastInput = input;
      // Serial.print("Output: ");
      // Serial.println(output/255);
      
      state = Photometry; // Return to photometry for next sample
      break;
  }
}
