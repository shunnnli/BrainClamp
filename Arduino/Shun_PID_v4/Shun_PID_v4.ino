#define Idle 0
#define Photometry 1
#define Control 2

#include <PID_v1.h>
#include <DataTome.h>
#include <DataTomeAnalysis.h>
#include <math.h>

// -----------------------
// Global Flags & Modes
// -----------------------
bool onlineTuningMode = true;   // false = offline mode; true = online tuning mode
bool startAutomatic = true;     // Set to true for AUTOMATIC startup, false for MANUAL (requires pressing '8')

// -----------------------
// Photometry & Baseline Params
// -----------------------
double PhotometryWindow = 30; // in ms
double PIDSampleTime = 1;    // in ms
double PhotometrySum = 0;
double rawSignal = 0;
double signal = 0;

unsigned long BaselineSumInWindow = 0;
unsigned long nBaselineSample = 0;
double BaselineAvgInWindow;
double baseline;
double baseline_std;
double zscore;

// -----------------------
// Define normalization methods
// -----------------------
enum NormalizeMethod {
  RAW,
  ZSCORE,
  BASELINE,
  STD
};
NormalizeMethod normalizeMethod = ZSCORE;

// -----------------------
// Initialize arrays for moving statistics
// (Assume 3s baseline sample)
// (20 -> 60s)
// (60 -> 180s)
// -----------------------
DataTomeAnalysis<int, unsigned long> baselineWindow(20);

// -----------------------
// PID Variables
// -----------------------
double input;            // Filtered dopamine signal (after moving average)
double target = 0;           // Desired dopamine level (might be normalized)
double output_inhibit;     // PID output for inhibition (controls inhibition laser)
double output_excite;    // PID output for excitation (controls excitation laser)
double control_inhibit;    // Final control value for inhibition laser
double control_excite;   // Final control value for excitation laser

// Default PID parameters for inhibition (reverse action) & excitation (direct)
double Kp_inhibit = 0, Ki_inhibit = 0, Kd_inhibit = 5;
double Kp_excite = 10, Ki_excite = 15, Kd_excite = 100;
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
const byte OutputPin_inhibit = 7;   // Inhibition laser pin
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
// Setup
// -----------------------
void setup() {
  Serial.begin(115200);
  pinMode(InputPin, INPUT);
  pinMode(OutputPin_inhibit, OUTPUT);
  pinMode(OutputPin_excite, OUTPUT);
  pinMode(TargetPin, INPUT);

  // Initial sensor read & baseline setup
  rawSignal = analogRead(InputPin);
  state = Idle;

  // Turn PIDs OFF initially (manual mode)
  if (startAutomatic){
    // Start PID immediately if startAutomatic == true
    myPID_inhibit.SetMode(AUTOMATIC);
    myPID_excite.SetMode(AUTOMATIC);
    Start = millis();
    End = 0;
    state = Photometry;
  } else {
    myPID_inhibit.SetMode(MANUAL);
    myPID_excite.SetMode(MANUAL);
  }

  myPID_inhibit.SetOutputLimits(minPIDOutput, maxPIDOutput);
  myPID_excite.SetOutputLimits(minPIDOutput, maxPIDOutput);
  myPID_inhibit.SetSampleTime(PIDSampleTime);
  myPID_excite.SetSampleTime(PIDSampleTime);

  Serial.println("---------------------PhotometryClamp---------------------");
  Serial.println("Toggles: online tuning: 't'");
  Serial.println("Command: start clamping: 8  | Stop clamping: 9");
  Serial.println("---------------------------------------------------------");
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
    else if (inChar == '8' && Start == 0) {
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
        Start = 0;
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
      rawSignal = analogRead(InputPin);
      // Serial.println(millis() - Start);
      if (millis() - Start >= PhotometryWindow) {
        PhotometrySum += rawSignal;

        // Compute moving average from nSample readings
        signal = PhotometrySum / nSample;
        PhotometrySum = 0;
        Start = millis();
        state = Control;
        nSample = 1;

        // Accumulate data of baseline sample (3s)
        BaselineSumInWindow += signal;
        nBaselineSample++;
        
        // Add each 100ms baseline sample to baselineWindow buffer
        if (nBaselineSample >= (3000 / PhotometryWindow)) {
          BaselineAvgInWindow = BaselineSumInWindow / nBaselineSample;
          baselineWindow.push(BaselineAvgInWindow);
          nBaselineSample = 0;
          BaselineSumInWindow = 0;
        }
      } else {
        PhotometrySum += rawSignal;
        nSample++;
      }
      break;

    // Control state: run the PIDs, update target, and output control signals
    case Control:
      // Get baseline
      if (baselineWindow.count() == 0){
        baseline = signal;
      } else{
        baseline = baselineWindow.median();
      }
      
      // Determine input & output
      switch (normalizeMethod) {
        case RAW:
          input = signal;
          target = baseline;
          break;

        case BASELINE:
          // Normalize by baseline: typically, you want baseline to become 1.
          input = signal / baseline;
          target = 1;
          break;   

        case ZSCORE:
          // Calculate zscore for current input
          baseline_std = baselineWindow.std();
          zscore = (baseline_std > 0) ? ((signal - baseline) / baseline_std) : 0;
          input = zscore;
          target = 0;
          break;

        case STD:
          // Normalized by std
          baseline_std = baselineWindow.std();
          input = (baseline_std > 0) ? (signal / baseline_std) : baseline;
          target = (baseline_std > 0) ? (baseline / baseline_std) : baseline;
          break;
      }

      // Compute PID
      myPID_inhibit.Compute();
      myPID_excite.Compute();

      // Determine final control values:
      control_inhibit = 255 - output_inhibit;
      control_excite = 255 - output_excite;

      // Write outputs to respective pins
      analogWrite(OutputPin_inhibit, (int)control_inhibit);
      analogWrite(OutputPin_excite, (int)control_excite);

      // Print error signal (used for online tuning)
      double errorSignal = (target - input);
      double squaredError = errorSignal * errorSignal;
      Serial.println(squaredError);
      // Serial.print("zscore: ");
      // Serial.println(zscore);
      // Serial.print("baseline std: ");
      // Serial.println(baseline_std);
      // Serial.print("output: ");
      // Serial.println(output_inhibit);
      //Serial.println((input - lastInput)*Kd_inhibit / 255);
      
      state = Photometry; // Return to photometry for next sample
      break;
  }
}
