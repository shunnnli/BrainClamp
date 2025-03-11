#define Idle 0
#define Photometry 1
#define Control 2

#include <PID_v1.h>
#include <DataTome.h>
#include <DataTomeAnalysis.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_MCP4725.h>

// -----------------------
// Global Flags & Modes
// -----------------------
bool onlineTuningMode = false;   // false = offline mode; true = online tuning mode
bool startAutomatic = false;     // Set to true for AUTOMATIC startup, false for MANUAL (requires pressing '8')

// -----------------------
// Photometry & Baseline Params
// 
// -----------------------
// For moving average
constexpr double PhotometryWindow = 30; // in ms
constexpr double ArduinoFrequency = 8563; // Arduino sampling frequency
constexpr int windowSize = (int)(ArduinoFrequency * (PhotometryWindow / 1000.0));
int sampleBuffer[windowSize];
int index = 0;
unsigned long runningSum = 0;
int sampleCount = 0; 

// For binned average
double PhotometrySum = 0;
double rawSignal = 0;
double processedSignal = 0;

unsigned long BaselineSumInWindow = 0;
unsigned long nBaselineSample = 1;
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
DataTomeAnalysis<double, double> baselineWindow(20);

// -----------------------
// PID Variables
// -----------------------
double input;            // Filtered dopamine processedSignal (after moving average)
double target = 0;           // Desired dopamine level (might be normalized)
double output_inhibit;     // PID output for inhibition (controls inhibition laser)
double output_excite;    // PID output for excitation (controls excitation laser)
double control_inhibit;    // Final control value for inhibition laser
double control_excite;   // Final control value for excitation laser

// Default PID parameters for inhibition (reverse action) & excitation (direct)
double Kp_inhibit = 9, Ki_inhibit = 8.6, Kd_inhibit = 55;
double Kp_excite = 10, Ki_excite = 15, Kd_excite = 100;
double minPIDOutput = 0;
double maxPIDOutput = 255;
double PIDSampleTime = 1;    // in ms

// -----------------------
// PID Setup (using PID_v1 library)
// -----------------------
PID myPID_inhibit(&input, &output_inhibit, &target, Kp_inhibit, Ki_inhibit, Kd_inhibit, REVERSE);
PID myPID_excite(&input, &output_excite, &target, Kp_excite, Ki_excite, Kd_excite, DIRECT);

// -----------------------
// Pin Definitions
// -----------------------
const byte InputPin = A1;
const byte ControlPin_inhibit = 7;   // Inhibition laser pin
const byte ControlPin_excite = 12;   // Excitation laser pin
const byte TargetPin = 3;           // (Unused in this version)
const byte OutputPin_inhibit = A0;
const byte ClampOnPin = 8;

// Create an MCP4725 DAC object
Adafruit_MCP4725 dac;

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
  pinMode(ControlPin_inhibit, OUTPUT);
  pinMode(ControlPin_excite, OUTPUT);
  pinMode(TargetPin, INPUT);
  pinMode(OutputPin_inhibit, OUTPUT);
  pinMode(ClampOnPin, INPUT);

  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  digitalWrite(A2, LOW);//Set A2 as GND
  digitalWrite(A3, HIGH);//Set A3 as Vcc

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

  // Wait for the serial connection to be established
  while (!Serial) {;}
  // Serial.println("Serial is ready!");

  Serial.println("---------------------PhotometryClamp---------------------");
  Serial.println("Toggles: online tuning: 't'");
  Serial.println("Command: start clamping: 8  | Stop clamping: 9");
  // Initialize the MCP4725. If initialization fails, halt the program.
  if (!dac.begin(0x60)) {
    Serial.println("Failed to initialize MCP4725 DAC!");
  } else{
    Serial.println("MCP4725 DAC initialized.");
  }
  Serial.println("---------------------------------------------------------");
  Serial.println("Current PID values");
  Serial.print("Inhib: Kp=");
  Serial.print(Kp_inhibit); Serial.print(" Ki=");
  Serial.print(Ki_inhibit); Serial.print(" Kd=");
  Serial.println(Kd_inhibit);
  Serial.print("Excite: Kp=");
  Serial.print(Kp_excite); Serial.print(" Ki=");
  Serial.print(Ki_excite); Serial.print(" Kd=");
  Serial.println(Kd_excite);
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
        digitalWrite(ControlPin_inhibit, HIGH);
        digitalWrite(ControlPin_excite, HIGH);
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
      // Calculate moving average of photometry
      rawSignal = analogRead(InputPin);
      processedSignal = updateMovingAverage(rawSignal);
      state = Control;
      
      // Add baseline sample to baselineWindow buffer
      if (millis() - Start >= 3000) {
        BaselineAvgInWindow = BaselineSumInWindow / (double)nBaselineSample;
        baselineWindow.push(BaselineAvgInWindow);
        // Serial.println(BaselineAvgInWindow);
        nBaselineSample = 0;
        BaselineSumInWindow = 0;
        Start = millis();
        
      } else{
        // Accumulate data of baseline sample (3s)
        BaselineSumInWindow += processedSignal;
        nBaselineSample++;
      }
      break;

    // Control state: run the PIDs, update target, and output control signals
    case Control:
      // Determine input & output
      switch (normalizeMethod) {
        case RAW:
          if (baselineWindow.count() <= 1){
            input = processedSignal;
            target = processedSignal;
          } else{
            input = processedSignal;
            target = baselineWindow.median();
          }
          break;

        case BASELINE:
          // Normalize by baseline: typically, you want baseline to become 1.
          if (baselineWindow.count() <= 1){
            input = 1;
            target = 1;
          } else{
            baseline = baselineWindow.median();
            input = processedSignal / baseline;
            target = 1;
          }
          break;   

        case ZSCORE:
          // Calculate zscore for current input
          if (baselineWindow.count() <= 1){
            input = 0;
            target = 0;
          } else{
            baseline = baselineWindow.median();
            baseline_std = baselineWindow.std();
            zscore = (baseline_std > 0) ? ((processedSignal - baseline) / baseline_std) : 0;
            input = zscore;
            target = 0;
          }
          break;

        case STD:
          // Normalized by std
          if (baselineWindow.count() <= 1){
            input = processedSignal;
            target = processedSignal;
          } else{
            baseline = baselineWindow.median();
            baseline_std = baselineWindow.std();
            input = (baseline_std > 0) ? (processedSignal / baseline_std) : baseline;
            target = (baseline_std > 0) ? (baseline / baseline_std) : baseline;
          }
          break;
      }

      // Compute PID
      myPID_inhibit.Compute();
      myPID_excite.Compute();

      // Determine final control values:
      control_inhibit = 255 - output_inhibit;
      control_excite = 255 - output_excite;

      // Write outputs to respective pins
      analogWrite(ControlPin_inhibit, (int)control_inhibit);
      analogWrite(ControlPin_excite, (int)control_excite);

      // Print error signal (used for online tuning)
      double errorSignal = (target - input);
      double squaredError = errorSignal * errorSignal;
      Serial.println(digitalRead(ClampOnPin));
      // Serial.print("zscore: ");
      // Serial.println(zscore);
      // Serial.print("baseline std: ");
      // Serial.println(baselineWindow.std());
      // Serial.print("output: ");
      // Serial.println(output_inhibit);
      // Serial.println((input - lastInput)*Kd_inhibit / 255);

      // Send control output to DAC
      int scaled_OutputInhibit = map(output_inhibit, 0, 255, 0, 4095);
      dac.setVoltage(scaled_OutputInhibit, false);  // false: wait for the write to finish
      // analogWrite(OutputPin_inhibit, output_inhibit);

      state = Photometry; // Return to photometry for next sample
      break;
  }
}

// -----------------------
// Moving average methods
// Function to update the moving average with a new sample
// Returns the current moving average
// -----------------------
double updateMovingAverage(double newSample) {
  // Subtract the oldest sample from the running sum
  runningSum -= sampleBuffer[index];

  // Insert the new sample into the buffer and add it to the running sum
  sampleBuffer[index] = newSample;
  runningSum += newSample;

  // Move to the next index (wrap around using modulo)
  index = (index + 1) % windowSize;
  
  // Keep track of the number of samples (only until the buffer is full)
  if (sampleCount < windowSize) sampleCount++;
  
  // Return the average
  return runningSum / (double) sampleCount;
}
