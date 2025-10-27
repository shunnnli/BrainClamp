#define Idle 0
#define Photometry 1
#define Control 2

#include <PID_v1.h>
#include <math.h>
#include <Wire.h>

// -----------------------
// Global Flags & Modes
// -----------------------
bool onlineTuningMode = false;  // false = offline mode; true = online tuning mode
bool startAutomatic = false;    // Set to true for AUTOMATIC startup, false for MANUAL (requires pressing '8')
bool debugMode = false;
bool baselineResetMode = false;
unsigned long baselineResetStartTime = 0;
unsigned long lastClampStatusTime = 0;
unsigned long PIDStartTime = 0;

// enable fixed output mode for each channel:
bool fixInhib = false;
bool fixExcite = false;
double constantInhib = 0.0;  // constant output for inhibition channel
double constantExcite = 0.0; // constant output for excitation channel

// exponential output
float k_inhibit = 5; 
float k_excite = 2;
float expo_inhibit;
float expo_excite;
float expo;

// -----------------------
// Photometry & Baseline Params
// -----------------------
// For baseline sample duration
double baselineWindowDuration = 1000.0;  // in ms
double signal = 0;
unsigned long BaselineSumInWindow = 0;
unsigned long nBaselineSample = 1;
double BaselineAvgInWindow;
double baseline;
double baseline_std;
double zscore;
//const double ArduinoFrequency = 8563;    // Arduino sampling frequency

// -----------------------
// Define normalization methods
// -----------------------
enum NormalizeMethod {
  RAW,
  ZSCORE
};
NormalizeMethod normalizeMethod = RAW;

// -----------------------
// Fast incremental statistics (replaces DataTome)
// -----------------------
const int BASELINE_WINDOW_SIZE = 60;  // 60 windows of 3s each = 180s total
double baselineWindow[BASELINE_WINDOW_SIZE];
int baselineWindowIndex = 0;
int baselineWindowCount = 0;
bool baselineWindowFull = false;

// For fast moving average and variance
double baseline_sum = 0;
double baseline_sum_squares = 0;

// -----------------------
// PID Variables
// -----------------------
double input;            // Filtered dopamine processedSignal (after moving average)
double target = 0;       // Desired dopamine level (might be normalized)
bool targetLocked = false;  // True if target is manually set, false if following baseline
double output_inhibit;   // PID output for inhibition (controls inhibition laser)
double output_excite;    // PID output for excitation (controls excitation laser)
double control_inhibit;  // Final control value for inhibition laser
double control_excite;   // Final control value for excitation laser

// Default PID parameters for inhibition (reverse action) & excitation (direct)
double Kp_inhibit = 9, Ki_inhibit = 8.6, Kd_inhibit = 55;
double Kp_excite = 10, Ki_excite = 15, Kd_excite = 100;
double PIDSampleTime = 1;  // in ms
float Max_inhibit = 255;
float Max_excite = 255;

// -----------------------
// PID Setup (using PID_v1 library)
// -----------------------
// Hysteresis gating between excitation and inhibition
// -----------------------
enum ControlSide { SIDE_IDLE, SIDE_EXCITE, SIDE_INHIBIT };
ControlSide controlSide = SIDE_IDLE;

// Turn-on/turn-off thresholds (in units of error = input - target).
// For ZSCORE normalization, these are in z-score units.
// Positive error -> signal too high -> use inhibition
// Negative error -> signal too low -> use excitation
double EPS_ON  = 1.5;  // enter control when |error| exceeds this
double EPS_OFF = 0.8;  // leave control when |error| falls below this (must be < EPS_ON)
// -----------------------

// -----------------------
PID myPID_inhibit(&input, &output_inhibit, &target, Kp_inhibit, Ki_inhibit, Kd_inhibit, REVERSE);
PID myPID_excite(&input, &output_excite, &target, Kp_excite, Ki_excite, Kd_excite, DIRECT);

// -----------------------
// Pin Definitions
// -----------------------
const byte InputPin = A1;
const byte ControlPin_inhibit = 7;  // Inhibition laser pin
const byte ControlPin_excite = 2;   // Excitation laser pin
const byte TargetPin = 3;           // (Unused in this version)
const byte OutputPin_inhibit = A0;
const byte ClampOnPin = 8;  // Turn on clamp or not (external control for trial type specific clamping)

// -----------------------
// Timing & State Variables
// -----------------------
char SerialInput = '0';
static int state = Idle;
unsigned long Start = 0;
unsigned long End = 0;
unsigned long LastSampleTime = 0;
unsigned long nSample = 1;
unsigned long lastInput = 0;

// -----------------------
// Fast Baseline Statistics Functions
// -----------------------

// Push new baseline value into circular buffer and update statistics
void pushBaseline(double newValue) {
  // Remove oldest value if window is full
  if (baselineWindowFull) {
    double oldValue = baselineWindow[baselineWindowIndex];
    baseline_sum -= oldValue;
    baseline_sum_squares -= oldValue * oldValue;
  }
  
  // Add new value
  baselineWindow[baselineWindowIndex] = newValue;
  baseline_sum += newValue;
  baseline_sum_squares += newValue * newValue;
  
  // Update index
  baselineWindowIndex = (baselineWindowIndex + 1) % BASELINE_WINDOW_SIZE;
  
  // Track count
  if (!baselineWindowFull) {
    baselineWindowCount++;
    if (baselineWindowIndex == 0) {
      baselineWindowFull = true;
    }
  }
}

// Get current baseline mean (O(1) operation)
double getBaselineMean() {
  if (baselineWindowCount == 0) return 0;
  int currentSize = baselineWindowFull ? BASELINE_WINDOW_SIZE : baselineWindowCount;
  return baseline_sum / currentSize;
}

// Get current baseline std (O(1) operation)
double getBaselineStd() {
  if (baselineWindowCount == 0) return 1.0;
  int currentSize = baselineWindowFull ? BASELINE_WINDOW_SIZE : baselineWindowCount;
  double mean = baseline_sum / currentSize;
  double variance = (baseline_sum_squares / currentSize) - (mean * mean);
  return sqrt(max(variance, 1e-8));  // Avoid division by zero
}

// Get count of baseline samples
int getBaselineCount() {
  return baselineWindowCount;
}

// Reset baseline statistics (for 'R' command)
void resetBaseline() {
  baselineWindowIndex = 0;
  baselineWindowCount = 0;
  baselineWindowFull = false;
  baseline_sum = 0;
  baseline_sum_squares = 0;
}

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
  digitalWrite(A2, LOW);   //Set A2 as GND
  digitalWrite(A3, HIGH);  //Set A3 as Vcc

  // Initial sensor read & baseline setup
  signal = analogRead(InputPin);
  state = Idle;

  // Turn PIDs OFF initially (manual mode)
  if (startAutomatic) {
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

  myPID_inhibit.SetOutputLimits(0, Max_inhibit);
  myPID_excite.SetOutputLimits(0, Max_excite);
  myPID_inhibit.SetSampleTime(PIDSampleTime);
  myPID_excite.SetSampleTime(PIDSampleTime);

  // Wait for the serial connection to be established
  while (!Serial) { ; }
  // Serial.println("Serial is ready!");

  Serial.println("---------------------PhotometryClamp---------------------");
  Serial.println("Toggles: online tuning: 't'");
  Serial.println("Command: start clamping: 8  | Stop clamping: 9");
  Serial.println("         Reset baseline: R  (locks target, collects baseline for 60s)");
  Serial.println("---------------------------------------------------------");
}

// -----------------------
// Main Loop
// -----------------------
void loop() {
  // --- Mode Control via Serial Commands ---
  // 't' toggles online tuning mode
  if (Serial.available() > 0) {
    char inChar = Serial.read();
    if (inChar == 't') {
      onlineTuningMode = !onlineTuningMode;
      Serial.print("Online Tuning Mode: ");
      Serial.println(onlineTuningMode ? "ON" : "OFF");

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // 'R' will stop PID output and refill the baselineWindow values
    else if (inChar == 'R') {
      // First, set target to current baseline window average (lock it)
      if (normalizeMethod == ZSCORE) {
        baseline = getBaselineMean();
        baseline_std = getBaselineStd();
        target = (baseline_std > 0) ? ((BaselineAvgInWindow - baseline) / baseline_std) : 0;
      } else {
        target = BaselineAvgInWindow;
      }
      targetLocked = true;  // Lock target during baseline reset
      
      // Then start baseline reset mode
      baselineResetMode = true;
      baselineResetStartTime = millis();
      resetBaseline();  // Clear baseline statistics
      
      Serial.print("Baseline reset started. Target locked at: ");
      Serial.println(target, 2);

      // Flush any remaining characters so stray digits aren't misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }


    else if (inChar == 'D') {
      // Debug mode command: read next character (expected '1' or '0')
      while (!Serial.available()) {}  // wait for next byte
      char debugChar = Serial.read();
      if (debugChar == '1') {
        debugMode = true;
        Serial.println("Debug mode enabled.");
      } else if (debugChar == '0') {
        debugMode = false;
        Serial.println("Debug mode disabled.");
      }

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // Calibration command
    else if (inChar == 'C') {
      // Wait for the rest of the command
      while (!Serial.available()) { ; }
      String paramStr = Serial.readStringUntil('\n');
      paramStr.trim();
      int commaIndex = paramStr.indexOf(',');
      if (commaIndex != -1) {
        int pwmInhib = paramStr.substring(0, commaIndex).toInt();
        int pwmExcite = paramStr.substring(commaIndex + 1).toInt();
        analogWrite(ControlPin_inhibit, pwmInhib);
        analogWrite(ControlPin_excite, pwmExcite);
        Serial.print("CALIBRATION SET: ");
        Serial.print(pwmInhib);
        Serial.print(",");
        Serial.println(pwmExcite);
      }

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // Input processing options
    else if (inChar == 'P') {
      // Wait for the rest of the command
      while (!Serial.available()) {}
      String paramStr = Serial.readStringUntil('\n');
      paramStr.trim();

      // Expected format: <baselineWindowDuration>,<normalizationMethod>,<eps_on>,<eps_off>
      int idx1 = paramStr.indexOf(',');
      if (idx1 > 0) {
        int idx2 = paramStr.indexOf(',', idx1 + 1);
        if (idx2 > 0) {
          int idx3 = paramStr.indexOf(',', idx2 + 1);
          if (idx3 > 0) {
            // 1) baseline window duration (ms)
            double newBaselineWindowDuration = paramStr.substring(0, idx1).toFloat();
            // 2) normalization method (0=RAW, 1=ZSCORE)
            int normMethod = paramStr.substring(idx1 + 1, idx2).toInt();
            // 3) EPS_ON threshold
            double newEpsOn = paramStr.substring(idx2 + 1, idx3).toFloat();
            // 4) EPS_OFF threshold
            double newEpsOff = paramStr.substring(idx3 + 1).toFloat();

            // apply
            baselineWindowDuration = newBaselineWindowDuration;
            normalizeMethod        = (NormalizeMethod)normMethod;
            EPS_ON                 = newEpsOn;
            EPS_OFF                = newEpsOff;

            // echo back
            Serial.println("Photometry settings updated:");
            Serial.print("  Baseline sample duration (ms): "); Serial.println(baselineWindowDuration);
            Serial.print("  Normalization method: ");              Serial.println(normMethod);
            Serial.print("  EPS_ON: ");  Serial.print(EPS_ON);
            Serial.print("  EPS_OFF: "); Serial.println(EPS_OFF);

            // flush any stray chars
            while (Serial.available()) Serial.read();
          }
        }
      }
    }

    // If in online tuning mode, expect a tuning command starting with 'T'
    else if (inChar == 'T') {
      // Wait for the rest of the message
      while (!Serial.available()) {}
      String paramStr = Serial.readStringUntil('\n');
      paramStr.trim();

      const int NUM_PARAMS = 12;
      float params[NUM_PARAMS];
      int tokenIndex = 0;
      int startIndex = 0;
      while (tokenIndex < NUM_PARAMS) {
        int commaIndex = paramStr.indexOf(',', startIndex);
        if (commaIndex == -1) break;
        String token = paramStr.substring(startIndex, commaIndex);
        token.trim();
        params[tokenIndex] = token.toFloat();
        tokenIndex++;
        startIndex = commaIndex + 1;
      }
      
      if (tokenIndex == NUM_PARAMS) {
        Kp_inhibit = params[0];
        Ki_inhibit = params[1];
        Kd_inhibit = params[2];
        Kp_excite  = params[3];
        Ki_excite  = params[4];
        Kd_excite  = params[5];
        Max_inhibit  = params[6];
        Max_excite = params[7];
        int fixFlagInhib = (int)params[8];
        int fixFlagExcite = (int)params[9];
        k_inhibit = params[10];
        k_excite = params[11];
        
        if (fixFlagInhib == 1) {
          fixInhib = true;
          constantInhib = Max_inhibit; // Use the provided value as the fixed output
          myPID_inhibit.SetMode(MANUAL);
        } else {
          fixInhib = false;
        }
        
        if (fixFlagExcite == 1) {
          fixExcite = true;
          constantExcite = Max_excite;
          myPID_excite.SetMode(MANUAL);
        } else {
          fixExcite = false;
        }
        
        myPID_inhibit.SetTunings(Kp_inhibit, Ki_inhibit, Kd_inhibit);
        myPID_excite.SetTunings(Kp_excite, Ki_excite, Kd_excite);
        myPID_inhibit.SetOutputLimits(0, Max_inhibit);
        myPID_excite.SetOutputLimits(0, Max_excite);
        
        Serial.println("T command processed:");
        Serial.print("Inhib PID: Kp="); Serial.print(Kp_inhibit);
        Serial.print(" Ki="); Serial.print(Ki_inhibit);
        Serial.print(" Kd="); Serial.println(Kd_inhibit);
        Serial.print("Excite PID: Kp="); Serial.print(Kp_excite);
        Serial.print(" Ki="); Serial.print(Ki_excite);
        Serial.print(" Kd="); Serial.println(Kd_excite);
        Serial.print("Max Inhib: "); Serial.print(Max_inhibit);
        Serial.print(" Fix flag: "); Serial.println(fixFlagInhib);
        Serial.print("Max Excite: "); Serial.print(Max_excite);
        Serial.print(" Fix flag: "); Serial.println(fixFlagExcite);
        Serial.print("expo-k inhib: "); Serial.println(k_inhibit);
        Serial.print("expo-k excite: "); Serial.println(k_excite);
      } else {
        Serial.println("Error: T command expected 12 parameters.");
      }

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // '8' starts control (PID set to AUTOMATIC)
    else if (inChar == '8' && Start == 0) {
      myPID_inhibit.SetMode(AUTOMATIC);
      myPID_excite.SetMode(AUTOMATIC);
      Serial.print("CLAMP:");
      Serial.println("1");
      Serial.println("Received 8: PIDs set to AUTOMATIC");
      Start = millis();
      End = 0;
      LastSampleTime = 0;
      state = Photometry;

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // '9' stops control (PID set to MANUAL)
    else if (inChar == '9') {
      if (End == 0) {
        myPID_inhibit.SetMode(MANUAL);
        myPID_excite.SetMode(MANUAL);
        Serial.print("CLAMP:");
        Serial.println("0");
        Serial.println("Received 9: PIDs set to MANUAL");
        state = Idle;
        digitalWrite(ControlPin_inhibit, LOW);
        digitalWrite(ControlPin_excite, LOW);
        End = millis();
        Start = 0;
        LastSampleTime = 0;
      }

      // Flush any remaining characters so stray digits aren’t misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }
  }

  // -----------------------
  // State Machine
  // -----------------------
  switch (state) {
    // Idle state: waiting to start
    case Idle:
      // In Idle state, output clamp status every 100ms
      if (millis() - lastClampStatusTime >= 100) {
        Serial.println("CLAMP:0");
        lastClampStatusTime = millis();
      }
      break;

    // Photometry state: process sensor data & calculate moving average and baseline
    case Photometry:
      // Calculate moving average of photometry
      // if (debugMode) {
      //   Serial.print(" Duration: ");
      //   Serial.print(millis() - LastSampleTime, 3);
      //   LastSampleTime = millis();
      // }
      signal = analogRead(InputPin);
      state = Control;

      // Add baseline sample to baselineWindow buffer
      if (millis() - Start >= baselineWindowDuration) {
        BaselineAvgInWindow = BaselineSumInWindow / (double)nBaselineSample;
        pushBaseline(BaselineAvgInWindow);  // Use fast incremental push
        // Serial.println(BaselineAvgInWindow);
        nBaselineSample = 0;
        BaselineSumInWindow = 0;
        Start = millis();
      } else {
        // Accumulate data of baseline sample (3s)
        BaselineSumInWindow += signal;
        nBaselineSample++;
      }
      break;

    // Control state: run the PIDs, update target, and output control signals
    case Control:
      // Determine input & output
      // if (debugMode){
      //   Serial.print(" PID loop time: ");
      //   Serial.println(millis() - PIDStartTime);
      //   PIDStartTime = millis();
      // }
      switch (normalizeMethod) {
        case RAW:
          if (getBaselineCount() <= 1) {
            input = signal;
            if (!targetLocked) target = signal;
          } else {
            baseline = getBaselineMean();  // O(1) operation using mean
            if (!targetLocked) target = baseline;
            /* deadband no longer forces input to target; gating handles small errors */
            input = signal;
          }
          break;

        case ZSCORE:
          // Calculate zscore for current input (Fast O(1) operations!)
          if (getBaselineCount() <= 1) {
            input = 0;
            if (!targetLocked) target = 0;
          } else {
            if (!targetLocked) target = 0;
            baseline = getBaselineMean();      // O(1) operation
            baseline_std = getBaselineStd();   // O(1) operation
            /* deadband no longer forces input to target; gating handles small errors */
            input = (baseline_std > 0) ? ((signal - baseline) / baseline_std) : 0;
          }
          break;
      }

      // Compute PID (gated with hysteresis so only one side runs)
      // Signed error for gating:
      double e = (input - target);

      // If either channel is in fixed-output mode, skip gating and let
      // the fixed-output logic below take effect; set both PIDs to MANUAL.
      if (fixInhib || fixExcite) {
        myPID_inhibit.SetMode(MANUAL);  output_inhibit = 0;
        myPID_excite.SetMode(MANUAL);   output_excite  = 0;
      } else {
        // Update mode with hysteresis
        if (controlSide != SIDE_INHIBIT && e >  EPS_ON)  controlSide = SIDE_INHIBIT;
        if (controlSide != SIDE_EXCITE && e < -EPS_ON) controlSide = SIDE_EXCITE;
        if (controlSide == SIDE_INHIBIT  && e <  EPS_OFF)  controlSide = SIDE_IDLE;
        if (controlSide == SIDE_EXCITE && e > -EPS_OFF) controlSide = SIDE_IDLE;

        // Set PID modes based on which side is active
        if (controlSide == SIDE_EXCITE) {
          myPID_excite.SetMode(AUTOMATIC);
          myPID_inhibit.SetMode(MANUAL);
          output_inhibit = 0;
        } else if (controlSide == SIDE_INHIBIT) {
          myPID_inhibit.SetMode(AUTOMATIC);
          myPID_excite.SetMode(MANUAL);
          output_excite = 0;
        } else { // SIDE_IDLE
          myPID_inhibit.SetMode(MANUAL); output_inhibit = 0;
          myPID_excite.SetMode(MANUAL);  output_excite  = 0;
        }

        // Compute only the active side
        if (controlSide == SIDE_INHIBIT) { myPID_inhibit.Compute(); }
        if (controlSide == SIDE_EXCITE)  { myPID_excite.Compute();  }
      }


      // Determine final control values:
      bool ClampON = true;  //digitalRead(ClampOnPin);
      if (ClampON) {
        if (fixInhib){
          control_inhibit = constantInhib;
        }else{
          // 1) normalize
          double norm = constrain(output_inhibit/Max_inhibit, 0.0, 1.0);
          // 2) inverse exponential mapping (amplifies small inputs, compresses large)
          expo = log(1.0 + norm * (exp(k_inhibit) - 1.0)) / k_inhibit;
          // 3) rescale to PWM range
          control_inhibit = expo * Max_inhibit + 0.5;
        }
        if (fixExcite){
          control_excite = constantExcite;
        }else{
          // 1) normalize
          double norm = constrain(output_excite/Max_excite, 0.0, 1.0);
          // 2) inverse exponential mapping (amplifies small inputs, compresses large)
          expo = log(1.0 + norm * (exp(k_excite) - 1.0)) / k_excite;
          // 3) rescale to PWM range
          control_excite = expo * Max_excite + 0.5;
        }
      } else {
        control_inhibit = 0;
        control_excite = 0;
      }

      // If in baseline reset mode, check if 60s has passed to unlock target
      if (baselineResetMode) {
        if (millis() - baselineResetStartTime >= 60000) {
          baselineResetMode = false;  // Reset the mode after 60 seconds
          targetLocked = false;  // Unlock target to follow new baseline
          Serial.print("BASELINE_STATS:");
          Serial.print(getBaselineMean(),1);
          Serial.print(",");
          Serial.print(getBaselineStd(),1);
          Serial.println(" - Target now follows baseline");
        }
        // PID continues to run normally with locked target during collection
      }

      // Write outputs to respective pins
      analogWrite(ControlPin_inhibit, (int)control_inhibit);
      analogWrite(ControlPin_excite, (int)control_excite);

      // Serial output
      // If online tuning mode is active, output only the error.
      // Otherwise, output the state of ClampOnPin
      if (onlineTuningMode) {
        double squaredError = e * e;
        Serial.println(squaredError);
      } else if (debugMode) {
        // Print real-time values
        //Serial.print("  Target: "); Serial.print(target, 1);
        //Serial.print("  Signal: "); Serial.print(signal,1);
        //Serial.print("  Deadband: "); Serial.print(deadband,1);
        //Serial.print("  Input: "); Serial.println(input, 1);
        //Serial.print("  Control: "); Serial.println(output_inhibit, 1);
        //Serial.print("  Baseline Std: "); Serial.print(baseline_std, 1);
        //Serial.print("  Baseline: "); Serial.println(baseline, 1);
        Serial.print("  Error: "); Serial.print(e, 1);
        Serial.print("  Ctrl (inhi): "); Serial.print(control_inhibit, 1);
        Serial.print("  Ctrl (exci): "); Serial.println(control_excite, 1);
      } else {
        // Send clamp status to GUI
        Serial.print("CLAMP:");
        Serial.println(ClampON);
      }

      state = Photometry;  // Return to photometry for next sample
      break;
  }
}
