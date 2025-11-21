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
unsigned long lastClampStatusTime = 0;
unsigned long PIDStartTime = 0;

// enable fixed output mode for each channel:
bool fixInhib = false;
bool fixExcite = false;
double constantInhib = 0.0;  // constant output for inhibition channel
double constantExcite = 0.0; // constant output for excitation channel

// exponential output
float k_inhibit = 2; 
float k_excite = 1;
float expo_inhibit;
float expo_excite;
float expo;

// -----------------------
// Photometry & Baseline Params
// -----------------------
double signal = 0;

// Baseline estimator parameters (EWMA)
const double baselineTauS = 60.0;  // adapts over ~60 s; increase to slow adaptation
const double STD_FLOOR    = 0.1;   // minimum sigma to avoid division blow-ups (units of input_filt)

double baseline = 0.0;
double baseline_std = 1.0;
double zscore = 0.0;
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
// Baseline estimator state (EWMA mean & variance)
// -----------------------
// Implemented below in the "Fast Baseline Statistics Functions" section.
// -----------------------
// PID Variables
// -----------------------
double input;            // Filtered dopamine processedSignal (after moving average)
double input_filt = 0;    // filtered version of 'signal' used by PID
bool input_filt_initialized = false;
double target = 0;       // Desired dopamine level (might be normalized)
bool targetLocked = false;  // True if target is manually set, false if following baseline
double output_inhibit;   // PID output for inhibition (controls inhibition laser)
double output_excite;    // PID output for excitation (controls excitation laser)
double control_inhibit;  // Final control value for inhibition laser
double control_excite;   // Final control value for excitation laser

// Default PID parameters for inhibition (reverse action) & excitation (direct)
double Kp_inhibit = 20, Ki_inhibit = 0, Kd_inhibit = 1;
double Kp_excite = 20, Ki_excite = 0, Kd_excite = 0.5;
// --- Control scheduler & filter settings ---
const unsigned long PID_TS_MS = 5;   // 200 Hz control update
const double EMA_TAU_MS = 10.0;      // EMA time constant for input prefilter (tune 5–20 ms)
double PIDSampleTime = PID_TS_MS;  // in ms


float Max_inhibit = 255;
float Max_excite = 255;

// -----------------------
// PID Setup (using PID_v1 library)
// -----------------------
// Dwell (minimum hold) between side changes
const unsigned long MIN_HOLD_MS = 10;  // 10 ms dwell
unsigned long lastSideSwitchMs = 0;
// Control side state machine
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

// Loop sample-time measurement (microseconds)
static uint32_t fsLastUs = 0;
static uint16_t fsCount = 0;
static uint32_t fsSampleUs = 0;  // average sample time in microseconds

// -----------------------
// Fast Baseline Statistics Functions (EWMA-based)
// -----------------------
//
// We estimate baseline mean (mu) and variance (s2) using an exponentially
// weighted moving average (EWMA). This adapts slowly and uses O(1) time/memory.
//
//   mu_{t+1} = mu_t + beta * (x - mu_t)
//   s2_{t+1} = (1 - beta) * s2_t + beta * (x - mu_{t})^2
//
// where beta = dt / tau, dt in seconds (PID_TS_MS/1000), and tau = baselineTauS.
//
// Updates are gated so we don't contaminate baseline while actively controlling.

static double bl_mu = 0.0;
static double bl_s2 = 1.0;
static bool   bl_init = false;
static unsigned long bl_update_count = 0;

// Return current baseline mean (no floor)
double getBaselineMean() {
  return bl_mu;
}

// Return current baseline standard deviation with a safety floor
double getBaselineStd() {
  // Apply floor at the point of use
  double sigma2 = (bl_s2 > 1e-12) ? bl_s2 : 1e-12;
  double sigma = sqrt(sigma2);
  if (sigma < STD_FLOOR) sigma = STD_FLOOR;
  return sigma;
}

// Count of baseline updates
int getBaselineCount() {
  return (int)bl_update_count;
}

// Reset baseline estimator
void resetBaseline() {
  bl_mu = 0.0;
  bl_s2 = 1.0;
  bl_init = false;
  bl_update_count = 0;
}

// Optional: raw (unfloored) sigma for internal gating
static inline double getBaselineStdRaw() {
  double sigma2 = (bl_s2 > 1e-12) ? bl_s2 : 1e-12;
  return sqrt(sigma2);
}

// Update EWMA baseline with a new sample x, time-step dt_s (seconds).
// 'eligible' gates updates (e.g., only when SIDE_IDLE).
// 'gate_outliers' drops samples >3σ away from current mu.
void updateBaselineEWMA(double x, double dt_s, bool eligible, bool gate_outliers) {
  if (!eligible) return;

  // Initialize on first eligible value
  if (!bl_init) {
    bl_mu = x;
    bl_s2 = 0.0;
    bl_init = true;
    bl_update_count = 1;
    return;
  }

  double beta = dt_s / baselineTauS;
  if (beta < 0.0) beta = 0.0;
  if (beta > 1.0) beta = 1.0;

  if (gate_outliers) {
    double sigma = getBaselineStdRaw();
    if (fabs(x - bl_mu) > 3.0 * (sigma > 1.0 ? sigma : 1.0)) {
      return; // discard outlier
    }
  }

  double delta = x - bl_mu;
  bl_mu += beta * delta;
  // Use pre-update mu for variance increment to avoid double counting
  bl_s2 = (1.0 - beta) * bl_s2 + beta * (delta * delta);

  bl_update_count++;
}
// -----------------------
// Fast logging function
// -----------------------
// file scope
char line[64];
static uint32_t lastLog = 0;

void fastLog(double target, double input, double control_inhibit, double control_excite) {
  uint32_t now = millis();
  if (now - lastLog < 10) return;          // ~100 Hz; adjust as needed
  lastLog = now;

  // scale to avoid float prints
  uint16_t u_target  = (uint16_t)target;
  uint16_t u_input  = (uint16_t)input;
  uint16_t u_inhi  = (uint16_t)control_inhibit;
  uint16_t u_exci  = (uint16_t)control_excite;

  // compact, still human-readable
  int n = snprintf(line, sizeof(line),
                   "T:%u,I:%u,Ui:%u,Ue:%u\n", (unsigned)u_target, (unsigned)u_input, (unsigned)u_inhi, (unsigned)u_exci);

  if (n > 0) {
    // best-effort, non-blocking
    if ((int)Serial.availableForWrite() >= n) {
      Serial.write((uint8_t*)line, (size_t)n);
    }
    // else: drop this frame to protect control timing
  }
}


// -----------------------
// Setup
// -----------------------
void setup() {
  Serial.begin(500000);

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
  Serial.println("         Reset baseline: R  (instant reset with current value)");
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

    // 'R' will propagate current 1-second sample value to entire baseline window
    else if (inChar == 'R') {
      // Reset/seed the baseline estimator to the current filtered value
      // and re-lock the target accordingly.
      double currentValue;
      // Prefer the latest filtered sample if available; otherwise read the pin.
      if (input_filt_initialized) {
        currentValue = input_filt;
        Serial.print("Using current filtered value: ");
        Serial.println(currentValue, 2);
      } else {
        currentValue = analogRead(InputPin);
        Serial.print("Filtered value not initialized. Using current signal: ");
        Serial.println(currentValue, 2);
      }

      // Reset EWMA baseline to current value
      resetBaseline();
      bl_mu = currentValue;   // seed mean directly
      bl_s2 = 0.0;            // zero variance; will grow as samples arrive
      bl_init = true;
      bl_update_count = 1;

      // Set target to current value (lock it)
      if (normalizeMethod == ZSCORE) {
        baseline = getBaselineMean();
        baseline_std = getBaselineStd();
        target = (baseline_std > 0) ? ((currentValue - baseline) / baseline_std) : 0;
      } else {
        target = currentValue;
      }

      // Reset PID controllers to clear integral windup
      myPID_inhibit.SetMode(MANUAL);
      myPID_excite.SetMode(MANUAL);
      // This clears the integral term and resets internal state
      myPID_inhibit.SetMode(AUTOMATIC);
      myPID_excite.SetMode(AUTOMATIC);

      Serial.print("Baseline reset completed. Target set to: ");
      Serial.println(target, 2);

      // Flush any remaining characters so stray digits aren't misinterpreted.
      while (Serial.available()) {
        Serial.read();
      }
    }

    // Debug mode command
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

      // Expected format: <baselineTauS>,<normalizationMethod>,<eps_on>,<eps_off>
      int idx1 = paramStr.indexOf(',');
      if (idx1 > 0) {
        int idx2 = paramStr.indexOf(',', idx1 + 1);
        if (idx2 > 0) {
          int idx3 = paramStr.indexOf(',', idx2 + 1);
          if (idx3 > 0) {
            // 1) baseline tau in seconds
            double newBaselineTauS = paramStr.substring(0, idx1).toFloat();
            // 2) normalization method (0=RAW, 1=ZSCORE)
            int normMethod = paramStr.substring(idx1 + 1, idx2).toInt();
            // 3) EPS_ON threshold
            double newEpsOn = paramStr.substring(idx2 + 1, idx3).toFloat();
            // 4) EPS_OFF threshold
            double newEpsOff = paramStr.substring(idx3 + 1).toFloat();

            // apply
            if (newBaselineTauS > 0.01) baselineTauS = newBaselineTauS;
            normalizeMethod        = (NormalizeMethod)normMethod;
            EPS_ON                 = newEpsOn;
            EPS_OFF                = newEpsOff;

            // echo back
            Serial.println("Photometry settings updated:");
            Serial.print("  Baseline tau (s): "); Serial.println(baselineTauS, 2);
            Serial.print("  Normalization method: ");              Serial.println(normMethod);
            Serial.print("  EPS_ON: ");  Serial.println(EPS_ON, 3);
            Serial.print("  EPS_OFF: "); Serial.println(EPS_OFF, 3);

            // flush any stray chars
            while (Serial.available()) Serial.read();
          }
        }
      }
    }

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
      // EMA prefilter on the raw signal
      if (!input_filt_initialized) { input_filt = signal; input_filt_initialized = true; }
      else {
        const double alpha = (EMA_TAU_MS > 0.0) ? (1.0 - exp(-(double)PIDSampleTime/EMA_TAU_MS)) : 1.0;
        input_filt += alpha * (signal - input_filt);
      }
      state = Control;

      // Update EWMA baseline once per sample (from filtered input)
      const double dt_s = (double)PID_TS_MS / 1000.0;
      const bool baselineEligible = (controlSide == SIDE_IDLE);
      // Conservative outlier gating to prevent contamination
      const bool gateOutliers = true;
      updateBaselineEWMA(input_filt, dt_s, baselineEligible, gateOutliers);
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
            input = input_filt;
            if (!targetLocked) target = input_filt;
          } else {
            baseline = getBaselineMean();  // O(1) operation using mean
            if (!targetLocked) target = baseline;
            /* deadband no longer forces input to target; gating handles small errors */
            input = input_filt;
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
            input = (baseline_std > 0) ? ((input_filt - baseline) / baseline_std) : 0;
          }
          break;
      }

      // Compute PID (gated with hysteresis so only one side runs)
      // Signed error for gating:
      double e = (input - target);

      // If either channel is in fixed-output mode, skip gating and let
      // the fixed-output logic below take effect; set both PIDs to MANUAL.
      if (fixInhib || fixExcite) {
        myPID_inhibit.SetMode(MANUAL);
        myPID_excite.SetMode(MANUAL);
      } else {
        // Update mode with hysteresis; no dwell in IDLE, dwell only side-to-side
        ControlSide wantSide = controlSide;
        if (controlSide != SIDE_INHIBIT && e >  EPS_ON)  wantSide = SIDE_INHIBIT;
        if (controlSide != SIDE_EXCITE  && e < -EPS_ON)  wantSide = SIDE_EXCITE;
        if (controlSide == SIDE_INHIBIT && e <  EPS_OFF) wantSide = SIDE_IDLE;
        if (controlSide == SIDE_EXCITE  && e > -EPS_OFF) wantSide = SIDE_IDLE;

        bool canSwitch   = (millis() - lastSideSwitchMs) >= MIN_HOLD_MS;
        bool leavingIdle = (controlSide == SIDE_IDLE && wantSide != SIDE_IDLE);
        bool goingToIdle = (controlSide != SIDE_IDLE && wantSide == SIDE_IDLE);
        bool sideToSide  = (controlSide != SIDE_IDLE && wantSide != SIDE_IDLE && wantSide != controlSide);

        if (leavingIdle || goingToIdle || (sideToSide && canSwitch)) {
          controlSide = wantSide;
          lastSideSwitchMs = millis();
        }

        // Set PID modes based on which side is active
        if (controlSide == SIDE_EXCITE) {
          myPID_excite.SetMode(AUTOMATIC);
          myPID_inhibit.SetMode(MANUAL);
        } else if (controlSide == SIDE_INHIBIT) {
          myPID_inhibit.SetMode(AUTOMATIC);
          myPID_excite.SetMode(MANUAL);
        } else { // SIDE_IDLE
          myPID_inhibit.SetMode(MANUAL);
          myPID_excite.SetMode(MANUAL);
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
        } else if (controlSide == SIDE_INHIBIT){
          // 1) normalize
          double norm = constrain(output_inhibit/Max_inhibit, 0.0, 1.0);
          // 2) inverse exponential mapping (amplifies small inputs, compresses large)
          expo = log(1.0 + norm * (exp(k_inhibit) - 1.0)) / k_inhibit;
          // 3) rescale to PWM range
          control_inhibit = expo * Max_inhibit + 0.5;
        } else {
          control_inhibit = 0;
        }

        if (fixExcite){
          control_excite = constantExcite;
        } else if (controlSide == SIDE_EXCITE){
          // 1) normalize
          double norm = constrain(output_excite/Max_excite, 0.0, 1.0);
          // 2) inverse exponential mapping (amplifies small inputs, compresses large)
          expo = log(1.0 + norm * (exp(k_excite) - 1.0)) / k_excite;
          // 3) rescale to PWM range
          control_excite = expo * Max_excite + 0.5;
        } else {
          control_excite = 0;
        }
      } else {
        control_inhibit = 0;
        control_excite = 0;
      }


      // Write outputs to respective pins
      analogWrite(ControlPin_inhibit, (int)control_inhibit);
      analogWrite(ControlPin_excite, (int)control_excite);

      // --- Measure PID sample time (microseconds only) ---
      fsCount++;
      uint32_t nowUs = micros();
      if (nowUs - fsLastUs >= 500000UL) { // compute about twice per second
        uint32_t elapsedUs = nowUs - fsLastUs;
        fsSampleUs = (fsCount > 0) ? (elapsedUs / fsCount) : 0;  // average period (µs)
        fsCount = 0;
        fsLastUs = nowUs;
        if (debugMode) {
          Serial.print("DEBUG:STus:");
          Serial.println(fsSampleUs);
        }
      }

      // Serial output
      // If online tuning mode is active, output only the error.
      // Otherwise, output the state of ClampOnPin
      if (onlineTuningMode) {
        double squaredError = e * e;
        Serial.println(squaredError);
      } else if (debugMode) {
        fastLog(target, input, control_inhibit, control_excite);
      } else {
        // Send clamp status to GUI
        Serial.print("CLAMP:");
        Serial.println(ClampON);
      }

      state = Photometry;  // Return to photometry for next sample
      break;
  }
}
