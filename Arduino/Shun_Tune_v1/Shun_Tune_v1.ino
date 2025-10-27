/* 
  Shun_tuning_v1.ino
  -----------------------------------------------------------
  Open-loop tuning helper for two-channel (excite/inhibit) system.
  Uses the SAME wiring as Shun_PID_v15.ino (pins, input power pins).
  
  What it provides (triggered via Serial commands):
    1) '1' : 5 min baseline (no stimulation), continuous RAW logging
    2) '2' : 3 s step sweeps for EXCITATION (multiple power levels), ITI=5 s, 5 repeats each
    3) '3' : 3 s step sweeps for INHIBITION (multiple power levels), ITI=5 s, 5 repeats each
    4) '4' : 10 s fixed EXCITATION with 3 s INHIBITION in the middle (overlay)
    5) '9' : 10 s fixed INHIBITION with 3 s EXCITATION in the middle (overlay)
    6) '5' : Manual EXCITATION: 2 s at max (for reward-aligned test)
    7) '6' : Manual INHIBITION: 2 s at max (for punishment-aligned test)
    x) 'x' : Abort current protocol / set outputs to zero
    h) 'h' or '?' : help
  
  Logging:
    CSV lines to Serial at 500000 baud with columns:
      t_ms, raw, excite_pwm, inhibit_pwm, label
    RAW is analogRead(A1) (same as in your PID sketch).

  Board assumptions:
    - Uses Arduino MEGA-style PWM: pin 2 and pin 7 support PWM (matches Shun_PID_v15.ino usage).
    - A2 and A3 are driven LOW/HIGH to supply GND/Vcc to the photometry sensor (replicated here).
  
  Safe defaults:
    - PWM range: 0..255
    - Power levels (fraction of max): {0.05, 0.10, 0.20, 0.30, 0.50, 0.80, 1.00}
    - ITI: 5 s; steps: 3 s; overlay: background 10 s with 3 s opposite channel centered.
  
  You can safely tweak the constants in the "User config" section below.
*/

// -----------------------
// Wiring (copied from Shun_PID_v15.ino)
// -----------------------
const byte InputPin           = A1; // photometry RAW input
const byte ControlPin_inhibit = 7;  // inhibition laser PWM
const byte ControlPin_excite  = 2;  // excitation laser PWM
const byte TargetPin          = 3;  // (unused)
const byte ClampOnPin         = 8;  // (unused here)

// Provide power to the sensor from pins (as in your PID file)
const byte FakeGND            = A2;
const byte FakeVCC            = A3;

// -----------------------
// User config
// -----------------------
const unsigned long LOG_INTERVAL_MS = 10;     // logging/sample rate (~100 Hz)
const unsigned long BASELINE_MS     = 5UL * 60UL * 1000UL; // 5 minutes

// Step-sweep parameters
const unsigned long STEP_DURATION_MS = 3000;  // 3 s step
const unsigned long ITI_MS           = 5000;  // 5 s ITI between stimuli
const int REPEATS_PER_LEVEL          = 5;

// Overlay parameters
const unsigned long OVERLAY_TOTAL_MS   = 10000; // 10 s background
const unsigned long OVERLAY_MIDDLE_MS  = 3000;  // 3 s overlay
// Start overlay so it's centered within 10 s
const unsigned long OVERLAY_START_MS   = (OVERLAY_TOTAL_MS - OVERLAY_MIDDLE_MS) / 2; // 3500 ms
const unsigned long OVERLAY_END_MS     = OVERLAY_START_MS + OVERLAY_MIDDLE_MS;       // 6500 ms

// PWM scaling (caps) — change if you want to limit "max"
const int MAX_PWM_EXCITE = 255;
const int MAX_PWM_INHIB  = 255;

// Power levels as fraction of "max"
const float POWER_LEVELS[] = {0.05, 0.10, 0.20, 0.30, 0.50, 0.80, 1.00};
const int   N_LEVELS      = sizeof(POWER_LEVELS)/sizeof(POWER_LEVELS[0]);

// Background/overlay intensities (fraction of max) for Protocols 4 and 5
const float BG_EXCITE_FRAC   = 0.50;
const float BG_INHIB_FRAC    = 0.50;
const float OVERLAY_FRAC     = 0.50;  // intensity of the opposite channel during overlay

// -----------------------
// State / helpers
// -----------------------
unsigned long now_ms() { return millis(); }

volatile bool abortFlag = false;

int fracToPWM(float frac, int max_pwm) {
  if (frac <= 0) return 0;
  if (frac >= 1) return max_pwm;
  int v = (int)(frac * max_pwm + 0.5f);
  if (v < 0) v = 0;
  if (v > max_pwm) v = max_pwm;
  return v;
}

void setExcite(int pwm)  { analogWrite(ControlPin_excite,  pwm); }
void setInhibit(int pwm) { analogWrite(ControlPin_inhibit, pwm); }
void allOff()            { setExcite(0); setInhibit(0); }

void logHeader() {
  Serial.println("# t_ms,raw,excite_pwm,inhibit_pwm,label");
}

void logLine(const char* label) {
  unsigned long t = now_ms();
  int raw = analogRead(InputPin);
  int pwmE = 0; int pwmI = 0;
  // Read back last set values is not native; store shadow if needed.
  // Simpler approach: track as globals.
}

int g_lastExcite = 0;
int g_lastInhibit = 0;

void setExciteTracked(int pwm)  { g_lastExcite = pwm;  setExcite(pwm); }
void setInhibitTracked(int pwm) { g_lastInhibit = pwm; setInhibit(pwm); }

void logCSV(const char* label) {
  unsigned long t = now_ms();
  int raw = analogRead(InputPin);
  Serial.print(t); Serial.print(',');
  Serial.print(raw); Serial.print(',');
  Serial.print(g_lastExcite); Serial.print(',');
  Serial.print(g_lastInhibit); Serial.print(',');
  Serial.println(label);
}

void waitWithLogging(unsigned long duration_ms, const char* label) {
  unsigned long t0 = now_ms();
  unsigned long nextLog = t0;
  while (!abortFlag && (now_ms() - t0 < duration_ms)) {
    unsigned long t = now_ms();
    if (t >= nextLog) {
      logCSV(label);
      nextLog += LOG_INTERVAL_MS;
    }
    // allow serial commands (abort)
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { abortFlag = true; }
    }
  }
}

// -----------------------
// Protocols
// -----------------------

// 1) Baseline: 5 min with logging
void protocol_baseline() {
  abortFlag = false;
  allOff();
  Serial.println("# BEGIN BASELINE (5 min)");
  logHeader();
  waitWithLogging(BASELINE_MS, "BASELINE");
  allOff();
  Serial.println("# END BASELINE");
}

// Helper to run a single fixed segment at specified PWMs with logging
void runSegment(unsigned long duration_ms, int pwmExcite, int pwmInhib, const char* label) {
  abortFlag = false;
  setExciteTracked(pwmExcite);
  setInhibitTracked(pwmInhib);
  waitWithLogging(duration_ms, label);
  // Leave outputs as they are; caller decides to turn off
}

// 2) Step sweeps: EXCITATION channel
void protocol_stepSweep_excite() {
  abortFlag = false;
  Serial.println("# BEGIN STEP SWEEP (EXCITATION)");
  logHeader();
  for (int r = 0; r < REPEATS_PER_LEVEL && !abortFlag; r++) {
    for (int i = 0; i < N_LEVELS && !abortFlag; i++) {
      // ITI
      allOff();
      waitWithLogging(ITI_MS, "ITI");
      // Step
      int pwmE = fracToPWM(POWER_LEVELS[i], MAX_PWM_EXCITE);
      setInhibitTracked(0);
      setExciteTracked(pwmE);
      char label[64];
      snprintf(label, sizeof(label), "STEP_EXCITE_p=%.2f_rep=%d", POWER_LEVELS[i], r+1);
      waitWithLogging(STEP_DURATION_MS, label);
      // Return to zero (end-of-trial marker with one more second of logging for separation? optional)
      allOff();
      logCSV("STEP_EXCITE_END");
    }
  }
  allOff();
  Serial.println("# END STEP SWEEP (EXCITATION)");
}

// 3) Step sweeps: INHIBITION channel
void protocol_stepSweep_inhibit() {
  abortFlag = false;
  Serial.println("# BEGIN STEP SWEEP (INHIBITION)");
  logHeader();
  for (int r = 0; r < REPEATS_PER_LEVEL && !abortFlag; r++) {
    for (int i = 0; i < N_LEVELS && !abortFlag; i++) {
      // ITI
      allOff();
      waitWithLogging(ITI_MS, "ITI");
      // Step
      int pwmI = fracToPWM(POWER_LEVELS[i], MAX_PWM_INHIB);
      setExciteTracked(0);
      setInhibitTracked(pwmI);
      char label[64];
      snprintf(label, sizeof(label), "STEP_INHIB_p=%.2f_rep=%d", POWER_LEVELS[i], r+1);
      waitWithLogging(STEP_DURATION_MS, label);
      // Return to zero
      allOff();
      logCSV("STEP_INHIB_END");
    }
  }
  allOff();
  Serial.println("# END STEP SWEEP (INHIBITION)");
}

// 4) Overlay: 10 s fixed EXCITATION with 3 s INHIBITION in the middle
void protocol_overlay_exc_bg_with_inhib_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (EXC bg 10s + Inh 3s mid)");
  logHeader();
  int bgE   = fracToPWM(BG_EXCITE_FRAC,  MAX_PWM_EXCITE);
  int midI  = fracToPWM(OVERLAY_FRAC,    MAX_PWM_INHIB);
  unsigned long t0 = now_ms();
  unsigned long nextLog = t0;
  while (!abortFlag && (now_ms() - t0 < OVERLAY_TOTAL_MS)) {
    unsigned long t = now_ms();
    unsigned long dt = t - t0;
    // base excitation
    setExciteTracked(bgE);
    // overlay inhibition during middle window
    if (dt >= OVERLAY_START_MS && dt < OVERLAY_END_MS) {
      setInhibitTracked(midI);
      if (dt < OVERLAY_START_MS + LOG_INTERVAL_MS) { logCSV("OVERLAY:INHIB_ON"); }
    } else {
      setInhibitTracked(0);
      if (dt < LOG_INTERVAL_MS) { logCSV("OVERLAY:START"); }
    }
    if (t >= nextLog) {
      logCSV("OVERLAY");
      nextLog += LOG_INTERVAL_MS;
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { abortFlag = true; }
    }
  }
  allOff();
  Serial.println("# END OVERLAY (EXC bg + INH mid)");
}

// 5) Overlay: 10 s fixed INHIBITION with 3 s EXCITATION in the middle
void protocol_overlay_inh_bg_with_exc_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (INH bg 10s + Exc 3s mid)");
  logHeader();
  int bgI   = fracToPWM(BG_INHIB_FRAC,   MAX_PWM_INHIB);
  int midE  = fracToPWM(OVERLAY_FRAC,    MAX_PWM_EXCITE);
  unsigned long t0 = now_ms();
  unsigned long nextLog = t0;
  while (!abortFlag && (now_ms() - t0 < OVERLAY_TOTAL_MS)) {
    unsigned long t = now_ms();
    unsigned long dt = t - t0;
    // base inhibition
    setInhibitTracked(bgI);
    // overlay excitation during middle window
    if (dt >= OVERLAY_START_MS && dt < OVERLAY_END_MS) {
      setExciteTracked(midE);
      if (dt < OVERLAY_START_MS + LOG_INTERVAL_MS) { logCSV("OVERLAY:EXCITE_ON"); }
    } else {
      setExciteTracked(0);
      if (dt < LOG_INTERVAL_MS) { logCSV("OVERLAY:START"); }
    }
    if (t >= nextLog) {
      logCSV("OVERLAY");
      nextLog += LOG_INTERVAL_MS;
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { abortFlag = true; }
    }
  }
  allOff();
  Serial.println("# END OVERLAY (INH bg + EXC mid)");
}

// 6) Manual 2 s EXCITATION at MAX
void manual_excitation_max_2s() {
  abortFlag = false;
  Serial.println("# MANUAL: EXCITE 2s @ MAX");
  logHeader();
  int pwmE = MAX_PWM_EXCITE;
  setInhibitTracked(0);
  setExciteTracked(pwmE);
  waitWithLogging(2000, "MANUAL_EXCITE_MAX");
  allOff();
  logCSV("MANUAL_EXCITE_END");
}

// 7) Manual 2 s INHIBITION at MAX
void manual_inhibition_max_2s() {
  abortFlag = false;
  Serial.println("# MANUAL: INHIBIT 2s @ MAX");
  logHeader();
  int pwmI = MAX_PWM_INHIB;
  setExciteTracked(0);
  setInhibitTracked(pwmI);
  waitWithLogging(2000, "MANUAL_INHIBIT_MAX");
  allOff();
  logCSV("MANUAL_INHIBIT_END");
}

// -----------------------
// Setup / Loop
// -----------------------
void printHelp() {
  Serial.println();
  Serial.println(F("==== Shun_tuning_v1 controls ===="));
  Serial.println(F("  '1' : 5 min baseline"));
  Serial.println(F("  '2' : EXCITE step sweep (3s steps; ITI 5s; 5 repeats each level)"));
  Serial.println(F("  '3' : INHIBIT step sweep (3s steps; ITI 5s; 5 repeats each level)"));
  Serial.println(F("  '4' : 10s EXC bg + 3s INH in the middle"));
  Serial.println(F("  '9' : 10s INH bg + 3s EXC in the middle"));
  Serial.println(F("  '5' : Manual EXCITE 2s @ MAX"));
  Serial.println(F("  '6' : Manual INHIBIT 2s @ MAX"));
  Serial.println(F("  'x' : Abort current protocol (also sets outputs to zero)"));
  Serial.println(F("  'h' or '?' : help"));
  Serial.println();
  Serial.print (F("  PWM caps: MAX_PWM_EXCITE=")); Serial.print(MAX_PWM_EXCITE);
  Serial.print (F(", MAX_PWM_INHIB=")); Serial.println(MAX_PWM_INHIB);
  Serial.print (F("  Power levels: "));
  for (int i=0;i<N_LEVELS;i++) { Serial.print(POWER_LEVELS[i],2); if (i<N_LEVELS-1) Serial.print(", "); }
  Serial.println();
}

void setup() {
  Serial.begin(500000);
  // Wiring setup as in your PID file
  pinMode(InputPin, INPUT);
  pinMode(ControlPin_inhibit, OUTPUT);
  pinMode(ControlPin_excite, OUTPUT);
  pinMode(TargetPin, INPUT);
  pinMode(ClampOnPin, INPUT);

  pinMode(FakeGND, OUTPUT);
  pinMode(FakeVCC, OUTPUT);
  digitalWrite(FakeGND, LOW);   // A2 as GND
  digitalWrite(FakeVCC, HIGH);  // A3 as Vcc

  allOff();
  delay(50);
  printHelp();
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case '1': protocol_baseline(); break;
      case '2': protocol_stepSweep_excite(); break;
      case '3': protocol_stepSweep_inhibit(); break;
      case '4': protocol_overlay_exc_bg_with_inhib_mid(); break;
      case '9': protocol_overlay_inh_bg_with_exc_mid(); break;
      case '5': manual_excitation_max_2s(); break;
      case '6': manual_inhibition_max_2s(); break;
      case 'x': abortFlag = true; allOff(); Serial.println("# ABORTED"); break;
      case 'h': case '?': printHelp(); break;
      default: break; // ignore other keys
    }
  }
  // idle: do nothing
}
