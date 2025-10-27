/* 
  Shun_Tune_v1.ino
  Open-loop tuning for two-channel (excite/inhibit) system.
  Same wiring as Shun_PID_v15.ino.
  
  Serial commands:
    '1' : 5 min baseline
    '2' : EXCITE step sweep (3s steps, 5s ITI, 5 repeats)
    '3' : INHIBIT step sweep (3s steps, 5s ITI, 5 repeats)
    '4' : 10s EXCITE + 3s INHIBIT overlay
    '9' : 10s INHIBIT + 3s EXCITE overlay
    '5' : Manual EXCITE 2s @ 255
    '6' : Manual INHIBIT 2s @ 255
    'x' : Abort / set outputs to zero
    'h' : help
  
  Logging: CSV at 500000 baud
    t_ms, raw, excite_pwm, inhibit_pwm, label
*/

// Pin definitions
const byte InputPin           = A1;
const byte ControlPin_inhibit = 7;
const byte ControlPin_excite  = 2;
const byte TargetPin          = 3;
const byte ClampOnPin         = 8;
const byte FakeGND            = A2;
const byte FakeVCC            = A3;

// User config
const unsigned long LOG_INTERVAL_MS = 10;     // ~100 Hz
const unsigned long BASELINE_MS     = 5UL * 60UL * 1000UL; // 5 min

// Step-sweep parameters
const unsigned long STEP_DURATION_MS = 3000;  // 3 s
const unsigned long ITI_MS           = 5000;  // 5 s
const int REPEATS_PER_LEVEL          = 5;

// Overlay parameters
const unsigned long OVERLAY_TOTAL_MS   = 10000; // 10 s
const unsigned long OVERLAY_MIDDLE_MS  = 3000;  // 3 s
const unsigned long OVERLAY_START_MS   = (OVERLAY_TOTAL_MS - OVERLAY_MIDDLE_MS) / 2;
const unsigned long OVERLAY_END_MS     = OVERLAY_START_MS + OVERLAY_MIDDLE_MS;

// PWM values (0-255)
int excite_pwm[] = {13, 26, 51, 77, 128, 204, 255}; // PWM levels
int inhibit_pwm[] = {13, 26, 51, 77, 128, 204, 255}; // PWM levels
const int N_LEVELS = sizeof(excite_pwm)/sizeof(excite_pwm[0]);

// Overlay PWM values
int overlay_bg_pwm = 128;
int overlay_mid_pwm = 128;

// State
volatile bool abortFlag = false;
int g_excite = 0;
int g_inhibit = 0;

void setExcite(int pwm)  { g_excite = pwm; analogWrite(ControlPin_excite, pwm); }
void setInhibit(int pwm) { g_inhibit = pwm; analogWrite(ControlPin_inhibit, pwm); }
void allOff()            { setExcite(0); setInhibit(0); }

void logHeader() {
  Serial.println("# t_ms,raw,excite_pwm,inhibit_pwm,label");
}

void logCSV(const char* label) {
  Serial.print(millis()); Serial.print(',');
  Serial.print(analogRead(InputPin)); Serial.print(',');
  Serial.print(g_excite); Serial.print(',');
  Serial.print(g_inhibit); Serial.print(',');
  Serial.println(label);
}

void waitWithLogging(unsigned long duration_ms, const char* label) {
  unsigned long t0 = millis();
  unsigned long nextLog = t0;
  while (!abortFlag && (millis() - t0 < duration_ms)) {
    unsigned long t = millis();
    if (t >= nextLog) {
      logCSV(label);
      nextLog += LOG_INTERVAL_MS;
    }
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') { abortFlag = true; }
    }
  }
}

// Protocols
void protocol_baseline() {
  abortFlag = false;
  allOff();
  Serial.println("# BEGIN BASELINE (5 min)");
  logHeader();
  waitWithLogging(BASELINE_MS, "BASELINE");
  allOff();
  Serial.println("# END BASELINE");
}

void protocol_stepSweep_excite() {
  abortFlag = false;
  Serial.println("# BEGIN STEP SWEEP (EXCITATION)");
  logHeader();
  for (int r = 0; r < REPEATS_PER_LEVEL && !abortFlag; r++) {
    for (int i = 0; i < N_LEVELS && !abortFlag; i++) {
      allOff();
      waitWithLogging(ITI_MS, "ITI");
      
      setInhibit(0);
      setExcite(excite_pwm[i]);
      char label[64];
      snprintf(label, sizeof(label), "EXCITE_pwm=%d_rep=%d", excite_pwm[i], r+1);
      waitWithLogging(STEP_DURATION_MS, label);
      
      allOff();
      logCSV("EXCITE_END");
    }
  }
  allOff();
  Serial.println("# END STEP SWEEP (EXCITATION)");
}

void protocol_stepSweep_inhibit() {
  abortFlag = false;
  Serial.println("# BEGIN STEP SWEEP (INHIBITION)");
  logHeader();
  for (int r = 0; r < REPEATS_PER_LEVEL && !abortFlag; r++) {
    for (int i = 0; i < N_LEVELS && !abortFlag; i++) {
      allOff();
      waitWithLogging(ITI_MS, "ITI");
      
      setExcite(0);
      setInhibit(inhibit_pwm[i]);
      char label[64];
      snprintf(label, sizeof(label), "INHIBIT_pwm=%d_rep=%d", inhibit_pwm[i], r+1);
      waitWithLogging(STEP_DURATION_MS, label);
      
      allOff();
      logCSV("INHIBIT_END");
    }
  }
  allOff();
  Serial.println("# END STEP SWEEP (INHIBITION)");
}

void protocol_overlay_exc_bg_with_inhib_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (EXC bg + INH mid)");
  logHeader();
  unsigned long t0 = millis();
  unsigned long nextLog = t0;
  while (!abortFlag && (millis() - t0 < OVERLAY_TOTAL_MS)) {
    unsigned long t = millis();
    unsigned long dt = t - t0;
    
    setExcite(overlay_bg_pwm);
    if (dt >= OVERLAY_START_MS && dt < OVERLAY_END_MS) {
      setInhibit(overlay_mid_pwm);
      if (dt < OVERLAY_START_MS + LOG_INTERVAL_MS) { logCSV("OVERLAY:INHIB_ON"); }
    } else {
      setInhibit(0);
      if (dt < LOG_INTERVAL_MS) { logCSV("OVERLAY:START"); }
    }
    
    if (t >= nextLog) {
      logCSV("OVERLAY_EXC");
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

void protocol_overlay_inh_bg_with_exc_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (INH bg + EXC mid)");
  logHeader();
  unsigned long t0 = millis();
  unsigned long nextLog = t0;
  while (!abortFlag && (millis() - t0 < OVERLAY_TOTAL_MS)) {
    unsigned long t = millis();
    unsigned long dt = t - t0;
    
    setInhibit(overlay_bg_pwm);
    if (dt >= OVERLAY_START_MS && dt < OVERLAY_END_MS) {
      setExcite(overlay_mid_pwm);
      if (dt < OVERLAY_START_MS + LOG_INTERVAL_MS) { logCSV("OVERLAY:EXCITE_ON"); }
    } else {
      setExcite(0);
      if (dt < LOG_INTERVAL_MS) { logCSV("OVERLAY:START"); }
    }
    
    if (t >= nextLog) {
      logCSV("OVERLAY_INH");
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

void manual_excitation_max_2s() {
  abortFlag = false;
  Serial.println("# MANUAL: EXCITE 2s @ 255");
  logHeader();
  setInhibit(0);
  setExcite(255);
  waitWithLogging(2000, "MANUAL_EXCITE");
  allOff();
  logCSV("MANUAL_EXCITE_END");
}

void manual_inhibition_max_2s() {
  abortFlag = false;
  Serial.println("# MANUAL: INHIBIT 2s @ 255");
  logHeader();
  setExcite(0);
  setInhibit(255);
  waitWithLogging(2000, "MANUAL_INHIBIT");
  allOff();
  logCSV("MANUAL_INHIBIT_END");
}

void printHelp() {
  Serial.println();
  Serial.println(F("==== Shun_Tune_v1 ===="));
  Serial.println(F("  '1' : 5 min baseline"));
  Serial.println(F("  '2' : EXCITE step sweep (3s steps, 5s ITI, 5 repeats)"));
  Serial.println(F("  '3' : INHIBIT step sweep (3s steps, 5s ITI, 5 repeats)"));
  Serial.println(F("  '4' : 10s EXCITE + 3s INHIBIT overlay"));
  Serial.println(F("  '9' : 10s INHIBIT + 3s EXCITE overlay"));
  Serial.println(F("  '5' : Manual EXCITE 2s @ 255"));
  Serial.println(F("  '6' : Manual INHIBIT 2s @ 255"));
  Serial.println(F("  'x' : Abort / outputs to zero"));
  Serial.println(F("  'h' or '?' : help"));
  Serial.println();
  Serial.print(F("  Excite PWM levels: "));
  for (int i=0; i<N_LEVELS; i++) { Serial.print(excite_pwm[i]); if (i<N_LEVELS-1) Serial.print(", "); }
  Serial.println();
  Serial.print(F("  Inhibit PWM levels: "));
  for (int i=0; i<N_LEVELS; i++) { Serial.print(inhibit_pwm[i]); if (i<N_LEVELS-1) Serial.print(", "); }
  Serial.println();
}

void setup() {
  Serial.begin(500000);
  
  pinMode(InputPin, INPUT);
  pinMode(ControlPin_inhibit, OUTPUT);
  pinMode(ControlPin_excite, OUTPUT);
  pinMode(TargetPin, INPUT);
  pinMode(ClampOnPin, INPUT);
  pinMode(FakeGND, OUTPUT);
  pinMode(FakeVCC, OUTPUT);
  
  digitalWrite(FakeGND, LOW);
  digitalWrite(FakeVCC, HIGH);
  
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
      default: break;
    }
  }
}
