// Shun_Tune_v2.ino
// Open-loop tuning for two-channel (excite/inhibit) photometry system
// Based on OptoSweeps structure with simplified state machine

#define Idle 0
#define ExciteStepSweep 1
#define InhibitStepSweep 2
#define OverlayExcInh 3
#define OverlayInhExc 4
#define ExciteManual 5
#define InhibitManual 6

// Pin definitions
const byte InputPin = A1;
const byte ControlPin_inhibit = 7;
const byte ControlPin_excite = 2;
const byte TargetPin = 3;
const byte ClampOnPin = 8;
const byte FakeGND = A2;
const byte FakeVCC = A3;

// Config
const unsigned long STEP_DURATION_MS = 3000;  // 3 s
const unsigned long ITI_MS = 5000;            // 5 s
const int REPEATS_PER_LEVEL = 5;
const unsigned long OVERLAY_TOTAL_MS = 9000;  // 9 s
const unsigned long OVERLAY_MIDDLE_MS = 3000; // 3 s
const unsigned long OVERLAY_START_MS = (OVERLAY_TOTAL_MS - OVERLAY_MIDDLE_MS) / 2;
const unsigned long MANUAL_STIM_MS = 2000;   // 2 s

// Max PWM values
const int max_excite = 50;
const int max_inhibit = 80;
const int N_LEVELS = 5;

// PWM arrays (generated automatically from max values)
int excite_pwm[N_LEVELS];
int inhibit_pwm[N_LEVELS];

// State
char SerialInput = '0';
static int state = Idle;
unsigned long Start = 0;
unsigned long End = 0;
unsigned long ITI_start = 0;

// Step sweep tracking
int currentRepeat = 0;
int currentLevel = 0;

// Overlay tracking
int overlayRepeat = 0;

// State variables for protocols
boolean abortFlag = false;
int g_excite = 0;
int g_inhibit = 0;

// Helper functions
void setExcite(int pwm) {
  g_excite = pwm;
  analogWrite(ControlPin_excite, pwm);
}

void setInhibit(int pwm) {
  g_inhibit = pwm;
  analogWrite(ControlPin_inhibit, pwm);
}

void allOff() {
  setExcite(0);
  setInhibit(0);
}

void waitWithAbort(unsigned long duration_ms) {
  unsigned long t0 = millis();
  while (!abortFlag && (millis() - t0 < duration_ms)) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        abortFlag = true;
      }
    }
  }
}

// Protocol implementations
void start_excite_step_sweep() {
  abortFlag = false;
  Serial.println("# BEGIN EXCITE STEP SWEEP");
  currentRepeat = 0;
  currentLevel = 0;
  ITI_start = millis();
  state = ExciteStepSweep;
}

void start_inhibit_step_sweep() {
  abortFlag = false;
  Serial.println("# BEGIN INHIBIT STEP SWEEP");
  currentRepeat = 0;
  currentLevel = 0;
  ITI_start = millis();
  state = InhibitStepSweep;
}

void start_overlay_exc_bg_with_inhib_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (EXC bg @ max + INH mid varying levels)");
  overlayRepeat = 0;
  currentLevel = 0;
  ITI_start = millis();
  state = OverlayExcInh;
}

void start_overlay_inh_bg_with_exc_mid() {
  abortFlag = false;
  Serial.println("# BEGIN OVERLAY (INH bg @ max + EXC mid varying levels)");
  overlayRepeat = 0;
  currentLevel = 0;
  ITI_start = millis();
  state = OverlayInhExc;
}

void manual_excitation_max_2s() {
  abortFlag = false;
  Serial.print("# MANUAL: EXCITE 2s @ ");
  Serial.println(max_excite);
  setInhibit(0);
  setExcite(max_excite);
  waitWithAbort(MANUAL_STIM_MS);
  allOff();
  Serial.println("# END MANUAL EXCITE");
}

void manual_inhibition_max_2s() {
  abortFlag = false;
  Serial.print("# MANUAL: INHIBIT 2s @ ");
  Serial.println(max_inhibit);
  setExcite(0);
  setInhibit(max_inhibit);
  waitWithAbort(MANUAL_STIM_MS);
  allOff();
  Serial.println("# END MANUAL INHIBIT");
}

void generatePWMArrays() {
  // Generate evenly spaced PWM values from 10 to max value
  // For 5 levels, creates: fraction = 1/6, 2/6, 3/6, 4/6, 5/6
  // This gives values evenly spaced between 10 and max
  for (int i = 0; i < N_LEVELS; i++) {
    float fraction = (float)(i + 1) / (float)(N_LEVELS + 1);
    excite_pwm[i] = 10 + (int)(fraction * (max_excite - 10) + 0.5);
    inhibit_pwm[i] = 10 + (int)(fraction * (max_inhibit - 10) + 0.5);
  }
}

void printHelp() {
  Serial.println();
  Serial.println(F("==== Shun_Tune_v2 ===="));
  Serial.println(F("Step sweeps (3s steps, 5s ITI, 5 repeats):"));
  Serial.println(F("  '1' : EXCITE step sweep"));
  Serial.println(F("  '2' : INHIBIT step sweep"));
  Serial.println(F("Overlay (9s background @ max + 3s middle varying levels, 5 repeats):"));
  Serial.println(F("  '3' : Background EXCITE @ max + 3s INHIBIT overlay"));
  Serial.println(F("  '4' : Background INHIBIT @ max + 3s EXCITE overlay"));
  Serial.println(F("Manual (2s @ max):"));
  char msg1[64], msg2[64];
  snprintf(msg1, sizeof(msg1), "  '5' : Manual EXCITE @ %d", max_excite);
  snprintf(msg2, sizeof(msg2), "  '6' : Manual INHIBIT @ %d", max_inhibit);
  Serial.println(msg1);
  Serial.println(msg2);
  Serial.println(F("  'x' : Abort / outputs to zero"));
  Serial.println(F("  'h' or '?' : help"));
  Serial.println();
  char msg[128];
  snprintf(msg, sizeof(msg), "  Max values: excite=%d, inhibit=%d", max_excite, max_inhibit);
  Serial.println(msg);
  Serial.print(F("  Excite PWM levels: "));
  for (int i=0; i<N_LEVELS; i++) {
    Serial.print(excite_pwm[i]);
    if (i<N_LEVELS-1) Serial.print(", ");
  }
  Serial.println();
  Serial.print(F("  Inhibit PWM levels: "));
  for (int i=0; i<N_LEVELS; i++) {
    Serial.print(inhibit_pwm[i]);
    if (i<N_LEVELS-1) Serial.print(", ");
  }
  Serial.println();
}

void setup() {
  Serial.begin(500000);
  
  // Generate PWM arrays from max values
  generatePWMArrays();
  
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
  
  switch (state) {
    case Idle:
      if (Serial.available()) {
        SerialInput = Serial.read();
        switch (SerialInput) {
          case '1': 
            start_excite_step_sweep();
            break;
          case '2': 
            start_inhibit_step_sweep();
            break;
          case '3': 
            start_overlay_exc_bg_with_inhib_mid();
            break;
          case '4': 
            start_overlay_inh_bg_with_exc_mid();
            break;
          case '5': 
            manual_excitation_max_2s();
            state = Idle;
            break;
          case '6': 
            manual_inhibition_max_2s();
            state = Idle;
            break;
          case 'x': 
            abortFlag = true; 
            allOff(); 
            Serial.println("# ABORTED"); 
            break;
          case 'h': case '?': 
            printHelp(); 
            break;
          default: 
            break;
        }
      }
      break;
      
    case ExciteStepSweep:
      if (millis() - ITI_start > ITI_MS) {
        allOff();
        setInhibit(0);
        setExcite(excite_pwm[currentLevel]);
        
        Serial.print("# EXCITE PWM: ");
        Serial.print(excite_pwm[currentLevel]);
        Serial.print(" (Level ");
        Serial.print(currentLevel + 1);
        Serial.print("/");
        Serial.print(N_LEVELS);
        Serial.print(", Rep ");
        Serial.print(currentRepeat + 1);
        Serial.print("/");
        Serial.println(REPEATS_PER_LEVEL);
        
        waitWithAbort(STEP_DURATION_MS);
        allOff();
        
        // Move to next level or repeat
        currentLevel++;
        if (currentLevel >= N_LEVELS) {
          currentLevel = 0;
          currentRepeat++;
          if (currentRepeat >= REPEATS_PER_LEVEL) {
            Serial.println("# END EXCITE STEP SWEEP");
            allOff();
            state = Idle;
            break;
          }
        }
        
        ITI_start = millis();
      }
      break;
      
    case InhibitStepSweep:
      if (millis() - ITI_start > ITI_MS) {
        allOff();
        setExcite(0);
        setInhibit(inhibit_pwm[currentLevel]);
        
        Serial.print("# INHIBIT PWM: ");
        Serial.print(inhibit_pwm[currentLevel]);
        Serial.print(" (Level ");
        Serial.print(currentLevel + 1);
        Serial.print("/");
        Serial.print(N_LEVELS);
        Serial.print(", Rep ");
        Serial.print(currentRepeat + 1);
        Serial.print("/");
        Serial.println(REPEATS_PER_LEVEL);
        
        waitWithAbort(STEP_DURATION_MS);
        allOff();
        
        // Move to next level or repeat
        currentLevel++;
        if (currentLevel >= N_LEVELS) {
          currentLevel = 0;
          currentRepeat++;
          if (currentRepeat >= REPEATS_PER_LEVEL) {
            Serial.println("# END INHIBIT STEP SWEEP");
            allOff();
            state = Idle;
            break;
          }
        }
        
        ITI_start = millis();
      }
      break;
      
    case OverlayExcInh:
      if (millis() - ITI_start > ITI_MS) {
        Serial.print("# OVERLAY EXC+INH: inhibit PWM ");
        Serial.print(inhibit_pwm[currentLevel]);
        Serial.print(" (Level ");
        Serial.print(currentLevel + 1);
        Serial.print("/");
        Serial.print(N_LEVELS);
        Serial.print(", Rep ");
        Serial.print(overlayRepeat + 1);
        Serial.print("/");
        Serial.println(REPEATS_PER_LEVEL);
        
        unsigned long start = millis();
        while (!abortFlag && (millis() - start < OVERLAY_TOTAL_MS)) {
          unsigned long dt = millis() - start;
          
          setExcite(max_excite);
          if (dt >= OVERLAY_START_MS && dt < (OVERLAY_START_MS + OVERLAY_MIDDLE_MS)) {
            setInhibit(inhibit_pwm[currentLevel]);
          } else {
            setInhibit(0);
          }
          
          if (Serial.available()) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') { abortFlag = true; }
          }
        }
        
        allOff();
        
        // Move to next level or repeat
        currentLevel++;
        if (currentLevel >= N_LEVELS) {
          currentLevel = 0;
          overlayRepeat++;
          if (overlayRepeat >= REPEATS_PER_LEVEL) {
            Serial.println("# END OVERLAY (EXC bg + INH mid)");
            allOff();
            state = Idle;
            break;
          }
        }
        
        ITI_start = millis();
      }
      break;
      
    case OverlayInhExc:
      if (millis() - ITI_start > ITI_MS) {
        Serial.print("# OVERLAY INH+EXC: excite PWM ");
        Serial.print(excite_pwm[currentLevel]);
        Serial.print(" (Level ");
        Serial.print(currentLevel + 1);
        Serial.print("/");
        Serial.print(N_LEVELS);
        Serial.print(", Rep ");
        Serial.print(overlayRepeat + 1);
        Serial.print("/");
        Serial.println(REPEATS_PER_LEVEL);
        
        unsigned long start = millis();
        while (!abortFlag && (millis() - start < OVERLAY_TOTAL_MS)) {
          unsigned long dt = millis() - start;
          
          setInhibit(max_inhibit);
          if (dt >= OVERLAY_START_MS && dt < (OVERLAY_START_MS + OVERLAY_MIDDLE_MS)) {
            setExcite(excite_pwm[currentLevel]);
          } else {
            setExcite(0);
          }
          
          if (Serial.available()) {
            char c = Serial.read();
            if (c == 'x' || c == 'X') { abortFlag = true; }
          }
        }
        
        allOff();
        
        // Move to next level or repeat
        currentLevel++;
        if (currentLevel >= N_LEVELS) {
          currentLevel = 0;
          overlayRepeat++;
          if (overlayRepeat >= REPEATS_PER_LEVEL) {
            Serial.println("# END OVERLAY (INH bg + EXC mid)");
            allOff();
            state = Idle;
            break;
          }
        }
        
        ITI_start = millis();
      }
      break;
      
    default:
      state = Idle;
      break;
  }
  
  // Check for abort during protocols
  if (state == ExciteStepSweep || state == InhibitStepSweep || 
      state == OverlayExcInh || state == OverlayInhExc) {
    if (Serial.available()) {
      char c = Serial.read();
      if (c == 'x' || c == 'X') {
        abortFlag = true;
        state = Idle;
        allOff();
        Serial.println("# ABORTED");
      }
    }
  }
}

