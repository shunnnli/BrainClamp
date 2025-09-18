// Shun_OptoSweeps_Frequency
// Shun Li, 2023/07/18
// Modified from code by Ricardo López, 2022/12/15

// Give pulses of opto stimulation at fixed define pulse duration and frequency
// but different frequency

#define Idle 0 //Defining constants and assigning values. These are variables with fixed values defined before the program runs.
#define ITI_State 1
#define OptoSweeps 2
#define SweepInterval 3
//#define SecondCueOn 4
//#define SolenoidOn 5
//#define SolenoidOff 6
//#define TurnOffLickLeft 7
//#define RestartClock 8
//#define TimeOut 9

#include <math.h>
// Define opto stim parameters
int RepeatPerPattern = 30; //Total number of repeats per pattern

// Blue laser parameters
int blue_pwm[] = {20, 30, 40}; // PWM frequencies in Hz
float blue_duration[] = {0.1, 0.5, 1}; // durations in seconds
int blue_patterns = 3; // number of blue patterns

// Red laser parameters  
int red_pwm[] = {45, 55}; // PWM frequencies in Hz
float red_duration[] = {0.1, 0.5, 1}; // durations in seconds
int red_patterns = 2; // number of red patterns

unsigned long PulseDuration = 5; //Total duration of each pulse
// Initialize opto stim params
unsigned long PulseFreq = 0; //Current pulse frequency in the current sweep
unsigned long PulseInterval = 0;
int PulseNum = 0; //number of laser trials
int LaserColor = 0;
float StimDuration = 0; // current stimulation duration in seconds
int PWMIntensity = 0; // PWM intensity (0-255), default 50%

// Initialize pattern counting (supports different counts of freqs/durations)
const int nBlueFreqs = sizeof(blue_pwm) / sizeof(blue_pwm[0]);
const int nBlueDurs  = sizeof(blue_duration) / sizeof(blue_duration[0]);
const int nRedFreqs  = sizeof(red_pwm) / sizeof(red_pwm[0]);
const int nRedDurs   = sizeof(red_duration) / sizeof(red_duration[0]);

// Use fixed maximums for static allocation; index only up to n* values above
unsigned long BluePatternCount[3][3] = {0};
unsigned long RedPatternCount[3][3]  = {0};

// Pattern completion tracking
int TotalPatterns = (nBlueFreqs * nBlueDurs) + (nRedFreqs * nRedDurs);
int CompletedPatterns = 0;
boolean AllPatternsCompleted = false;


// Set up parameters for the behavior - REMOVED UNUSED VARIABLES

// Time params
unsigned long ITI1 = 4000;
unsigned long ITI2 = 4000;
unsigned long ITI = 0; // ITI = random(ITI1,ITI2)

// Input output pin description //Constants prevent specific object/method()/variable to modify data
const byte ShutterBlue = 7; //1=blue shutter open, 0=closed
const byte ShutterRed = 2; //1=red shutter open, 0=closed [This may be important for laser stimulation]

// Initialize real time variables //
char SerialInput = '0'; //for incoming serial data

// Constantly occuring stuffs
static int state = 0 ; // MAIN behavior state variable for running behavior task
// Opto stim parameters
unsigned long TimerPulse = 0;
int OptoNow = 0;
unsigned long OptoInterval = 0;
boolean OptoDelivering = false;
// Continuous stim control
boolean StimActive = false;
unsigned long StimStartMs = 0;
// REMOVED UNUSED SHUTTER SOUND PARAMS

// REMOVED UNUSED VARIABLES: lick detection, outcome related

// Timestamp related
unsigned long ITI_start = 0; //timestamp for beginning of ITI
unsigned long Start = 0; //timestamp for starting the session (used for triggering camera)
unsigned long End = 0; //timestamp for ending the session

void setup() { //Setup function called when sketch starts. The setup function is used to initialize variables, pin modes, etc.

  Serial.begin(115200); //Set the data rate in bits per second. 115200 is default

  pinMode(ShutterBlue, OUTPUT);
  pinMode(ShutterRed, OUTPUT);

  // Initialize opto params
  TimerPulse = 0;
  OptoNow = 0;

  digitalWrite(ShutterBlue, LOW);
  digitalWrite(ShutterRed, LOW);
  randomSeed(analogRead(3));

  Serial.println("-----------------------------------------------------------------");
  Serial.println("Manual check: 1 -> reward; 2 -> punishment; 3 -> blue; 4 -> red");
  Serial.println("Laser shutter: 5 -> red stim; 6 -> blue stim");
  Serial.println("Trial start/stop: 8 -> start; 9 -> end");
  Serial.println("-----------------------------------------------------------------");
}


void loop() {

  switch (state) {
    //state 0: Idle state until Start button pushed
    case Idle: //Recall Idle = 0 so we begin in our first state here
      if (SerialInput == '8') {
        Start = millis();
        End = 0;
        Serial.print("TASK STARTED AT ");
        Serial.print("\t");
        Serial.println(millis());
        digitalWrite(ShutterBlue, LOW);
        digitalWrite(ShutterRed, LOW);
        state = 1;
      }
      break;

    //state 1: Determine the intertrial interval
    case ITI_State:
      ITI_start = millis();
      ITI = random(ITI1,ITI2);
      state = 2;
      break;

    //state 2: Opto stimulation with random PWM frequency and duration combinations
    case OptoSweeps:
      if (millis()-ITI_start > ITI){
        // Randomly choose laser color (0=blue, 1=red)
        int laserChoice = random(0, 2);
        
        if (laserChoice == 0) {
          // Blue laser
          LaserColor = ShutterBlue;
          int rand_pwm = random(0, nBlueFreqs);
          int rand_dur = random(0, nBlueDurs);
          PWMIntensity = blue_pwm[rand_pwm];
          StimDuration = blue_duration[rand_dur];
          BluePatternCount[rand_pwm][rand_dur] += 1;
          
          // Deliver stim
          Serial.print("Blue stim #");
          Serial.print(BluePatternCount[rand_pwm][rand_dur]);
          Serial.print(": PWM ");
          Serial.print(PWMIntensity);
          Serial.print(" for ");
          Serial.print(StimDuration);
          Serial.println("s");
          giveOpto();

        } else {
          // Red laser
          LaserColor = ShutterRed;
          int rand_pwm = random(0, nRedFreqs);
          int rand_dur = random(0, nRedDurs);
          PWMIntensity = red_pwm[rand_pwm];
          StimDuration = red_duration[rand_dur];
          RedPatternCount[rand_pwm][rand_dur] += 1;
          
          // Deliver stim
          Serial.print("Red stim #");
          Serial.print(RedPatternCount[rand_pwm][rand_dur]);
          Serial.print(": PWM ");
          Serial.print(PWMIntensity);
          Serial.print(" for ");
          Serial.print(StimDuration);
          Serial.println("s");
          giveOpto();
        }
        
        // Check if all patterns are completed (counts per combination)
        CompletedPatterns = 0;
        for (int i = 0; i < nBlueFreqs; i++) {
          for (int j = 0; j < nBlueDurs; j++) {
            if (BluePatternCount[i][j] >= (unsigned long)RepeatPerPattern) CompletedPatterns++;
          }
        }
        for (int i = 0; i < nRedFreqs; i++) {
          for (int j = 0; j < nRedDurs; j++) {
            if (RedPatternCount[i][j] >= (unsigned long)RepeatPerPattern) CompletedPatterns++;
          }
        }
        
        if (CompletedPatterns >= TotalPatterns) {
          state = 3; // Go to SweepInterval state
        } else {
          state = 1; // Continue with ITI
        }
      }
      break;

    // state 3: interval between each sweep pattern
    case SweepInterval:
      Serial.println("Finished: all opto sweeps");
      state = 0;
      break;
  }
  // END OF SWITCH STRCUTURE //


  // Ending Task //
  if (SerialInput == '9') {
    if (End == 0) {
      Serial.print("TASK ENDED AT ");
      Serial.print("\t");
      Serial.println(millis());
      state = 0;
      End = millis();
      digitalWrite(ShutterBlue, LOW);
      digitalWrite(ShutterRed, LOW);
    }
  }

  if (Serial.available() > 0) {
    // read the incoming byte:
    SerialInput = Serial.read();

    if (SerialInput == '3') {
      digitalWrite(ShutterBlue, HIGH);
      //delay(2000);
      //digitalWrite(ShutterBlue, LOW);
    }

    if (SerialInput == '4') {
      digitalWrite(ShutterRed, HIGH);
      //delay(2000);
      //digitalWrite(ShutterRed, LOW);
    }

  }
}


//********************************************************************************************//
// Assigned upcoming opto delivery
void giveOpto(){
  // Deliver one continuous PWM pulse for the selected duration (blocking)
  analogWrite(LaserColor, PWMIntensity);
  delay((unsigned long)(StimDuration * 1000.0));
  analogWrite(LaserColor, 0);
}