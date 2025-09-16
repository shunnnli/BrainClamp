// Shun_OptoSweeps_Frequency
// Shun Li, 2023/07/18
// Modified from code by Ricardo López, 2022/12/15

// Give pulses of opto stimulation at fixed define pulse duration and frequency
// but different frequency

#define Idle 0 //Defining constants and assigning values. These are variables with fixed values defined before the program runs.
#define ITI_State 1
#define OptoSweeps 2
//#define SweepInterval 3
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
int blue_pwm[] = {40, 50, 80}; // PWM frequencies in Hz
float blue_duration[] = {0.1, 0.5, 1.0}; // durations in seconds
int blue_patterns = 3; // number of blue patterns

// Red laser parameters  
int red_pwm[] = {40, 50, 80}; // PWM frequencies in Hz
float red_duration[] = {0.1, 0.5, 1.0}; // durations in seconds
int red_patterns = 3; // number of red patterns

unsigned long PulseDuration = 5; //Total duration of each pulse
// Initialize opto stim params
unsigned long PulseFreq = 0; //Current pulse frequency in the current sweep
unsigned long PulseInterval = 0;
int PulseNum = 0; //number of laser trials
int LaserColor = 22; // ShutterRed(24); ShutterBlue(22)
float StimDuration = 0; // current stimulation duration in seconds

// Initialize pattern counting
unsigned long BluePatternCount[3] = {0,0,0};
unsigned long RedPatternCount[3] = {0,0,0};


// Set up parameters for the behavior - REMOVED UNUSED VARIABLES

// Time params
unsigned long ITI1 = 40000;
unsigned long ITI2 = 40000;
unsigned long ITI = 0; // ITI = random(ITI1,ITI2)

// Input output pin description //Constants prevent specific object/method()/variable to modify data
const byte Sync = 2; // non-periodic sync pulse
const byte ShutterBlue = 7; //1=blue shutter open, 0=closed
const byte ShutterRed = 2; //1=red shutter open, 0=closed [This may be important for laser stimulation]

// Initialize real time variables //
char SerialInput = '0'; //for incoming serial data

// Constantly occuring stuffs
static int state = 0 ; // MAIN behavior state variable for running behavior task
unsigned long TimerSync = 0; //timer for non-periodic sync pulse
int SyncPulseInterval = 1000; //interval for non-periodic sync pulse
int SyncNow = 0; //current sync signal status
// Opto stim parameters
unsigned long TimerPulse = 0;
int OptoNow = 0;
unsigned long OptoInterval = 0;
boolean OptoDelivering = false;
// REMOVED UNUSED SHUTTER SOUND PARAMS

// REMOVED UNUSED VARIABLES: lick detection, outcome related

// Timestamp related
unsigned long ITI_start = 0; //timestamp for beginning of ITI
unsigned long Start = 0; //timestamp for starting the session (used for triggering camera)
unsigned long End = 0; //timestamp for ending the session

void setup() { //Setup function called when sketch starts. The setup function is used to initialize variables, pin modes, etc.

  Serial.begin(115200); //Set the data rate in bits per second. 115200 is default

  pinMode(Sync, OUTPUT); //Configure specified pins as inputs or outputs
  pinMode(ShutterBlue, OUTPUT);
  pinMode(ShutterRed, OUTPUT);

  // Initialize sync params
  Start = millis(); //Number of milliseconds passed since program starts
  TimerSync = millis();
  state = 0; //Sync parameters are defined above beginning at L62
  SyncNow = 0;
  // Initialize opto params
  TimerPulse = 0;
  OptoNow = 0;

  digitalWrite(Sync, LOW); //Set pins off
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
  sync(); //Non period sync pulse (1s width) generation
  opto(); 

  switch (state) {
    //state 0: Idle state until Start button pushed
    case Idle: //Recall Idle = 0 so we begin in our first state here
      if (SerialInput == '8') {
        Start = millis();
        End = 0;
        Serial.print("TASK STARTED AT ");
        Serial.print("\t");
        Serial.println(millis());
        digitalWrite(ShutterBlue, HIGH);
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
          int randChoice = random(0, blue_patterns);
          PulseFreq = blue_pwm[randChoice];
          StimDuration = blue_duration[randChoice];
          BluePatternCount[randChoice] += 1;
          
          // Deliver stim
          giveOpto();
          Serial.print("Blue stim #");
          Serial.print(BluePatternCount[randChoice]);
          Serial.print(": ");
          Serial.print(PulseFreq);
          Serial.print("Hz for ");
          Serial.print(StimDuration);
          Serial.println("s");
        } else {
          // Red laser
          LaserColor = ShutterRed;
          int randChoice = random(0, red_patterns);
          PulseFreq = red_pwm[randChoice];
          StimDuration = red_duration[randChoice];
          RedPatternCount[randChoice] += 1;
          
          // Deliver stim
          giveOpto();
          Serial.print("Red stim #");
          Serial.print(RedPatternCount[randChoice]);
          Serial.print(": ");
          Serial.print(PulseFreq);
          Serial.print("Hz for ");
          Serial.print(StimDuration);
          Serial.println("s");
        }
        
        state = 1;
      }
      
      break;

    // state 3: interval between each sweep pattern
//    case SweepInterval:
//      if (PatternNum == CurrentPattern && CurrentRepeat == RepeatPerPattern){
//        Serial.println("Finished: all opto sweeps");
//        state = 1;
//      }
//      state = 1;
//      break;
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
    }
  }

  if (Serial.available() > 0) {
    // read the incoming byte:
    SerialInput = Serial.read();

    //Serial.print("I received: ");
    //Serial.println(SerialInput);

    // REMOVED UNUSED SERIAL INPUT HANDLERS

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

    // if (SerialInput == '5') { //red pulsing pattern
    //   Serial.println("Entered 5: red pulsing");
    //   for (int i = 0; i < 20; i++) { //For loop runs 20 times
    //     Serial.print("Red stim #");
    //     Serial.println(i + 1); //Print red laser stimulus number
    //     for (int j = 0; j < 5; j++) { //For loop runs 5 times
    //       digitalWrite(ShutterRed, HIGH); //Red shutter opens
    //       delay(PulseDurationRed); //Shutter remains open for a delay of 2ms
    //       //delay(4);
    //       digitalWrite(ShutterRed, LOW); //Red shutter closes
    //       delay(50 - PulseDurationRed); //48ms delay
    //       //delay(46);
    //     }
    //     delay(9500);
    //     //delay(19000); // for j == 20, 20Hz
    //     //delay(18000); // for j == 40, 20Hz
    //     //delay(18000); // for j == 40, 20Hz
    //   }
    //   Serial.println("5: red pulsing finished");
    // }

    // if (SerialInput == '6') { //blue pulsing pattern
    //   Serial.println("Entered 6: blue pulsing");
    //   for (int i = 0; i < 50; i++) {
    //     digitalWrite(ShutterBlue, HIGH);
    //     delay(PulseDurationBlue);
    //     digitalWrite(ShutterBlue, LOW);
    //     delay(ITIblue - PulseDurationBlue);
    //   }
    //   Serial.println("6: blue pulsing finished");
    // }

  }

  // REMOVED UNUSED OUTCOME HANDLING CODE
}

//********************************************************************************************//
void sync() { // Void function governing sync pulses?
  if (millis() - TimerSync >= SyncPulseInterval) {
    if (SyncNow == 1) {
      TimerSync = millis();
      digitalWrite(Sync, LOW);
      SyncNow = 0;
      SyncPulseInterval = 100 + random(1, 400); // random sync pulse interval between 1~2s
    } else { 
      TimerSync = millis();
      digitalWrite(Sync, HIGH);
      SyncNow = 1;
      SyncPulseInterval = 50;
    }
  }
}

//********************************************************************************************//
// Assigned upcoming opto delivery
void giveOpto(){
  PulseNum = StimDuration * PulseFreq; // StimDuration is now in seconds
  PulseInterval = (1000.0/PulseFreq) - PulseDuration;

  if (PulseInterval <= 0 && PulseNum > 1){
    PulseInterval = 5;
    Serial.println("Negative PulseInterval: reset PulseInterval to 5ms");
  }
}

//********************************************************************************************//
// Execute opto delivery 
void opto(){
  if (PulseNum > 0 && millis() - TimerPulse >= OptoInterval){
    OptoDelivering = true;
    if (OptoNow == 1){
      TimerPulse = millis();
      digitalWrite(LaserColor, LOW);
      OptoNow = 0;
      PulseNum -= 1;
      OptoInterval = PulseInterval;
    } else {
      TimerPulse = millis();
      digitalWrite(LaserColor, HIGH);
      OptoNow = 1;
      OptoInterval = PulseDuration;
    }
  }
  OptoDelivering = false;
}

// REMOVED UNUSED RANDOM SHUTTER SOUND FUNCTION

//********************************************************************************************//
// REMOVED UNUSED LICK DETECTION FUNCTION

//********************************************************************************************//
// REMOVED UNUSED PRINT TRIALS FUNCTION