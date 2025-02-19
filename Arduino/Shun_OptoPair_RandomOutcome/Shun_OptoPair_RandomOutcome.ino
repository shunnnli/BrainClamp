// Shun Li, 2022/11/10
// Day 1 of opto pair: give opto stim randomly, give water reward randomly

//20230210: tidied up code, renamed to OptoPair_RandomOutocme

//20240502
// 1. Add option to choose between using blue or red to stimulate

#define Idle 0
#define ITI_State 1
#define OutcomeDelivery 2
#define EndOutcomeDelivery 3
//#define SecondCueOn 4
#define SolenoidOn 5
#define SolenoidOff 6
#define TurnOffLickLeft 7
#define RestartClock 8
#define TimeOut 9

#include <math.h>

//********** User settings ***********//
// Set up for the behavior
boolean ENL = false; // whether ITI is ENL
unsigned long UnitRewardSize = 17; // reward size of 1ul
unsigned long SmallRewardSize = 2 * UnitRewardSize;
unsigned long BigRewardSize = 8* UnitRewardSize;
unsigned long SmallPunishSize = 100;
unsigned long BigPunishSize = 200;

// Outcome probability params
int RewardProbRange[2] = {1,30}; //{0, 50};
int PunishProbRange[2] = {31,50}; //{51, 70};
int ToneProbRange[2] = {51,60}; //{71, 80};
int StimProbRange[2] = {61,100}; //{81, 100};
int PunishStimProbRange[2] = {101, 110};

// Opto stim params
boolean RedStim = true; // 1 means using red to stim, 0 means using blue to stim
int StimTotalPulseNum = 25; // number of pulses per pattern
unsigned long StimPulseDuration = 5; // Total duration of each pulse within a stimulation
unsigned long StimPulseFreq = 50; //For single 500ms pulse
unsigned long StimTotalDuration = 500; // Total duration of each stimulation
unsigned long StimPulseInterval = 0;

// Red opto stim params
// if RedStim if true, these will be rewritten to match opto stim params
int RedTotalPulseNum = 25; // number of pulses per pattern
unsigned long RedPulseDuration = 5; // Total duration of each pulse within a stimulation
unsigned long RedPulseFreq = 50; //For single 500ms pulse
unsigned long RedStimDuration = 500; // Total duration of each stimulation
unsigned long RedPulseInterval = 0;

// Blue opto stim params (basically a trigger pulse to matlab galvo.m)
// if RedStim if false, these will be rewritten to match opto stim params
int BlueTotalPulseNum = 15; // number of pulses per pattern
unsigned long BluePulseDuration = 10; // Total duration of each pulse within a stimulation
unsigned long BluePulseFreq = 30; //For single 500ms pulse
unsigned long BlueStimDuration = 500; // Total duration of each stimulation
unsigned long BluePulseInterval = 0;

// Time dependent params
unsigned long ShortToneDuration = 500;
unsigned long LongToneDuration = 1000;
unsigned long ITI1 = 2000;
unsigned long ITI2 = 4000;
unsigned long ITIMax = 200000; //200000;
unsigned long ITIMin = 15000; //15000;
unsigned long ITIGracePeriod = 1000;
unsigned long ITI = 0;

// Optotag pattern parameters
unsigned long ITIlaser = 25; // Time of stim in (ms)after cue
unsigned long ITIblue = 1000; // ITI of blue laser in ms
unsigned long PulseDurationBlue = 20; // duration of blue laser in ms
unsigned long PulseDurationRed = 2;
unsigned long ITIred = 10000; // duration of red laser in ms

//********** Params Initializtion ***********//
// Input output pin description //
const byte Sync = 2; // non-periodic sync pulse
const byte Speaker = 47; //speaker output pin
const byte SpeakerLeft_copy = 48; //speaker output step copy - Left tone
const byte SpeakerRight_copy = 49; //speaker output step copy - Right tone
const byte LickDetectLeft = 18; //left lick detection
const byte LickDetectRight = 19; //right lick detection
const byte WaterSpout = 8; //left spout solenoid
const byte WaterSpout2 = 9; //right spout solenoid
const byte WaterSpout_copy = 6; //copy left spout solenoid for data recording device
const byte WaterSpout2_copy = 7; //copy right spout solenoid for data recording device
const byte Airpuff = 32; //airpuff valve
const byte Airpuff_copy = 34; //airpuff valve copy for data receiving device
const byte ShutterBlue = 22; //1=blue shutter open, 0=closed
const byte ShutterRed = 24; //1=red shutter open, 0=closed

// Misc
char SerialInput = '0'; //for incoming serial data
unsigned long ToneDuration = 0;
int trialRandomProb = 0;

// Constantly occuring stuffs
static int state = 0 ; // MAIN behavior state variable for running behavior task
unsigned long TimerSync = 0; //timer for non-periodic sync pulse
int SyncPulseInterval = 1000; //interval for non-periodic sync pulse
int SyncNow = 0; //current sync signal status
//randomReward()
unsigned long TimerReward = 0;
unsigned long RewardInterval = 0;
int RewardNow = 0;
//randomPunish()
unsigned long TimerPunish = 0;
unsigned long PunishInterval = 0;
int PunishNow = 0;
//randomOpto()
unsigned long TimerOpto = 0;
unsigned long OptoInterval = 0;
int OptoNow = 0;
//randomShutterSound()
unsigned long TimerShutterSound = 0;
unsigned long ShutterSoundInterval = 0;
int ShutterSoundNow = 0;
//giveOpto()
// Red Opto stim parameters
int RedPulseNum = 0;
unsigned long RedTimerPulse = 0;
int RedOptoNow = 0;
unsigned long RedOptoInterval = 0;
// Blue Opto stim parameters
int BluePulseNum = 0;
unsigned long BlueTimerPulse = 0;
int BlueOptoNow = 0;
unsigned long BlueOptoInterval = 0;

// Trial structure related
int PositiveNum = 0; //current positive outcome number
int OptoNum = 0; //current negative outcome number
int NegativeNum = 0;
int ManualPositiveNum = 0;
int ManualNegativeNum = 0;
int toneNum = 0;
int pairNum = 0;

// boolean for printTrials()
int getReward = 0; //0: no reward; 1: small reward; 2: large reward
int getPunish = 0; //0: no punish; 1: small punish; 2: large punish
int getFreeReward = 0; //0: no free reward; 1: small free reward; 2: large free reward
int trialReward = getReward;
int trialPunish = getPunish;
int trialFreeReward = getFreeReward;

// Detection related
int Lick = 0;
unsigned long LastLick = 0; //timestamp of last lick
unsigned long Lick_Duration; //duration of current lick in ms

// Outcome related
unsigned long OutcomeSize = 20; // current trial's port reward size (updated by OutcomeSizeLeft or OutcomeSizeRight)
int LeftOutcomeButton = 0; //left reward button status
int RightOutcomeButton = 0; //right reward button status
unsigned long LeftOutcomeTimer = 0; //timer for left reward button
unsigned long RightOutcomeTimer = 0; //timer for right reward button

// Timestamp related
unsigned long Lick_Start = 0; //timestamp for current lick
unsigned long ITI_start = 0; //timestamp for beginning of ITI
unsigned long Current_ITI = 0; // current ITI (reset by licks)
unsigned long Actual_ITI = 0; // actual elapsed time from last trial (not reset by licks)
unsigned long ITI_firstStart = 0;
unsigned long trialITIMin = 0;
unsigned long trialITIMax = 0;
unsigned long Cue_start = 0; //timestamp for cue onset
unsigned long Cue_off = 0; //timestamp for cue off
unsigned long Reward_start = 0; //timestamp for start of reward
unsigned long Punish_start = 0; //timestamp for start of punishment
unsigned long Opto_start = 0;
unsigned long Outcome_off = 0; //timestamp for solenoid off
unsigned long Timeout_start = 0; //timestamp for timeout
unsigned long On; //timestamp for reward delivery (solenoid on)
unsigned long Start = 0; //timestamp for starting the session (used for triggering camera)
unsigned long Now = 0; //current timesatmp (used for triggering camera)
unsigned long End = 0; //timestamp for ending the session

void setup()
{
  Serial.begin(115200);

  pinMode(Sync, OUTPUT);
  pinMode(WaterSpout, OUTPUT);
  pinMode(WaterSpout2, OUTPUT);
  pinMode(Airpuff, OUTPUT);
  pinMode(WaterSpout_copy, OUTPUT);
  pinMode(WaterSpout2_copy, OUTPUT);
  pinMode(Airpuff_copy, OUTPUT);
  pinMode(LickDetectLeft, INPUT);
  pinMode(LickDetectRight, INPUT);
  pinMode(Speaker, OUTPUT);
  pinMode(SpeakerLeft_copy, OUTPUT);
  pinMode(SpeakerRight_copy, OUTPUT);
  pinMode(ShutterBlue, OUTPUT);
  pinMode(ShutterRed, OUTPUT);

  // Initialize sync params
  Start = millis();
  TimerSync = millis();
  state = 0;
  SyncNow = 0;
  // Initialize opto params
  RedTimerPulse = 0;
  RedOptoNow = 0;
  BlueTimerPulse = 0;
  BlueOptoNow = 0;
  // Initialize random reward params
  TimerReward = 0;
  RewardNow = 1;
  // Initialize random punish params
  TimerPunish = 0;
  PunishNow = 1;
  // Initialize random stim params
  TimerOpto = 0;
  OptoNow = 1;

  digitalWrite(Sync, LOW);
  digitalWrite(WaterSpout, LOW);
  digitalWrite(WaterSpout2, LOW);
  digitalWrite(Airpuff, LOW);
  digitalWrite(WaterSpout_copy, LOW);
  digitalWrite(WaterSpout2_copy, LOW);
  digitalWrite(Airpuff_copy, LOW);
  noTone(Speaker);
  digitalWrite(ShutterBlue, HIGH);
  digitalWrite(ShutterRed, HIGH);
  randomSeed(analogRead(3));

  Serial.println("---------------------------------RandomOutcome--------------------------------");
  Serial.println("Manual check: 1 -> reward; 2 -> punishment; 3 -> blue; 4 -> red");
  Serial.println("Laser shutter: 5 -> blue stim; 6 -> red stim");
  Serial.println("Water calibration: 7");
  Serial.println("Trial start/stop: 8 -> start; 9 -> end");
  Serial.println("---------------------------------RandomOutcome--------------------------------");

  if (RedStim){
    // Red opto stim params
    RedTotalPulseNum = StimTotalPulseNum; // number of pulses per pattern
    RedPulseDuration = StimPulseDuration; // Total duration of each pulse within a stimulation
    RedPulseFreq = StimPulseFreq; //For single 500ms pulse
    RedStimDuration = StimTotalDuration; // Total duration of each stimulation
    RedPulseInterval = StimPulseInterval;

    // Print checks
    Serial.println("Stim color: red");
    Serial.print("Stim total pulse num: ");
    Serial.println(RedTotalPulseNum);
    Serial.print("Stim pulse duration: ");
    Serial.println(RedPulseDuration);
    Serial.print("Stim pulse freq: ");
    Serial.println(RedPulseFreq);
    Serial.print("Stim total duration: ");
    Serial.println(RedStimDuration);
    
  }else{
    // Blue opto stim params
    BlueTotalPulseNum = StimTotalPulseNum; // number of pulses per pattern
    BluePulseDuration = StimPulseDuration; // Total duration of each pulse within a stimulation
    BluePulseFreq = StimPulseFreq; //For single 500ms pulse
    BlueStimDuration = StimTotalDuration; // Total duration of each stimulation
    BluePulseInterval = StimPulseInterval;

    // Print checks
    Serial.println("Stim color: blue");
    Serial.print("Stim total pulse num: ");
    Serial.println(BlueTotalPulseNum);
    Serial.print("Stim pulse duration: ");
    Serial.println(BluePulseDuration);
    Serial.print("Stim pulse freq: ");
    Serial.println(BluePulseFreq);
    Serial.print("Stim total duration: ");
    Serial.println(BlueStimDuration);
  }
}


void loop() {
  sync(); //Non period sync pulse (1s width) generation
  lickDetection();
  opto();
  //randomShutterSound(ShutterBlue);

  switch (state) {
    //state 0: Idle state until Start button pushed
    case Idle:
      if (SerialInput == '8') {
        Start = millis();
        End = 0;
        Serial.print("TASK STARTED AT ");
        Serial.print("\t");
        Serial.println(millis());
        state = 1;
        digitalWrite(WaterSpout, LOW);
        digitalWrite(WaterSpout2, LOW);
        digitalWrite(Airpuff, LOW);
        digitalWrite(WaterSpout_copy, LOW);
        digitalWrite(WaterSpout2_copy, LOW);
        digitalWrite(Airpuff_copy, LOW);
        digitalWrite(ShutterBlue, HIGH);
        digitalWrite(ShutterRed, HIGH);
        noTone(Speaker);
      }
      break;

    //state 1: Determine the intertrial interval
    case ITI_State:
      ITI_start = millis();
      ITI_firstStart = millis();
      ITI = random(ITI1, ITI2);
      state = 2;
      Serial.print("ITI: ");
      Serial.println(ITI / 1000.0);
      break;


    //state 2: select what to deliver
    case OutcomeDelivery:
      Cue_start = millis();
      Actual_ITI = Cue_start - ITI_firstStart;
      Current_ITI = Cue_start - ITI_start;
      trialITIMin = random(ITIMin - ITIGracePeriod, ITIMin + ITIGracePeriod);
      trialITIMax = random(ITIMax - ITIGracePeriod, ITIMax + ITIGracePeriod);
      
      if (Current_ITI > ITI && Actual_ITI > trialITIMin) {

        trialRandomProb = random(101);
        //Serial.println(RewardProbRange[1]);
        if (trialRandomProb >= RewardProbRange[0] && trialRandomProb <= RewardProbRange[1]) {
          Reward_start = millis();
          digitalWrite(WaterSpout2, HIGH);
          digitalWrite(WaterSpout2_copy, HIGH);
          PositiveNum += 1;
          Punish_start = 0;
          getReward = 2;

          Serial.print("Reward: ");
          Serial.print(PositiveNum);
          Serial.print("\t");
          Serial.print("Time: ");
          Serial.println(millis() / 1000.0);

        } else if (trialRandomProb >= PunishProbRange[0] && trialRandomProb <= PunishProbRange[1]) {
          Punish_start = millis();
          digitalWrite(Airpuff, HIGH);
          digitalWrite(Airpuff_copy, HIGH);
          NegativeNum += 1;
          Reward_start = 0;
          getPunish = 2;

          Serial.print("Punish: ");
          Serial.print(NegativeNum);
          Serial.print("\t");
          Serial.print("Time: ");
          Serial.println(millis() / 1000.0);
        
        } else if (trialRandomProb >= ToneProbRange[0] && trialRandomProb <= ToneProbRange[1]){
          tone(Speaker, 3000);
          digitalWrite(SpeakerLeft_copy, HIGH);
          delay(ShortToneDuration);
          noTone(Speaker);
          digitalWrite(SpeakerLeft_copy, LOW);
          toneNum += 1;
          
          Serial.print("Tone: ");
          Serial.print(toneNum);
          Serial.print("\t");
          Serial.print("Time: ");
          Serial.println(millis() / 1000.0);
        
        } else if (trialRandomProb >= StimProbRange[0] && trialRandomProb <= StimProbRange[1]){
          Opto_start = millis();
          if (RedStim){giveRedOpto();}
          else{giveBlueOpto();}
          //giveRedOpto();
          //giveBlueOpto();
          OptoNum += 1;
          Serial.print("Optostim: ");
          Serial.print(OptoNum);
          Serial.print("\t");
          Serial.print("Time: ");
          Serial.println(millis() / 1000.0);

        } else if (trialRandomProb >= PunishStimProbRange[0] && trialRandomProb <= PunishStimProbRange[1]){
          Opto_start = millis();
          Punish_start = millis();
          
          if (RedStim){giveRedOpto();}
          else{giveBlueOpto();}
          
          digitalWrite(Airpuff, HIGH);
          digitalWrite(Airpuff_copy, HIGH);
          Reward_start = 0;
          getPunish = 2;
          pairNum += 1;
          
          Serial.print("Pair airpuff & stim: ");
          Serial.print(pairNum);
          Serial.print("\t");
          Serial.print("Time: ");
          Serial.println(millis() / 1000.0);
        }
        state = 3;
      }

      if (Lick == 1 && ENL) {
        ITI_start = millis();
      }

      // Record reward/punishment delivery for current trial
      trialReward = getReward;
      trialPunish = getPunish;
      trialFreeReward = getFreeReward;
      break;

  //state 3: end reward delivery
case EndOutcomeDelivery:
  Outcome_off = millis();
  if (getReward == 1) {
    if ((Outcome_off - Reward_start) >= SmallRewardSize) {
      digitalWrite(WaterSpout, LOW);
      digitalWrite(WaterSpout_copy, LOW);
      digitalWrite(WaterSpout2, LOW);
      digitalWrite(WaterSpout2_copy, LOW);
      getReward = 0;
    }
  } else if (getReward == 2) {
    if ((Outcome_off - Reward_start) >= BigRewardSize) {
      digitalWrite(WaterSpout, LOW);
      digitalWrite(WaterSpout_copy, LOW);
      digitalWrite(WaterSpout2, LOW);
      digitalWrite(WaterSpout2_copy, LOW);
      getReward = 0;
      Serial.println("dfsdfaf");
    }
  }

  if (getPunish == 1) {
    if ((Outcome_off - Punish_start) >= SmallPunishSize) {
      digitalWrite(Airpuff, LOW);
      digitalWrite(Airpuff_copy, LOW);
      getPunish = 0;
    }
  } else if (getPunish == 2) {
    if ((Outcome_off - Punish_start) >= BigPunishSize) {
      digitalWrite(Airpuff, LOW);
      digitalWrite(Airpuff_copy, LOW);
      getPunish = 0;
    }
  }

  if (getReward == 0 && getPunish == 0) {
    printTrials();
    state = 1; //go to state 1 if all reward and punish are given
  }
  break;

default:
  state = 1;
  break;
}
// END OF SWITCH STRCUTURE //


// Ending Task //
if (SerialInput == '9') {
  if (End == 0) {
    Serial.print("TASK ENDED AT ");
    Serial.print("\t");
    Serial.println(millis());
    Serial.print("Total reward: ");
    Serial.println(PositiveNum);
    Serial.print("Total punish: ");
    Serial.println(NegativeNum);
    Serial.print("Total opto: ");
    Serial.println(OptoNum);
    state = 0;
    noTone(Speaker);
    End = millis();
  }
}

if (Serial.available() > 0) {
  // read the incoming byte:
  SerialInput = Serial.read();

  //Serial.print("I received: ");
  //Serial.println(SerialInput);

  if (SerialInput == '1' && LeftOutcomeButton == 0) { // dispense left reward
    ManualPositiveNum += 1;
    Serial.print("Entered 1: manual reward #");
    Serial.println(ManualPositiveNum);
    LeftOutcomeButton = 1;
    digitalWrite(WaterSpout2, HIGH);
    digitalWrite(WaterSpout2_copy, HIGH);
    OutcomeSize = BigRewardSize;
    LeftOutcomeTimer = millis();
  }

  if (SerialInput == '2' && RightOutcomeButton == 0) {
    Serial.print("Entered 2: manual punishment #");
    Serial.println(ManualNegativeNum);
    RightOutcomeButton = 1;
    digitalWrite(Airpuff, HIGH);
    digitalWrite(Airpuff_copy, HIGH);
    OutcomeSize = SmallPunishSize;
    RightOutcomeTimer = millis();
  }

  if (SerialInput == '3') {
    Serial.println("Entered 3: Deliver blue stim");
//    giveBlueOpto();
    digitalWrite(ShutterBlue,LOW);
  }

  if (SerialInput == '4') {
    Serial.println("Entered 4: Deliver red stim");
    //giveRedOpto();
    digitalWrite(ShutterRed,LOW);
  }

  if (SerialInput == '5') { //blue pulsing pattern
    Serial.println("Entered 5: deliver blue stim");
    giveBlueOpto();
//
//    // Do 20x 100s stim at 10s interval
//    for (int i = 0; i < 30; i++){
//      delay(10000);
//      digitalWrite(ShutterBlue,LOW);
//      delay(200);
//      digitalWrite(ShutterBlue,HIGH);
//      Serial.print("Delivered blue stim #");
//      Serial.println(i+1);
//    }
//    Serial.println("Finished blue stim");
  }

  if (SerialInput == '6') { //red pulsing pattern
    Serial.println("Entered 6: deliver red stim");
    giveRedOpto();
  }

  if (SerialInput == '7'){
      Serial.println("Deliver 200 reward for calibration");
      int num_repeat = 200;
      for (int i = 0; i < num_repeat; i++) {
        delay(100);
        digitalWrite(WaterSpout2, HIGH);
        digitalWrite(WaterSpout2_copy, HIGH);
        delay(UnitRewardSize);
        digitalWrite(WaterSpout2, LOW);
        digitalWrite(WaterSpout2_copy, LOW);
      }
      Serial.print("Finished delivery x");
      Serial.println(num_repeat);
    }

}

//Left Button outcome
if ((millis() - LeftOutcomeTimer > OutcomeSize) && LeftOutcomeButton == 1) {
  digitalWrite(WaterSpout2, LOW);
  digitalWrite(WaterSpout2_copy, LOW);
  Serial.print("Manual reward");
  Serial.print("\t");
  Serial.print(LeftOutcomeTimer / 1000.0);
  Serial.print("\t");
  Serial.println(OutcomeSize);
  LeftOutcomeButton = 0;
}

//Right button outcome
if ((millis() - RightOutcomeTimer) > OutcomeSize && RightOutcomeButton == 1) {
  digitalWrite(Airpuff, LOW);
  digitalWrite(Airpuff_copy, LOW);
  Serial.print("Manual punishment");
  Serial.print("\t");
  Serial.print(RightOutcomeTimer / 1000.0);
  Serial.print("\t");
  Serial.println(OutcomeSize);
  RightOutcomeButton = 0;
}
}

//********************************************************************************************//
void sync() {
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
// Assigned upcoming red opto delivery
void giveRedOpto(){
  if (RedTotalPulseNum != 1){
    RedPulseNum = (RedStimDuration/1000.0) * RedPulseFreq;
    RedPulseInterval = (1000.0/RedPulseFreq) - RedPulseDuration;
  } else{
    RedPulseNum = RedTotalPulseNum;
    RedPulseInterval = 5;
  }

  if (RedPulseInterval <= 0 && RedPulseNum > 1){
    RedPulseInterval = 5;
    Serial.println("Negative RedPulseInterval: reset to 5ms");
  }
}

//********************************************************************************************//
// Assigned upcoming blue opto delivery
void giveBlueOpto(){
  if (BlueTotalPulseNum != 1){
    BluePulseNum = (BlueStimDuration/1000.0) * BluePulseFreq;
    BluePulseInterval = (1000.0/BluePulseFreq) - BluePulseDuration;
  } else {
    BluePulseNum = BlueTotalPulseNum;
    BluePulseInterval = 5;
  }

  if (BluePulseInterval <= 0 && BluePulseNum > 1){
    BluePulseInterval = 5;
    Serial.println("Negative BluePulseInterval: reset to 5ms");
  }
}

//********************************************************************************************//
// Execute opto delivery 
void opto(){
  if (RedPulseNum > 0 && millis() - RedTimerPulse >= RedOptoInterval){
    if (RedOptoNow == 1){
      RedTimerPulse = millis();
      digitalWrite(ShutterRed, HIGH);
      RedOptoNow = 0;
      RedPulseNum -= 1;
      RedOptoInterval = RedPulseInterval;
    } else {
      RedTimerPulse = millis();
      digitalWrite(ShutterRed, LOW);
      RedOptoNow = 1;
      RedOptoInterval = RedPulseDuration;
    }
  }

  if (BluePulseNum > 0 && millis() - BlueTimerPulse >= BlueOptoInterval){
    if (BlueOptoNow == 1){
      BlueTimerPulse = millis();
      digitalWrite(ShutterBlue, HIGH);
      BlueOptoNow = 0;
      BluePulseNum -= 1;
      BlueOptoInterval = BluePulseInterval;
    } else {
      BlueTimerPulse = millis();
      digitalWrite(ShutterBlue, LOW);
      BlueOptoNow = 1;
      BlueOptoInterval = BluePulseDuration;
    }
  }
}

//********************************************************************************************//
void randomReward() {
  if (millis() - TimerReward >= RewardInterval) {
    if (RewardNow == 1) {
      TimerReward = millis();
      digitalWrite(WaterSpout2, LOW);
      digitalWrite(WaterSpout2_copy, LOW);
      RewardNow = 0;
      RewardInterval = 5000 + random(10000); // random reward interval between 5 - 15s

      //Avoid other events
      //NextRewardTime = millis() + RewardInterval;
      //if (NextRewardTime + 5 < NextOptoTime){

      //}

      Serial.print("Next reward after ");
      Serial.print(RewardInterval / 1000.0);
      Serial.println("s");
    } else {
      TimerReward = millis();
      digitalWrite(WaterSpout2, HIGH);
      digitalWrite(WaterSpout2_copy, HIGH);
      RewardNow = 1;
      PositiveNum += 1;
      RewardInterval = BigRewardSize;
      printTrials();
    }
  }
}

//********************************************************************************************//
void randomPunish() {
  if (millis() - TimerPunish >= PunishInterval) {
    if (PunishNow == 1) {
      TimerPunish = millis();
      digitalWrite(Airpuff, LOW);
      digitalWrite(Airpuff_copy, LOW);
      PunishNow = 0;
      PunishInterval = 20000 + random(1, 30000); // random reward interval between 20~50s
      Serial.print("Next punishment after ");
      Serial.print(PunishInterval / 1000.0);
      Serial.println("s");
    } else {
      TimerPunish = millis();
      digitalWrite(Airpuff, HIGH);
      digitalWrite(Airpuff_copy, HIGH);
      PunishNow = 1;
      NegativeNum += 1;
      PunishInterval = SmallPunishSize;
      printTrials();
    }
  }
}

//********************************************************************************************//
void randomOpto() {
  if (millis() - TimerOpto >= OptoInterval) {
    if (OptoNow == 1) {
      TimerOpto = millis();
      digitalWrite(ShutterRed, LOW);
      //digitalWrite(ShutterRed_copy, LOW);
      OptoNow = 0;
      OptoInterval = 10000 + random(1, 10000); // random opto interval between 10~20s
      Serial.print("Next opto after ");
      Serial.print(OptoInterval / 1000.0);
      Serial.println("s");
    } else {
      TimerOpto = millis();
      digitalWrite(ShutterRed, HIGH);
      //digitalWrite(ShutterRed_copy, HIGH);
      OptoNow = 1;
      OptoNum += 1;
      OptoInterval = 500;
      printTrials();
    }
  }
}

//********************************************************************************************//
void randomShutterSound(const byte ShutterColor) {
  if (millis() - TimerShutterSound >= ShutterSoundInterval) {
    if (ShutterSoundNow == 1) {
      TimerShutterSound = millis();
      digitalWrite(ShutterColor, LOW);
      ShutterSoundNow = 0;
      ShutterSoundInterval = 100 + random(1, 900); // random opto interval between 0.1~1s
      //Serial.print("Next ShutterSound after ");
      //Serial.print(ShutterSoundInterval / 1000.0);
      //Serial.println("s");
    } else {
      TimerShutterSound = millis();
      digitalWrite(ShutterColor, HIGH);
      ShutterSoundNow = 1;
      ShutterSoundInterval = 500;
    }
  }
}

//********************************************************************************************//
void lickDetection() {
  // Lick Detection //
  if (digitalRead(LickDetectRight) == 0) {
    if (Lick == 0) {
      Lick = 1;
      Lick_Start = millis();
    }
  }
  if (digitalRead(LickDetectRight) == 1) {
    if (Lick == 1) {
      Lick = 0;

      Serial.print("Lick Detected");
      Serial.print("\t");

      Lick_Duration = millis() - Lick_Start;
      Serial.print("Time: ");
      Serial.println(millis() / 1000.0);
      LastLick = millis();
    }
  }
  // Lick Detection END //
}

//********************************************************************************************//
void printTrials() {
  // ITI state
  if (state == 3) {
    if (trialReward == 1) {
      Serial.print("Reward block: small reward");
      Serial.print("\t");
    } else if (trialReward == 2) {
      Serial.print("Reward block: big reward");
      Serial.print("\t");
    } else if (trialPunish == 1) {
      Serial.print("Reward block: small punish");
      Serial.print("\t");
    } else if (trialPunish == 2) {
      Serial.print("Reward block: big punish");
      Serial.print("\t");
    }
    Serial.print("Time: ");
    Serial.println(millis() / 1000.0);
  }
}
