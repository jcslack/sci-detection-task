
#include "actuator.h"
//contant and variable intializations
const long baudrate = 9600;
const int forwardPin = 11;
const int backwardPin = 12;
const int upwardPin = 9;
const int downwardPin = 10;
const int solL = 8; //left reward solenoid
const int solR = 7; // right reward solenoid valve
const int capSenL = 6; //left capacitive sensor
const int capSenR = 5; //right capacitive sensor
const int toneS = 4;
actuator reward_actuator(forwardPin,backwardPin,upwardPin,downwardPin);

int senStateL = 0;  //initialize sensor states (high or low)
int senStateR = 0;
unsigned long startSession; //initialize zero point for session
unsigned long startWait = 0.0;  //initialize zero point for trial
unsigned long responseT;
volatile unsigned long runTime =  3600000; //length of session (msec)
volatile unsigned long responseTime = 10000; //unforced trial response time (msec)
unsigned int unresponsive = 0; //initialize check for non-response trial

bool broken = false; //tell if loop was broken

volatile int maxConsecIncorrectResponses = 3; //consecutive error
volatile bool allowForced = true; //setting for allowing forced and repeated trials (1 for forced and repeat, 0 for no forced or repeat)
volatile bool isAlternating = false; //setting for alternating port session (initial training)

//event counters
int nNonResponse = 0; //number of non-responses
int nRightPortResponse = 0; //number of right port responses
int nLeftPortResponse = 0; //number of left port responses
int nResponses = 0; //total responses (L+R)
int nTrials = 0; //total trials (L+R+u)
int nCorrectResponses = 0; //correct responses
int nIncorrectResponses = 0; //incorrect responses
int nConsecIncorrectResponses  = 0; //consequtive incorrect responses
int nForcedTrials = 0; //forced trial 

int n = 1; //ntrials from 1
float runningPercentCorrect = 0.0; // running percentage correct

const int nTypes = 2; // number of trial types
enum PORT {LEFT = 1, RIGHT = 2, UNRESPONSIVE = 5};
int trialType[nTypes] = {LEFT,RIGHT}; //left port (1) or right port (2) trial

const int nT = sizeof(trialType)/sizeof(trialType[0]);
int entryT = 0; //trialType index
int setTrial; //variable for chosen trial from trialType[entryT]

// comment

//------------------------------MAIN------------------------------
void setup() {
  Serial.begin(baudrate);
  randomSeed(analogRead(0));
  delay(500);
  Serial.flush();
  Serial.println("Connected");
  setPINS();
  while(Serial.available() == 0){

  }
  String ev = Serial.readString();
  if (ev == "Wait"){      
      manualControl();
  }
  Serial.flush();
  startSession = millis();
  Serial.print("Start,");
  Serial.println(startSession);
  delay(100);
    
  serial_flush_buffer();
  if(isAlternating){  
    setTrial = random(1,3); 
  }
  setTrial = trialType[entryT];
  //for(int i = 0;i<=3;i++){
  //  Serial.println(trialType[i]);
  //  delay(100);
  //}
  //Serial.println(entryT);
  //Serial.println(setTrial);
}

void loop() {
  
//-------------------------TRIAL BEGINS---------------------------
  Serial.flush();
  Serial.print("Trial,");
  Serial.print(millis()-startSession);
  Serial.print(",");
  Serial.println(n);
  delay(100);
  Serial.print("Type,");
  Serial.print(setTrial);
  Serial.print(",");
  if (nConsecIncorrectResponses == maxConsecIncorrectResponses && allowForced){
    Serial.println("1");
  }
  else{
    Serial.println("0");
  }
  delay(500);


  stim(setTrial);
 
  
//------------------------WAIT FOR SENSOR INPuT--------------
  startWait = millis();

//--------------------------FORCED TRIAL---------------------
  if(nConsecIncorrectResponses == maxConsecIncorrectResponses && allowForced){
    nForcedTrials++;
    if(setTrial == LEFT){


      while(1){
        if(digitalRead(capSenL) == LOW){
          senStateL = HIGH;
          break;
        }
        else if(millis() - startSession > runTime){
          broken = true;
          break;
        }
        if(Serial.available() > 0){
          manualControl();
        }
          
      }
      
      responseT = millis();
      if(broken){
        nResponses = nLeftPortResponse + nRightPortResponse;
        nTrials = nLeftPortResponse + nRightPortResponse + nNonResponse;
        runningPercentCorrect = nCorrectResponses/(nResponses-nForcedTrials);

 //       digitalWrite(light,HIGH);
        endSession();
      }
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(setTrial);
      Serial.print(",");
      Serial.println("5");
      deliverReward(solL);
      nLeftPortResponse++;
      nConsecIncorrectResponses = 0;
      delay(6000);

    }
    else if(setTrial == RIGHT){
     //  do{
     //   loading(2,0);
     //   pauseSessionCheck();
        
      while(1){
        if(digitalRead(capSenR) == LOW){
          senStateL = HIGH;
          break;
        }
        else if(millis() - startSession > runTime){
          broken = true;
          break;
        }
        if(Serial.available() > 0){
          manualControl();
        }
          
      }
      responseT = millis();
      if(broken){
        nResponses = nLeftPortResponse + nRightPortResponse;
        nTrials = nLeftPortResponse + nRightPortResponse + nNonResponse;
        runningPercentCorrect = nCorrectResponses/(nResponses-nForcedTrials);

   //     digitalWrite(light,HIGH);
        endSession();
      }
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(setTrial);
      Serial.print(",");
      Serial.println("5");
      deliverReward(solR);
      nRightPortResponse++;
      nConsecIncorrectResponses = 0;
      delay(500);
      delay(5500);
    }
  }
  
//--------------------------UNFORCED TRIAL------------------------
  else{
    senStateL = LOW;
    senStateR = LOW;
    while(digitalRead(capSenL) == LOW && digitalRead(capSenR) == LOW && millis()-startWait <= responseTime){
      
    }
    while(1){
      if(digitalRead(capSenL) == HIGH){
        senStateL = HIGH;
        break;
      }
      else if(digitalRead(capSenR) == HIGH){
        senStateR = HIGH;
        break;
      }
      else if(millis() - startWait <= responseTime){
        senStateR = LOW;
        senStateR = LOW;
        break;
      }
    }
    responseT = millis();
    
    if(senStateL == LOW){
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(LEFT);
      if(setTrial == LEFT){
        deliverReward(solL);
        nLeftPortResponse++;
        nCorrectResponses++;
        Serial.print(",");
        Serial.println("1");
        nConsecIncorrectResponses = 0;
        delay(4000);
      }
      else{
        longTone();
        nLeftPortResponse++;
        nIncorrectResponses++;
        if(allowForced){
          nConsecIncorrectResponses++;
        }
        Serial.print(",");
        Serial.println("0");
      }
    }
    else if(senStateR == LOW){
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(RIGHT);

      if(setTrial == LEFT){
        longTone();
        nRightPortResponse++;
        nIncorrectResponses++;
        if(allowForced){
          nConsecIncorrectResponses++;
        }
        Serial.print(",");
        Serial.println("0");
      }
      else{
        deliverReward(solR);
        nRightPortResponse++;
        nCorrectResponses++;
        Serial.print(",");
        Serial.println("1");
        nConsecIncorrectResponses = 0;
        delay(4000);
      }
    }
  else{
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(UNRESPONSIVE);
      Serial.print(",");
      Serial.println("5");
      longTone();
      unresponsive = 0;
      nNonResponse++;
      if(allowForced){
        nConsecIncorrectResponses++;
      }
    }
    delay(1000);

  }


// next trial chosen  

  if(allowForced){ //forced trial on
    if(nConsecIncorrectResponses == 0){     
      entryT++;
      if(entryT > nTypes){
        if(!isAlternating){ //alternating ports off, shuffle array, else start at beginning
          shuffleArray(trialType,nT);
        }
        entryT = 0;
      }
    }
    setTrial = trialType[entryT];
  }
  else{ //forced trial off
    entryT++;
    if(entryT > nTypes){
      if(!isAlternating){
        shuffleArray(trialType,nT);
      }
      entryT = 0;
    }
    setTrial = trialType[entryT];
  }  
  
//adds trial counters
  nResponses = nLeftPortResponse + nRightPortResponse;
  nTrials = nLeftPortResponse + nRightPortResponse + nNonResponse;
  if(nResponses-nForcedTrials == 0){
    runningPercentCorrect = 0;
  }
  else{
    runningPercentCorrect =(float)nCorrectResponses/((float)nResponses-(float)nForcedTrials); 
  }
/*  if(runningPercentCorrect > 0.65 && runningPercentCorrect <= 0.80){
    maxConsecIncorrectResponses = 2;
  }
  if(runningPercentCorrect > 0.80){
    maxConsecIncorrectResponses = 3; 
  }*/
  n++;

  delay(500);
  //digitalWrite(light,HIGH);                 //houselight off
  if(millis()-startSession > runTime){
    endSession();
  }
  //pauseCheck();
  if(Serial.available() > 0){
    manualControl();
  }

  delay(1000);
}



//-----------------functions-----------------------------
//intializes pin modes and states
int flushWater(int s){
  int i = 0;
  pinMode(s,OUTPUT);
  digitalWrite(s,HIGH);
  while(i < 1){
  digitalWrite(s,LOW);
  delay(2000);
  digitalWrite(s,HIGH);
  delay(500);
  i++;
  }
  return(i);
}

void shuffleArray(int * array, int arrSize)
{
  randomSeed(analogRead(0));
  int last = 0;
  int temp = array[last];
  for (int i=0; i<arrSize; i++)
  {
    int index = random(arrSize);
    array[last] = array[index];
    last = index;
  }
  array[last] = temp;
}

void setPINS(){

  pinMode(solL, OUTPUT);
  pinMode(solR,OUTPUT);
  pinMode(toneS,OUTPUT); 
  pinMode(capSenL, INPUT);
  pinMode(capSenR, INPUT);  
  digitalWrite(toneS,HIGH); 
  digitalWrite(solL, HIGH);
  digitalWrite(solR,HIGH);
 
}

void stim(int stimType){
  Serial.print("Stim,");
  Serial.println(stimType);
  //write stuff here for vibration motor
}

//deliver reward
void deliverReward(int solenoid){
  if (solenoid == solL){
    digitalWrite(solenoid,LOW);
    delay(30);
    digitalWrite(solenoid,HIGH);
  }
  else{
    digitalWrite(solenoid,LOW);
    delay(40);
    digitalWrite(solenoid,HIGH);
  }
  shortTone();
}

//short tone
void shortTone(){
  digitalWrite(toneS,LOW);
  delay(100);
  digitalWrite(toneS,HIGH);
}

//long tone
void longTone(){
  digitalWrite(toneS,LOW);
  delay(1000);
  digitalWrite(toneS,HIGH);
}

void endTone(){
  for (int p = 1; p < 4; p++){
    digitalWrite(toneS,LOW);
    delay(100);
    digitalWrite(toneS,HIGH);
    delay(500);
  }
}

//when called (i.e. when session time reaches runTime), 
//initiates empty endless while loop  
void endSession(){
  unsigned long msec = millis() - startSession;
  unsigned long sec = msec / 1000;
  unsigned long mins = sec / 60;
  endTone();
  Serial.print("End,");
  Serial.println(msec);
  while(1){

  }
}

void serial_flush_buffer(){
  while(Serial.available()){
    Serial.read(); 
  }
}

void manualControl(){
  int ch = 0;
  volatile unsigned long pt0;
  Serial.println("Wait");
      while(ch == 0){
        if(Serial.available() > 0){
          int fbyte = Serial.read();
          switch(fbyte){
            case 'M':  //display for testing stimulation
              break;
              
            case 'G':  //display for manual control
              break;
              
           /* case 'O': //closes both doors
              digitalWrite(doorL,LOW);
              digitalWrite(doorR,LOW);
              break;*/
              
            case 'L':  //manual left port flush
              //digitalWrite(doorL,LOW);
              //delay(100);
              digitalWrite(solL,HIGH);
              digitalWrite(solL,LOW);
              break;
            
            case 'R':  //manual right port flush
              //digitalWrite(doorR,LOW);
              //delay(100);           
              digitalWrite(solR,HIGH);
              digitalWrite(solR,LOW);
              break;

            case 'S':  //
              digitalWrite(solR,HIGH);
              digitalWrite(solL,HIGH);
              break; 
              
            /*case 'C':
              digitalWrite(doorL,LOW);
              break;
              
            case 'D':
              digitalWrite(doorR,LOW);
              break;
              
            case 'A':
              digitalWrite(doorL,HIGH);
              break;
              
            case 'E':
              digitalWrite(doorR,HIGH);
              break;*/
            
           // case 'H':
           //   digitalWrite(light,LOW);
           //   break;
              
           // case 'I':
           //   digitalWrite(light,HIGH);
           //   break;
              
           /* case 'J':
              while(1){
                Serial.flush();
                digitalWrite(doorR,LOW);
                digitalWrite(doorL,LOW);
                digitalWrite(solR,HIGH);
                digitalWrite(solL,HIGH);
                while(digitalRead(senL) == HIGH && digitalRead(senR) == HIGH){
                     if(Serial.available()){
                      fbyte = Serial.read();
                      break;
                     }
                }
                if(fbyte == 'K'){
                  break;
                }
                senStateL = digitalRead(senL);
                senStateR = digitalRead(senR);
                if(senStateL == LOW){
                  deliverReward(solL);
                }
                else if (senStateR == LOW){
                  deliverReward(solR);
                }
                delay(1000);
              }
              digitalWrite(doorL,HIGH);
              digitalWrite(doorR,HIGH);
              break;*/

            case 'a':
              isAlternating = true;
              break;
                         
            case 'e':
              maxConsecIncorrectResponses = Serial.parseInt(SKIP_WHITESPACE);
              break;

            case 's':
              runTime = Serial.parseInt(SKIP_WHITESPACE);
              runTime = runTime*60000;
              break;

            case 'f': 
              allowForced = Serial.parseInt(SKIP_WHITESPACE);                 
              break;

            case 'r':
              responseTime = Serial.parseInt(SKIP_WHITESPACE);
              responseTime = responseTime*1000;
              break;
              
           /* case 'F':
              digitalWrite(doorL,HIGH);
              digitalWrite(doorR,HIGH);
              digitalWrite(solR,HIGH);
              digitalWrite(solL,HIGH);
              digitalWrite(light,HIGH);
              ch = 1;
              break;*/

            case 'P':
              pt0 = millis();
              break;
          
            case 'U':
              ch = 1;
              runTime = runTime + (millis() - pt0);
              Serial.print("Time,");
              Serial.println(runTime);
              serial_flush_buffer();
              break;

            case 'Q': 
              endSession();
              break;
            }
          }
        }
}
