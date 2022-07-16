
#include "actuator.h"
//contant and variable intializations
const long baudrate = 9600;
const int forwardPin = 11;
const int backwardPin = 12;
const int upwardPin = 9;
const int downwardPin = 10;
const int solW = 7; //reward solenoid valve
const int capSen = 6; //capacitive sensor
const int swL = 5; //left switch
const int swR = 4; //right switch
const int sol = 8; //piston solenoid valve
actuator reward_actuator(forwardPin,backwardPin,upwardPin,downwardPin);

int senStateL = 0;  //initialize sensor states (high or low)
int senStateR = 0;
unsigned long startSession; //initialize zero point for session
unsigned long startWait = 0.0;  //initialize zero point for trial
unsigned long responseT;
volatile unsigned long runTime =  3600000; //length of session (msec)
volatile unsigned long responseTime = 10000; //unforced trial response time (msec)
unsigned int unresponsive = 0; //initialize check for non-response trial

unsigned int b = 0; //tell if loop was broken

volatile int maxE = 3; //consecutive error
volatile int fcheck = 1; //setting for forced and repeated trials (1 for forced and repeat, 0 for no forced or repeat)
volatile int acheck = 1; //setting for alternating port session (initial training)
//event counters
int U = 0; //number of non-responses
int R = 0; //number of right port responses
int L = 0; //number of left port responses
int T = 0; //total responses (L+R)
int N = 0; //total trials (L+R+U)
int C = 0; //correct responses
int I = 0; //incorrect responses
int E = 0; //consequtive incorrect responses
int F = 0; //forced trial 
int n = 1;
float P = 0.0;
int A = 0; //trial type for saving data (1,2)
int B = 0; //left (1), right (2) or unresponsive (5) beam break;
int M = 0; //forced (1) or unforced (0) trial


int trialType[2] = {1,2}; //left port (1) or right port (2) trial
const int nT = sizeof(trialType)/sizeof(trialType[0]);
int entryT = 0; //trialType index
int setTrial; //variable for chosen trial from trialType[entryT]

 

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
  if(acheck == 1){  
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
  if (E == maxE && fcheck == 1){
    Serial.println("1");
  }
  else{
    Serial.println("0");
  }
  delay(500);
  
  lcd.setCursor(0,1);
  if(setTrial == 1){                    //display trial type on lcd and open doors
    A = 1;
    if(E == maxE && fcheck == 1){
      digitalWrite(light,LOW);
      stim(A);
      digitalWrite(doorL,LOW);
    }
    else{
     digitalWrite(light,LOW);
     stim(A);
     digitalWrite(doorL,LOW);
     delay(10);
     digitalWrite(doorR,LOW);
    }
  }
  else if(setTrial == 2){
    A = 2;
    if(E == maxE && fcheck == 1){
      digitalWrite(light,LOW);
      stim(A);
      digitalWrite(doorR,LOW);
    }
    else{
      digitalWrite(light,LOW);
      stim(A);
      digitalWrite(doorR,LOW);
      delay(10);
      digitalWrite(doorL,LOW);
    }
  }
  
//------------------------WAIT FOR SENSOR INPUT--------------
  startWait = millis();

//--------------------------FORCED TRIAL---------------------
  if(E == maxE && fcheck == 1){
    F++;
    if(setTrial == 1){
     // do{
     //  lcd.setCursor(0,2);
     //   lcd.print("Wait for Response");
     //   loading(2,0);

      while(digitalRead(senL) == HIGH && b == 0){
          if(Serial.available() > 0){
            manualControl();
          }
        if(millis()-startSession > runTime){
          b = 1;
          break;
        };
      }
      responseT = millis();
      if(b == 1){
        T = L + R;
        N = L + R + U;
        P = C/(T-F);
        digitalWrite(doorL,HIGH);
        digitalWrite(light,HIGH);
        endSession();
      }
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(setTrial);
      Serial.print(",");
      Serial.println("5");
      lcd.setCursor(0,3);
      lcd.print("--> Left Port Poke");
      deliverReward(solL);
      L++;
      B = 1;
      E = 0;
      delay(500);
      lcd.setCursor(4,3);
      lcd.print("Forced Left   ");
      delay(5500);
      digitalWrite(doorL,HIGH);     //close door
    }
    else if(setTrial == 2){
     //  do{
     //   loading(2,0);
     //   pauseSessionCheck();
        
      while(digitalRead(senR) == HIGH && b == 0){
          if(Serial.available() > 0){
            manualControl();
          }
        if(millis()-startSession > runTime){
          b = 1;
          break;
        }
      }
      responseT = millis();
      if(b == 1){
        T = L + R;
        N = L + R + U;
        P = C/(T-F);
        digitalWrite(doorR,HIGH);
        digitalWrite(light,HIGH);
        endSession();
      }
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(setTrial);
      Serial.print(",");
      Serial.println("5");
      lcd.setCursor(0,3);
      lcd.print("--> Right Port Poke");
      deliverReward(solR);
      R++;
      B = 2;
      E = 0;
      delay(500);
      lcd.setCursor(4,3);
      lcd.print("Forced Right   ");
      delay(5500);
      digitalWrite(doorR,HIGH);     //close door  
    }
  }
  
//--------------------------UNFORCED TRIAL------------------------
  else{

    while(digitalRead(senL) == HIGH && digitalRead(senR) == HIGH && millis()-startWait <= responseTime){
      
    }
    responseT = millis();
    senStateL = digitalRead(senL);
    senStateR = digitalRead(senR);
    
    if(senStateL == LOW){
      B = 1;
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(B);
      lcd.setCursor(0,3);
      lcd.print("--> Left Port Poke");
      if(setTrial == 1){
        deliverReward(solL);
        L++;
        C++;
        Serial.print(",");
        Serial.println("1");
        E = 0;
        lcd.setCursor(4,3);
        lcd.print("Correct Port  ");
        delay(4000);
      }
      else{
        longTone();
        L++;
        I++;
        if(fcheck == 1){
          E++;
        }
        Serial.print(",");
        Serial.println("0");
        lcd.setCursor(0,3);
        lcd.print("--> Incorrect Port  ");
      }
    }
    else if(senStateR == LOW){
      B = 2;
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(B);
      lcd.setCursor(0,3);
      lcd.print("--> Right Port Poke");

      if(setTrial == 1){
        longTone();
        R++;
        I++;
        if(fcheck == 1){
          E++;
        }
        Serial.print(",");
        Serial.println("0");
        lcd.setCursor(4,3);
        lcd.print("Incorrect Port ");
      }
      else{
        deliverReward(solR);
        R++;
        C++;
        Serial.print(",");
        Serial.println("1");
        E = 0;
        lcd.setCursor(4,3);
        lcd.print("Correct Port   ");
        delay(4000);
      }
    }
  else{
      B = 5;
      Serial.print("Response,");
      Serial.print(responseT-startWait);
      Serial.print(",");
      Serial.print(B);
      Serial.print(",");
      Serial.println("5");
      lcd.setCursor(0,3);
      lcd.print("--> No Response");
      longTone();
      unresponsive = 0;
      U++;
      if(fcheck == 1){
        E++;
      }
    }
    delay(1000);
    digitalWrite(doorR,HIGH);
    digitalWrite(doorL,HIGH);
  }


// next trial chosen  
  /*if(E == 0 || fcheck == 0){ //consecutive error is zero or forced trial off, continue through array
    entryT++;
    if(entryT > 3){         //reshuffle array if all values have been used
      if(acheck == 1){
        shuffleArray(trialType,nT);
      }
      entryT = 0;
      setTrial = trialType[entryT];
    }
    else{
      setTrial = trialType[entryT];
    }
  }
  else{  //repeat same trial on errors
    setTrial = setTrial;
  }*/
  if(fcheck == 0){ //forced trial off
    entryT++;
    if(entryT > 3){
      if(acheck == 1){
        shuffleArray(trialType,nT);
      }
      //Serial.println("Reshuffle");
      //for(int i = 0; i <= 3;i++){
        //Serial.println(trialType[i]);
      //}
      entryT = 0;
    }
    setTrial = trialType[entryT];
    //Serial.println(entryT);
    //Serial.println(setTrial);
  }
  else if(fcheck == 1){ //forced trial on
    //Serial.print("E = ");
    //Serial.println(E);
    if(E == 0){
      
      entryT++;
      if(entryT > 3){
        if(acheck == 1){ //alternating ports off, shuffle array, else start at beginning
          shuffleArray(trialType,nT);
        }
        //Serial.println("Reshuffle");
        entryT = 0;
      }
    }
    setTrial = trialType[entryT];
  }  
  
//adds trial counters
  T = L + R;
  N = L + R + U;
  if(T-F == 0){
    P = 0;
  }
  else{
    P =(float)C/((float)T-(float)F); 
  }
/*  if(P > 0.65 && P <= 0.80){
    maxE = 2;
  }
  if(P > 0.80){
    maxE = 3; 
  }*/
  n++;

  delay(500);
  digitalWrite(light,HIGH);                 //houselight off
  dispTrialCounters();
  if(millis()-startSession > runTime){
    endSession();
  }
  //pauseCheck();
  if(Serial.available() > 0){
    manualControl();
  }
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("-----Next Trial-----");
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
  pinMode(doorR,OUTPUT);
  pinMode(doorL,OUTPUT);
  pinMode(solL,OUTPUT);
  pinMode(solR,OUTPUT);
  pinMode(senL,INPUT_PULLUP);
  pinMode(senR,INPUT_PULLUP);
  pinMode(toneS,OUTPUT);
  pinMode(light,OUTPUT);
  digitalWrite(doorR,HIGH);
  digitalWrite(doorL,HIGH);
  digitalWrite(solL,HIGH);
  digitalWrite(solR,HIGH);
  digitalWrite(toneS,HIGH);
  digitalWrite(light,HIGH);
  delay(1000);
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
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Stimulation Test");
              break;
              
            case 'G':  //display for manual control
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Manual Control");
              break;
              
            case 'O': //closes both doors
              digitalWrite(doorL,LOW);
              digitalWrite(doorR,LOW);
              lcd.setCursor(0,1);
              lcd.print("Door Closed");
              break;
              
            case 'L':  //manual left port flush
              lcd.setCursor(0,2);
              lcd.print("Left Port Flush ");
              digitalWrite(doorL,LOW);
              delay(100);
              digitalWrite(solL,HIGH);
              digitalWrite(solL,LOW);
              break;
            
            case 'R':  //manual right port flush
              lcd.setCursor(0,2);
              lcd.print("Right Port Flush"); 
              digitalWrite(doorR,LOW);
              delay(100);           
              digitalWrite(solR,HIGH);
              digitalWrite(solR,LOW);
              break;

            case 'S':  //
              lcd.setCursor(0,2);
              lcd.print("Flushed         ");
              digitalWrite(solR,HIGH);
              digitalWrite(solL,HIGH);
              break; 
              
            case 'C':
              digitalWrite(doorL,LOW);
              lcd.setCursor(0,1);
              lcd.print("Left Door Open ");
              break;
              
            case 'D':
              digitalWrite(doorR,LOW);
              lcd.setCursor(0,1);
              lcd.print("Right Door Open");
              break;
              
            case 'A':
              digitalWrite(doorL,HIGH);
              lcd.setCursor(0,1);
              lcd.print("Left Door Closed");
              break;
              
            case 'E':
              digitalWrite(doorR,HIGH);
              lcd.setCursor(0,1);
              lcd.print("Right Door Closed");
              break;
            
            case 'H':
              digitalWrite(light,LOW);
              lcd.setCursor(0,1);
              lcd.print("Houselight On      ");
              break;
              
            case 'I':
              digitalWrite(light,HIGH);
              lcd.setCursor(0,1);
              lcd.print("Houselight Off     ");
              break;
              
            case 'J':
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
              break;

            case 'a':
              acheck = 2;
              lcd.setCursor(0,3);
              lcd.print("Alternating Ports");
              break;
                         
            case 'e':
              maxE = Serial.parseInt(SKIP_WHITESPACE);
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Consecutive Error");
              lcd.setCursor(0,1);
              lcd.print("Changed: ");
              lcd.setCursor(9,1);
              lcd.print(maxE); 
              break;

            case 's':
              runTime = Serial.parseInt(SKIP_WHITESPACE);
              runTime = runTime*60000;
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Session Time (min)");
              lcd.setCursor(0,1);
              lcd.print("Changed: ");
              lcd.setCursor(9,1);
              lcd.print(runTime/1000/60);
              break;

            case 'f': 
              int fc;
              fc = Serial.parseInt(SKIP_WHITESPACE);
              lcd.clear();
              lcd.setCursor(0,0);
              if(fc == 1){
                fcheck = 1;
                lcd.print("Forced Trial On"); 
              }
              else if(fc == 0){
                fcheck = 0;
                lcd.print("Forced Trial Off");
              }                      
              break;

            case 'r':
              responseTime = Serial.parseInt(SKIP_WHITESPACE);
              responseTime = responseTime*1000;
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Response Time (sec)");
              lcd.setCursor(0,1);
              lcd.print("Changed: ");
              lcd.setCursor(9,1);
              lcd.print(responseTime/1000);
              break;
              
            case 'F':
              digitalWrite(doorL,HIGH);
              digitalWrite(doorR,HIGH);
              digitalWrite(solR,HIGH);
              digitalWrite(solL,HIGH);
              digitalWrite(light,HIGH);
              ch = 1;
              break;

            case 'P':
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Session Paused");
              pt0 = millis();
              break;
          
            case 'U':
              lcd.clear();
              lcd.setCursor(0,0);
              lcd.print("Unpaused");
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
