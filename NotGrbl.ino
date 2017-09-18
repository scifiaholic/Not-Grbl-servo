// Simple 3 axis XY plane gCode interpreter
// Implemented:
//   G0   Rapid movement
//   G1   Cut movement
//   G92  Set position
//   M0   Stop (pause)
//   M3   Spindle on
//   M5   Spindle off
//   M17  Enable motors
//   M18  Disable motors
//   M30  End program (reset)
//   T0   Laser
//   T1   Engraver (default)
//   T?   Tools (1-255)
//   S?   Spindle (?)
// Assumes:
//   G17  XY plane only
//   G21  Metric
//   G90  Absolute coordinates
//   G94  mm/min
// Immediate codes:
//   !    Pause
//   ~    Resume
//   %    Reset
//   ?    Status

#include <Servo.h>

// Hardware mapping 
#define MXstep         D2
#define MYstep         D3
#define MZstep         D4
#define MXDir          D5
#define MYDir          D6
#define MZDir          D7
#define Spindle        D8
#define BAUD        115200
#define StepsPerMM   79


// Controller parameters
bool xReverse=true;
bool yReverse=true;
bool zReverse=false;
long tool=1;         // Current tool 1 (tool 0 is a laser)   
long gCode=0;        // Current G Code (rapid movement)
long mCode=5;        // Current M Code (spindle/laser off)
long spindle=16000;  // Spindle RPM
const int bufferSize=64;
char cmd[bufferSize];
long value[bufferSize];
int head=0;
int tail=0;
// Movement parameters
long stepCountDown;  // Movement steps remaining (-1 is movement just completed and 0 is ready for new movement)
long stepCountUp;    // Movement steps completed (-1 is movement just completed and 0 is ready for new movement)
long pause;          // -1 is paused or 0 is not paused
long xNew,yNew,zNew;
long xCurrent,yCurrent,zCurrent;
long feedConstant,feedRate,cutFeedRate,rapidFeedRate;
unsigned long cutFeedInterval,rapidFeedInterval,feedInterval;
long dx,dy,dz,ax,ay,az,sx,sy,sz,mx,my,mz;

Servo ScribeServo;
int ScribeON = 135;
int ScribeOFF = 45;

// G-Code Tokeniser
  
void tokeniseGCode(void)
{
  static char currentChar=' ';
  static char lastChar=' ';
  static bool readCode=true;
  static char lastCode='%';
  static char number[13]="";
  static bool serialStopped=false;
  static int nptr=0;
  int queue;
  
  queue=head-tail;
  if (queue<0) queue=queue+bufferSize;
  
  // Software serial flow control (XON/XOFF)
  if ((!serialStopped)&&((Serial.available()>48)||(4*queue>bufferSize*3))){
    // XOFF
    Serial.write(0x13);
    serialStopped=true;
  } else if ((serialStopped)&&(Serial.available()<32)&&(2*queue<bufferSize)) {
    // XON
    Serial.write(0x11);
    serialStopped=false;
  }
  
  // Tokenise and pre-process incoming g-Code stream
  if (Serial.available()>0) {
    lastChar=currentChar;
    currentChar=Serial.read();
    // Cover all End Of Line options
    if ((lastChar=='\n')&&(currentChar=='\r')) currentChar=' ';
    if ((lastChar=='\r')&&(currentChar=='\n')) currentChar=' ';
    // Cover comment options
    if (currentChar=='*') readCode=false;                        // Check sum - ignore
    if (currentChar==';') readCode=false;                        // End of line comment
    if (currentChar=='(') readCode=false;                        // Inbedded comment
    if (currentChar==')') readCode=true;                         // End of inbedded comment
    if ((currentChar=='\r')||(currentChar=='\n')) readCode=true; // End of line
    if (readCode) {
      // Check for immediate (non-gCode) commands
      if (currentChar=='%') { // Reset
        readCode=true;
        nptr=0;
        number[0]='\0';
        Serial.println("$ Reset");
        Serial.println("ok"); // Needed here
        Serial.end();
        setup();
      }  else if (currentChar=='!') { // Stop (indefinite pause)
        pause=-1;
        Serial.println("$ Paused");
      } else if (currentChar=='~') { // Resumed
        pause=0;
        Serial.println("$ Resumed");
      } else if (currentChar=='?') { // Status
        Serial.print("$ Status: ");
        if (pause!=0) {
          Serial.println("paused");
        } else {
          if (stepCountDown>0) {
            Serial.println("working");
          } else {
            Serial.println("idle");
          }
        }
        Serial.print("$ Tool: ");
        if (tool==0) {
          Serial.println("laser");
        } else {
          Serial.println(tool);
        }
      }
      // gCodes
      if ((currentChar>='a')&&(currentChar<='z')) currentChar=currentChar-32;
      if ((currentChar>='A')&&(currentChar<='Z')||(currentChar>='0')&&(currentChar<='9')||(currentChar=='-')||(currentChar=='.')||(currentChar=='\r')||(currentChar=='\n')) {
        if ((currentChar>='A')&&(currentChar<='Z')||(currentChar=='\r')||(currentChar=='\n')) {
          // Finish old currentChar (set value)
          if ((lastCode>='A')&&(lastCode<='Z')) {
            if ((lastCode=='X')||(lastCode=='Y')||(lastCode=='Z')) {
              // Convert mm to steps
              if (StepsPerMM==100) {
                {
                  strcat(number,"00");
                  char* tmp=strchr(number,'.');
                  if (tmp!=NULL) {
                    *tmp=*(tmp+1);
                    *(tmp+1)=*(tmp+2);
                    *(tmp+2)='\0';
                  }
                }
                value[head]=atol(number); // 26 us
              } else {  
                value[head]=(long)(atof(number)*StepsPerMM); // 126 us
              }
            } else { 
              value[head]=atol(number);
            }
          }
          // Start new currentChar
          lastCode=currentChar;
          if (++head>=bufferSize) head=head-bufferSize;
          cmd[head]=currentChar;
          value[head]=0;
          nptr=0;
        } else if ((currentChar>='0')&&(currentChar<='9')||(currentChar=='-')||(currentChar=='.')) {
          number[nptr]=currentChar;
          number[++nptr]='\0';
        }
      }
    }
  }
}

void parseGCode(void)
{  
  static bool xFlag=false;
  static bool yFlag=false;
  static bool zFlag=false;
  static bool fFlag=false;
  static bool sFlag=false;
  static long lastGCode=0;

  // Process end of movement codes (mainly mCodes)
  if (stepCountDown==-1) {
    stepCountDown=0;
    stepCountUp=0;
    if (mCode==0) {
      // Stop (pause)
      pause=-1;
    } else if (mCode==5) {
      // Spindle off (laser 0ff)
      //digitalWrite(Spindle,LOW);
      ScribeServo.write(ScribeOFF);
    } else if (mCode==30) {
      // Reset
      Serial.println("ok"); // Needed here
      Serial.end();
      setup();
    } else {
      // Reserved for expansion  
    }
    mCode=-1; // Reset M Code
  }

  // Process start of movement codes
  if ((stepCountDown==0)&&(head!=tail)&&((cmd[head]=='\r')||(cmd[head]=='\n'))) { 
    tail++;
    if (tail>=bufferSize) tail=tail-bufferSize;
    if ((cmd[tail]=='\r')||(cmd[tail]=='\n')) Serial.println("ok");
  
    // Decode G,X,Y,Z,F,S,T
    if (cmd[tail]=='G') {
      lastGCode=gCode;
      gCode=value[tail];
      xFlag=false;
      yFlag=false;
      zFlag=false;
      fFlag=false;
    } else if (cmd[tail]=='X') {
      xNew=value[tail];
      xFlag=true;
      Serial.print("X set to ");
      Serial.println(xNew);
    } else if (cmd[tail]=='Y') {
      yNew=value[tail];
      yFlag=true;
      Serial.print("Y set to ");
      Serial.println(yNew);
      } else if (cmd[tail]=='Z') {
      zNew=value[tail];
      zFlag=true;
      Serial.print("Z set to ");
      Serial.println(zNew);
    } else if (cmd[tail]=='F') {
      feedRate=value[tail];
      if (feedRate<=0) feedRate=1;
      feedInterval=feedConstant/feedRate;
      fFlag=true;
      Serial.print("F set to ");
      Serial.println(feedRate);
    } else if (cmd[tail]=='S') {
      spindle=value[tail];
      if (spindle<0) spindle=0;
      Serial.print("S set to ");
      Serial.println(spindle);
    } else if (cmd[tail]=='T') {
      tool=value[tail];
      if (tool<0) tool=0;
      if (tool>255) tool=255;
      Serial.print("T set to ");
      Serial.println(tool);
    }
    
    // Start of movement mCodes
    if (cmd[tail]=='M') {
      mCode=value[tail];
      if (mCode==3) {
        // Spindle on (laser on)
        ////digitalWrite(Spindle,HIGH);         
        ScribeServo.write(ScribeON);
        Serial.print("S set to ");
        Serial.println(ScribeON);
      } else {
        // Reserved for expansion
      }
    } 
        
    // Execute
    if ((cmd[tail]=='\r')||(cmd[tail]=='\n')) {
      if (gCode==0) {
        // Rapid movement
        if (fFlag) {
          rapidFeedRate=feedRate;
          rapidFeedInterval=feedInterval;
        } else {
          feedRate=rapidFeedInterval;
          feedInterval=rapidFeedInterval;
        }
      } else if (gCode==1) {
        // Cutting movement
        if (fFlag) {
          cutFeedRate=feedRate;
          cutFeedInterval=feedInterval;
        } else {
          feedRate=cutFeedInterval;
          feedInterval=cutFeedInterval;
        }
      } else if (gCode==92) {
        // Set current position
        if (xFlag) xCurrent=xNew;
        if (yFlag) yCurrent=yNew;
        if (zFlag) zCurrent=zNew;
        gCode=lastGCode;
      } else {          
        // Reserved for expansion
      } 
      xFlag=false;
      yFlag=false;
      zFlag=false;
      fFlag=false;
      
      // Laser
      if (tool==0) {
        zNew=zCurrent;                            // Laser cannot change Z
        if (gCode==1) //digitalWrite(Spindle,HIGH);
          ScribeServo.write(ScribeON);  // Laser off for rapid moves
          Serial.print("S set to ");
          Serial.println(ScribeON);
        if (gCode==0) //digitalWrite(Spindle,LOW);
          ScribeServo.write(ScribeOFF); // Laser on for cutting moves
          Serial.print("S set to ");
          Serial.println(ScribeOFF);
      }

      // Determine movement parameters
      dx=xNew-xCurrent;
      dy=yNew-yCurrent;
      dz=zNew-zCurrent;
      ax=abs(dx);
      ay=abs(dy);
      az=abs(dz);
      // Execution parameters
      sx=xNew<xCurrent?-1:xNew>xCurrent?1:0;
      sy=yNew<yCurrent?-1:yNew>yCurrent?1:0;
      sz=zNew<zCurrent?-1:zNew>zCurrent?1:0;
      if ((ax>=ay)&&(ax>=az)) {
        mx=0;
        my=ay-(ax>>1);
        mz=az-(ax>>1);
        stepCountDown=ax;
      } else if ((ay>=ax)&&(ay>=az)) {
        mx=ax-(ay>>1);
        my=0;
        mz=az-(ay>>1);
        stepCountDown=ay;
      } else {
        mx=ax-(az>>1);
        my=ay-(az>>1);
        mz=0;
        stepCountDown=az;
      }
      if (stepCountDown>0) {
        stepCountUp=1;
      } else {
        stepCountDown=-1;
        stepCountUp=-1;        
      }
      // Set the stepper directions
      if (xReverse) {
        digitalWrite(MXDir,(1-sx)>>1);
      } else {
        digitalWrite(MXDir,(sx+1)>>1);
      } 
      if (yReverse) {
        digitalWrite(MYDir,(1-sy)>>1);
      } else {
        digitalWrite(MYDir,(sy+1)>>1);
      }
      if (zReverse) {
        digitalWrite(MZDir,(1-sz)>>1);
      } else {
        digitalWrite(MZDir,(sz+1)>>1);
      }
    }
  }
}

void advanceSteppers(void)
{ 
  static bool cycleFlag=false;
  static unsigned long feedPreviousMicros=0;
  static unsigned long feedCurrentMicros=0;
  long stepX,stepY,stepZ;
  unsigned long ramp;

  if ((pause==0)&&(stepCountDown>0)) {
    // Maximum 1g ramp and 500pps (pull-in)
    ramp=400*max(max(6-stepCountUp,0),max(6-stepCountDown,0));
    feedCurrentMicros=micros();
    if (feedCurrentMicros-feedPreviousMicros>feedInterval+ramp) {
      feedPreviousMicros=feedCurrentMicros;
      // Advance steppers
      stepX=0;
      stepY=0;
      stepZ=0;
      if ((ax>=ay)&&(ax>=az)) {
        if (my>=0) {
          my-=ax;
          stepY=sy;
        }
        if (mz>=0) {
          mz-=ax;
          stepZ=sz;
        }
        my+=ay;
        mz+=az;
        stepX=sx;
      } else if ((ay>=ax)&&(ay>=az)) {
        if (mx>=0) {
          mx-=ay;
          stepX=sx;
        }
        if (mz>=0) {
          mz-=ay;
          stepZ=sz;
        }
        mx+=ax;
        mz+=az;
        stepY=sy;
      } else {
        if (mx>=0) {
          mx-=az;
          stepX=sx;
        }
        if (my>=0) {
          my-=az;
          stepY=sy;
        }
        mx+=ax;
        my+=ay;
        stepZ=sz;
      }
      xCurrent+=stepX;
      yCurrent+=stepY;
      zCurrent+=stepZ;
      // Step pulse (high for at least 10 us)
      digitalWrite(MXstep,abs(stepX));
      digitalWrite(MYstep,abs(stepY));
      digitalWrite(MZstep,abs(stepZ));
      delayMicroseconds(10);
      digitalWrite(MXstep,LOW);
      digitalWrite(MYstep,LOW);
      digitalWrite(MZstep,LOW);
      stepCountDown--;
      stepCountUp++;
      // Flag end of movement 
      if (stepCountDown==0) {
        stepCountDown=-1;
        stepCountUp=-1;
      }
    }
  } else {
    feedPreviousMicros=micros(); // Reset feed rate clock to ensure near full cycle on start
  }
}

void setup()
{
  // Reset Global Variables
  tool=1;
  gCode=0;  
  mCode=5;    
  spindle=16000;
  pause=0;
  head=0;
  tail=0;
  cmd[0]='\n';
  xNew=0;
  yNew=0;
  zNew=0;
  xCurrent=0;
  yCurrent=0;
  zCurrent=0;
  stepCountDown=0;
  stepCountUp=0;
  feedRate=1200;
  cutFeedRate=300;
  rapidFeedRate=1200;
  feedConstant=60000000/StepsPerMM;
  cutFeedInterval=feedConstant/cutFeedRate;
  rapidFeedInterval=feedConstant/rapidFeedRate;
  feedInterval=rapidFeedInterval;
  // Initialise H-Bridge
  pinMode(MXDir,OUTPUT);
  pinMode(MXstep,OUTPUT);
  pinMode(MYDir,OUTPUT);
  pinMode(MYstep,OUTPUT);
  pinMode(MZDir,OUTPUT);
  pinMode(MZstep,OUTPUT);
  //pinMode(Spindle,OUTPUT);
  ScribeServo.attach(Spindle);
  digitalWrite(MXDir,LOW);
  digitalWrite(MXstep,LOW);
  digitalWrite(MYDir,LOW);
  digitalWrite(MYstep,LOW);
  digitalWrite(MZDir,LOW);
  digitalWrite(MZstep,LOW);
  //digitalWrite(Spindle,LOW);         
  ScribeServo.write(ScribeOFF);
  // Initialise LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  // Initialize serial:
  Serial.begin(BAUD,SERIAL_8N1);
  Serial.println("");
  Serial.println("Grbl v0.1a");
}

void loop()
{
  static unsigned long intervalMillisLED=0;
  static unsigned long currentMillisLED=0;
  static unsigned long previousMillisLED=0;

  // Use LED to indicate working
  if (stepCountDown==0) {
    intervalMillisLED=1000;
  } else {
    intervalMillisLED=100;    
  }
  currentMillisLED=millis();
  if (currentMillisLED-previousMillisLED>intervalMillisLED) {
    previousMillisLED=currentMillisLED;   
    digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));
  }
  // Process the serial stream
  tokeniseGCode();   // Tokenise a character at a time from the serial stream
  parseGCode();      // Parse the code on end of line received
  advanceSteppers(); // Advance the stepper motors one step
}
