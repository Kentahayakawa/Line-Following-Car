#include <ECE3.h>
// Motor board pins

const double leftSLPPin = 31;  //3.7
const double rightSLPPin = 11; //3.6

const double leftDIRPin = 29;  //1.7
const double rightDIRPin = 30; //1.6

const double leftPWMPin = 40;  //2.7
const double rightPWMPin = 39; //2.6

//========= Other globals =========//

const double photoWeight1=-20.0;
const double photoWeight2=-14.0;
const double photoWeight3=-7.0;
const double photoWeight4=-2.0;
const double photoWeight5=2.0;
const double photoWeight6=7.0;
const double photoWeight7=14.0;
const double photoWeight8=20.0;

// car control variables

double lineCount=0;
double lspeed = 60;
double rspeed = 60;
double motorSpeed;
int line=0;


// There are only 8 sensors but position 0 in array is not used for
// later code readability
uint16_t sensor[8];

bool reached_line=false;

double kp = 0.04;
double kd = 0.4;
 
unsigned long currentTime;
unsigned long prevTime=0;
double elapsedTime;
double error;
double lastError;
double rateError;
double flag=0.0;

int direction;

//========= Setup =========//
// QTR-8RC module requires unusual setting of phototransistor pin modes
// in the loop rather than setup.

void setup() {

// Line sensor pins
  ECE3_Init();
// Motor board pins

  pinMode(leftSLPPin, OUTPUT);
  pinMode(rightSLPPin, OUTPUT);
 
  pinMode(leftDIRPin, OUTPUT);
  pinMode(rightDIRPin, OUTPUT);

  pinMode(leftPWMPin, OUTPUT);
  pinMode(rightPWMPin, OUTPUT);

  
//Serial.begin(9600);

}



void loop()
{
  // Turn wheels on so they are ready to turn when they sense a line

  digitalWrite(leftDIRPin, LOW);
  digitalWrite(leftSLPPin, HIGH);
//  analogWrite(leftPWMPin, lspeed);    
  
  digitalWrite(rightDIRPin, LOW);
  digitalWrite(rightSLPPin, HIGH);
//  analogWrite(rightPWMPin, rspeed); */

  ECE3_read_IR(sensor);

 /* Serial.print(sensor[0]);
  Serial.print("  ");
  Serial.print(sensor[1]);
  Serial.print("  ");
  Serial.print(sensor[2]);
  Serial.print("  ");
  Serial.print(sensor[3]);
  Serial.print("  ");
  Serial.print(sensor[4]);
  Serial.print("  ");
  Serial.print(sensor[5]);
  Serial.print("  ");
  Serial.print(sensor[6]);
  Serial.print("  ");
  Serial.println(sensor[7]); */

  currentTime=millis();
  
  error=(sensor[0]*photoWeight1)+(sensor[1]*photoWeight2)+(sensor[2]*photoWeight3)
                +(sensor[3]*photoWeight4)+(sensor[4]*photoWeight5)+(sensor[5]*photoWeight6)
                +(sensor[6]*photoWeight7)+(sensor[7]*photoWeight8);
   rateError=(error-lastError);
   motorSpeed=(kp*error)+(kd*rateError);
   if(motorSpeed<0)
   {
    motorSpeed=-motorSpeed;
   }
   lastError=error;
   
  direction=turnDirection(error);
  adjustCourse(direction,error);
  
//Code to make a doughtnut turn//
 if(detect_line(sensor)) 
  {
    if(flag<1.0)
    {
    digitalWrite(leftDIRPin, HIGH);
    digitalWrite(rightDIRPin, LOW);  //spin right wheel backwards
    analogWrite(leftPWMPin, 70);
    analogWrite(rightPWMPin, 70); //spin backwards at same speed as left wheel

    delay(700);

    digitalWrite(leftDIRPin, LOW);
    digitalWrite(rightDIRPin, LOW);
    analogWrite(leftPWMPin, rspeed);
    analogWrite(rightPWMPin, rspeed);
    flag++;
    }
    else if(flag>=1.0)
    {
      analogWrite(leftPWMPin, 0);
      analogWrite(rightPWMPin, 0);
      delay(10000);
    }
    
  }
}

bool detect_line(uint16_t arr[])
{
  int count=0;
  /*for(int i=0;i<8;i++)
  {
    if(arr[i]==2500) //if sensor value is black
      count++;  //increase count
  }*/
  int s=arr[0]+arr[1]+arr[2]+arr[3]+arr[4]+arr[5]+arr[6]+arr[7];
  if(s>19000)  //if all sensors detected black
    return true; //we have detected line
  else
    return false; //not a black, checkpoint line
} 


int turnDirection(double sensorVal)
{
  if(sensorVal>1.00)
    return 2;
  if(sensorVal<-1.00)
    return 1;
  else
    return 0;
}


void adjustCourse(int dir, double err)
{

  double netspeed1=rspeed-motorSpeed*(50/1700.0);
  double netspeed2=rspeed+motorSpeed*(50/1700.0);
  if(netspeed1 < 0)
    netspeed1=0;
  if(netspeed2 > 90)
    netspeed2=90;
  if(netspeed1 > 90)
    netspeed1=90;
  if(netspeed2<0)
    netspeed2=0;
  
    switch(dir)
    {
      case 0: 
        analogWrite(rightPWMPin, rspeed);
        analogWrite(leftPWMPin, rspeed);
        break;
      case 1:
        analogWrite(rightPWMPin, netspeed1);
        analogWrite(leftPWMPin, netspeed2);
        break;
      case 2:
        analogWrite(rightPWMPin, netspeed2);
        analogWrite(leftPWMPin, netspeed1);
        break;
    }
    
}