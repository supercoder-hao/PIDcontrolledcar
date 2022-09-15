#include <ECE3.h>

const int left_nslp_pin=31;
const int right_nslp_pin=11;
const int left_dir_pin=29;
const int left_pwm_pin=40;
const int right_dir_pin=30;
const int right_pwm_pin=39;
const int LED_RF = 41;

const uint16_t minvalues[8] = {342 ,365, 296, 318 ,227, 296, 273, 342};
const uint16_t maxvalues[8] = {1858, 1757,  1904 , 1255  ,1346 , 1803 , 1684 , 1858};

uint16_t sensorValues[8];
uint16_t minremove_sensorValues[8];
uint16_t normalized_sensorValues[8];
float weighted_sensorValue;

float oldvalue = 0;
float PID = 0;
float prePID = 0;
float delta_value;
float kp = 0.07; 
float kd = 0.42;
int leftSpd=50;
int rightSpd=48;

int count = 0;

bool preexist_2500 = false;

bool checkwhite = false;

bool starting = true;

int waswhite = 0;

bool roundtrip = false;
int encodervalue = 0;

int lastencodervalue = 0;

int startingpos = 0;
int currentstate = 0;
int firstcurve = 1900;
int secondturns = 1800;
int thirdcurve = 3400;
int fourthslowforreading = 3000;


void setup() {
  ECE3_Init();
  
  pinMode(left_nslp_pin,OUTPUT);
  pinMode(right_nslp_pin,OUTPUT);
  pinMode(left_dir_pin,OUTPUT);
  pinMode(left_pwm_pin,OUTPUT);
  pinMode(right_dir_pin,OUTPUT);
  pinMode(right_pwm_pin,OUTPUT);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);

  pinMode(LED_RF, OUTPUT);


  Serial.begin(9600); 
  delay(2000);
}

void loop() {
  ECE3_read_IR(sensorValues);

  for (unsigned char i=0; i<8; i++) {
    minremove_sensorValues[i] = sensorValues[i]-minvalues[i];
  }

  for (unsigned char i=0; i<8; i++) {
    if (minremove_sensorValues[i] == 0)
   normalized_sensorValues[i] = 0;
    else
   normalized_sensorValues[i] = (minremove_sensorValues[i] * 1000)/ maxvalues[i];
  }

 
  weighted_sensorValue = (-8*normalized_sensorValues[0]-4*normalized_sensorValues[1]-2*normalized_sensorValues[2]-normalized_sensorValues[3]
                          +normalized_sensorValues[4]+2*normalized_sensorValues[5]+4*normalized_sensorValues[6]+8*normalized_sensorValues[7])/4;
 

  delta_value = weighted_sensorValue-oldvalue;
  PID = kp*weighted_sensorValue+ kd * (weighted_sensorValue-oldvalue);

  if (starting) {

    analogWrite(left_pwm_pin,60); 
    analogWrite(right_pwm_pin,30);
    changeWheelSpeeds(35,25,50,50);
    starting = false;
    changeWheelSpeeds(constrain(leftSpd - (int)PID,0,255),constrain(rightSpd + (int)PID,0,255),80,80);


  }

///////////////////////////////////////////////////////////////////////////////////encoder settings start

  if (roundtrip)
    encodervalue = getEncoderCount() - lastencodervalue;
  else
    encodervalue = getEncoderCount();
    

  if (encodervalue - startingpos <= firstcurve && currentstate == 0) {
    //set some speed
    kp = 0.08; 
    kd = 0.95;
    leftSpd=240;
    rightSpd=238;
  }
  else if (currentstate == 0) {
    startingpos = firstcurve;
    currentstate++;
  }

  if (encodervalue - startingpos <= secondturns && currentstate == 1) {
    //set some speed
    kp = 0.19; 
    kd = 0.9;
    leftSpd=100;
    rightSpd=98;

  }
  else if (currentstate == 1) {
    startingpos = secondturns;
    currentstate++;
  }

  if (encodervalue - startingpos <= thirdcurve && currentstate == 2) {
    //set some speed
    kp = 0.08; 
    kd = 0.95;
    leftSpd=240;
    rightSpd=238;
  }
  else if (currentstate == 2) {
    startingpos = thirdcurve;
    currentstate++;
  }

  if (encodervalue - startingpos <= fourthslowforreading && currentstate == 3) {
    //set some speed
    kp = 0.09; 
    kd = 0.65;
    leftSpd=60;
    rightSpd=58;
  }
  else if (currentstate == 3) {
    startingpos = 0;
    currentstate = 0;
    roundtrip = true;
    lastencodervalue = getEncoderCount()+300;
  }

  /////////////////////////////////////////////////////////////////////////////////// encoder settings end
  if (!starting) {
    analogWrite(left_pwm_pin,constrain(leftSpd - (int)PID,0,255)); 
    analogWrite(right_pwm_pin,constrain(rightSpd + (int)PID,0,255));
  }
     
  if (sensorValues[0] != 2500 && sensorValues[1] != 2500 && sensorValues[2] != 2500 && sensorValues[3] != 2500 && sensorValues[4] != 2500 && 
      sensorValues[5] != 2500 && sensorValues[6] != 2500 && sensorValues[7] != 2500 || preexist_2500) {

        if (detectwhite()) {
          digitalWrite(LED_RF, LOW);

          if (checkwhite == true) {
            if (count == 0) {
              changeWheelSpeeds(100,100,0,0);
              donut();
              starting = true;
              count++;

            }
            else {
              analogWrite(left_pwm_pin,60); 
              analogWrite(right_pwm_pin,60);
              changeWheelSpeeds(100,100,60,60);
              digitalWrite(left_nslp_pin,LOW);
              digitalWrite(right_nslp_pin,LOW);
            }
          }
        }
        
        checkwhite = false;

        if (sensorValues[0] > 1000 && sensorValues[1] > 1000 && sensorValues[2] > 1000 
            && sensorValues[5] > 1000 && sensorValues[6] > 1000 && sensorValues[7] > 1000 && abs(prePID - PID) <= 10) {
          digitalWrite(LED_RF, HIGH);
          delay(120);
          
          checkwhite = true;
          preexist_2500 = false;        
        } 

         oldvalue = weighted_sensorValue;
         prePID = PID;
         //preexist_2500 = false;
  }
  else {
    preexist_2500 = true;
  }

}


void donut() 
{
  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,HIGH);
  digitalWrite(right_nslp_pin,HIGH);

  analogWrite(left_pwm_pin,255);
  analogWrite(right_pwm_pin,255);

  delay(140);

  digitalWrite(left_dir_pin,LOW);
  digitalWrite(left_nslp_pin,HIGH);
  digitalWrite(right_dir_pin,LOW);
  digitalWrite(right_nslp_pin,HIGH);
}

bool detectwhite()
{
  if (sensorValues[0] < 880 && sensorValues[1] < 880 && sensorValues[2] < 770 && sensorValues[3] < 750 
      && sensorValues[4] < 610 && sensorValues[5] < 740 && sensorValues[6] < 790 && sensorValues[0] < 930) {
    //all white stop
    return true;
  }
  else
    return false;
}

void changeWheelSpeeds(int initialLeftSpd, int finalLeftSpd, int initialRightSpd, int finalRightSpd) {
/*  
 *   This function changes the car speed gradually (in about 30 ms) from initial
 *   speed to final speed. This non-instantaneous speed change reduces the load 
 *   on the plastic geartrain, and reduces the failure rate of the motors. 
 */
  int numSteps = 6;
  int pwmLeftVal = initialLeftSpd; // initialize left wheel speed 
  int pwmRightVal = initialRightSpd;  // initialize right wheel speed 
  int deltaLeft = (finalLeftSpd-initialLeftSpd)/numSteps; // left in(de)crement
  int deltaRight = (finalRightSpd-initialRightSpd)/numSteps;  // right in(de)crement

  for(int k=0;k<numSteps;k++) {
    pwmLeftVal = pwmLeftVal + deltaLeft;
    pwmRightVal = pwmRightVal + deltaRight;
    analogWrite(left_pwm_pin,pwmLeftVal);    
    analogWrite(right_pwm_pin,pwmRightVal); 
    delay(60);   
  } // end for int k
} // end void changeWheelSpeeds

int getEncoderCount()  //average pulse count
{
  int getL=getEncoderCount_left();
  int getR=getEncoderCount_right();
//  Serial.print(getL);Serial.print("\t");Serial.println(getR);
  return ((getEncoderCount_left() + getEncoderCount_right())/2);
}
