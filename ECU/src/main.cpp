#include <Arduino.h>
#include <SPI.h> // SPI Library
#include <math.h>

#define Kp 200.0
#define Ki 15.0
#define Kd 0.0
// #define Fc_minus 1.25
// #define Fc_plus  1.25
#define deadBand  40
#define minError  0.035 
#define xT 0.1 // INTERVAL(Seconds) == 0.1 
#define INTERVAL 100000

#define CPR 16384
#define CONVERSION_RATIO 2*PI/CPR

float readPosition(int pin);
int deadZoneInverse(int input, int leftH, int rightH);

SPISettings settings(10000000, MSBFIRST, SPI_MODE1);

uint8_t dir1Pin = 7;
uint8_t u1Pin = 6;

uint8_t dir2Pin = 4;
uint8_t u2Pin = 5;


//Set Slave Select Pin
//MOSI, MISO, CLK are handeled automatically

uint8_t CSN1 = 10;
uint8_t CSN2 = 9;

float position1 = 0;
float position2 = 0;

float speed1 = 0;
float speed2 = 0;

float u_position1 = 0;
float u_position2 = 0;

float setpoint1 = 3.921;
float setpoint2 = 3.921;

float error1 = 0;
float error2 = 0;

float u_error1 = 0;
float u_error2 = 0;

float integral1 = 0;
float itegral2 = 0;

float diff1 = 0;
float diff2 = 0;

int u1 = 0;
int u2 = 0;

int motorSignal1 = 0;
int motorSignal2 = 0;

float maxLim1 = 5.579;
float maxLim2 = 5.579;
float minLim1 = 2.345;
float minLim2 = 2.345;


enum input_States
{
  Rest,     // Reposo 
  state1,   // Escalon 1
  state2,   // Escalon 2
  state3,   // Escalones pequeños abajo
  state4,   // Escalones pequeños arriba
  sineStateSlow,  // Senoide lento
  stop,           // Parada
  sineStateFast   // Senoide rapido 
};

float rad = 0;

int stateTimeElapsed = 0;
int counter = 0;

enum input_States currentState = Rest;
enum input_States prevState =  Rest;


void setup()
{
  Serial.begin(115200);
  SerialUSB.begin(115200);
  
  //Set Slave Select Pin Directions
  pinMode(CSN1, OUTPUT);
  pinMode(CSN2, OUTPUT);
  //Set Slave Select High to Start i.e disable chip
  digitalWrite(CSN1, HIGH);
  digitalWrite(CSN2, HIGH);

  pinMode(u1Pin,OUTPUT);
  pinMode(u2Pin,OUTPUT);
  pinMode(dir1Pin,OUTPUT);
  pinMode(dir2Pin,OUTPUT);
  
  digitalWrite(u1Pin, LOW);
  digitalWrite(u2Pin, LOW);
  digitalWrite(dir1Pin, LOW);
  digitalWrite(dir2Pin, LOW);

  delay(2000);

  SerialUSB.println("Hello world");
  SPI.begin(); // Start I2C Bus as Master
}

void loop()
{
  
  long startTime = micros();

  if(stateTimeElapsed >= 40){
    stateTimeElapsed = 0;
    switch(currentState){
      case Rest:
        currentState = state1;
        setpoint1 = 3;
        break;
      
      case state1:
        setpoint1 = 5;
        currentState = state2;
        break;

      case state2:
      setpoint1 = 4.8;
        currentState = state3;
        break;
      
      case state3:
      setpoint1 = 4.6;
        currentState = state4;
        break;
      
      case state4:
      setpoint1 = 4.4;
        currentState = sineStateSlow;
        break;

      case sineStateSlow:
        counter++;
        if(counter>10){
          currentState = stop;
          counter=0;
        }
        break;
      
      case stop:
        currentState = sineStateFast;
        break;
      
      case sineStateFast:
        counter++;
        if(counter>6){
          currentState = Rest;
          counter=0;
        }
       
        break;
    }
    SerialUSB.println("Current state: " + String(currentState));

  }

  // if(SerialUSB.available()){
  //   setpoint1 = SerialUSB.parseInt();
  //   SerialUSB.parseInt();
  // }

  if(currentState == sineStateSlow){
    setpoint1 = (maxLim1-minLim1)*(sin(rad) + 1)*0.5+minLim1; 
    rad = rad + 0.015;
  }

  if(currentState == sineStateFast){
    setpoint1 = (maxLim1-minLim1)*(sin(rad) + 1)*0.5+minLim1; 
    rad = rad + 0.04;
  }

  
  // Read sensors
  position1 = readPosition(CSN1);

  // Calculate errors
  error1 = setpoint1 - position1;

  if(abs(error1) < minError){
    error1 = 0;
    integral1 = 0;
  }

  integral1 = xT*error1 + integral1;

  diff1 = ((float)error1 - (float)u_error1)/xT;
  u_error1 = error1;

  // Control equation
  u1 = Kp*error1 + Ki*integral1 + Kd*diff1;
 
  // Set directions 
  if(u1<0){
    digitalWrite(dir1Pin,HIGH); // Set direction
    motorSignal1 = deadZoneInverse(abs(u1),-deadBand,deadBand); // Deadzone compensation
  
    if(motorSignal1>255){ // Saturate
      motorSignal1 = 255;
    }
    motorSignal1 = 255 - motorSignal1; // Driver specifig inversion of control signals
  }else{
    digitalWrite(dir1Pin,LOW); // Set direction 
    motorSignal1 = deadZoneInverse(u1,-deadBand,deadBand); // Deadzone compensation
    if(motorSignal1>255){ // Saturate
      motorSignal1 = 255;
    }
  }

  analogWrite(u1Pin,motorSignal1);
  
  Serial.println(String(setpoint1) + " " + String(setpoint1+minError) + " " + String(setpoint1-minError) + " " + String(position1)); // + " " + String(u1) + " " + String(motorSignal1));
  SerialUSB.println(String(u1) + " " + String(error1)+ " " + String(integral1)+ " " + String(diff1));

  long elapsedTime = micros()-startTime;

  if(elapsedTime<INTERVAL){
    delayMicroseconds(INTERVAL-elapsedTime);
    stateTimeElapsed++;
  }else{
    Serial.println("LOOOP IS FUCKED");
  }
} 

float readPosition(int pin)
{
  SPI.beginTransaction(settings);
  //Enable chip
  digitalWrite(pin, LOW);
  unsigned int angle = SPI.transfer16(0x3FFF);
  angle = (angle & 0x3FFF);
  //Wait 10 miliseconds for chip to write data (Time between CSn falling edge and CLK rising edge is 350ns)
  // delay(1);
  //disable device
  digitalWrite(pin, HIGH);
  SPI.endTransaction();

  // Convert to SI units
  float radian = angle*CONVERSION_RATIO;

  return radian;
}

int deadZoneInverse(int input,int leftH,int rightH){
  int h = 0;
  if(input == 0){
    h = 0;
  }else if(input > 0){
    h = rightH;
  }else if(input < 0){
    h = leftH;
  }
  return input + h;
}