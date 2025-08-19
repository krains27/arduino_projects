#include <PID_v1.h>
#include <util/atomic.h>

// Define control pins
int enA = 2;
int in1 = 3;
int in2 = 4;
int enB = 5;
int in3 = 6;
int in4 = 7;

// Pin numbers for encoder
#define encoderPinA1 20
#define encoderPinB1 21
#define encoderPinA2 18
#define encoderPinB2 29

//Encoder counts
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

// //PID control variables
// long previousTime = 0;
// float ePrevious = 0;
// float eIntegral = 0;
double setpointA, setpointB, inputA, inputB, outputA, outputB; 
double kp = .8, ki = .15, kd = .25;

PID myPIDA(&inputA, &outputA, &setpointA, kp, ki, kd, DIRECT);
PID myPIDB(&inputB, &outputB, &setpointB, kp, ki, kd, DIRECT);

// void setup(){
//   Serial.begin(9600);

//   //Set pin modes
//   pinMode(enA, OUTPUT);
//   pinMode(in1, OUTPUT);
//   pinMode(in2, OUTPUT);
//   pinMode(encoderPinA, INPUT);
//   pinMode(encoderPinB, INPUT);

//   //Encoder interrupt
//   attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, RISING);
// }
void setup(){
  Serial.begin(9600);
    //Set pin modes
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(encoderPinA1, INPUT);
  pinMode(encoderPinB1, INPUT);
  pinMode(encoderPinA2, INPUT);
  pinMode(encoderPinB2, INPUT);

  setpointA = 10000;
  setpointB = encoderCountA; 
  
  myPIDA.SetMode(AUTOMATIC);
  myPIDB.SetMode(AUTOMATIC);
  //Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinA1), handleEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), handleEncoderB, RISING);

}

void loop(){
  inputA = encoderCountA;
  inputB = encoderCountB;
  setpointB = (encoderCountA < setpointA) ? encoderCountA : setpointA;
  
  myPIDA.Compute();
  myPIDB.Compute();


   //Print statements for debugging
  Serial.print(setpointA);
  Serial.print(", ");
  Serial.print(outputA);
  Serial.print(", ");
  Serial.print(encoderCountA);

  Serial.print("-----");
  Serial.print(setpointB);
  Serial.print(", ");
  Serial.print(outputB);
  Serial.print(", ");
  Serial.println(encoderCountB);

   //Control motor
  // moveMotor(enA, in1, in2, output);
  moveMotor(enA, in1, in2, outputA);
  moveMotor(enB, in3, in4, outputB);
}

// void loop(){
//  //Set point
//  int target = 230;

//  //PID gains and computation
//  float kp = 2.0; 
//  float kd = 0.1;
//  float ki = 0.016;
//  float u = pidController(target, kp, kd, ki);

//  //Control motor
//  moveMotor(enA, in1, in2, u);

//  //Print statements for debugging
//  Serial.print(target);
// //  Serial.print(", ");
// //  Serial.print(u);
//  Serial.print(", ");
//  Serial.println(encoderCount);
// }

// Function called during interrupts
void handleEncoderA(){
  if(digitalRead(encoderPinA1) > digitalRead(encoderPinB1)){  
    encoderCountA++;    
  }
  else{    
    encoderCountA--;
  }
}

void handleEncoderB(){
  if(digitalRead(encoderPinA2) > digitalRead(encoderPinB2)){  
    encoderCountB++;    
  }
  else{    
    encoderCountB--;
  }
}

//Function to move motor
void moveMotor(int en, int inA, int inB, float u){
  //Maximum motor speed
  float speed = fabs(u); 
  if(speed > 255){
    speed = 255;
  }

  //Set the direction
  int direction = 1;
  if(u < 0){
    direction = 0;
  }

  if(direction){
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  }
  else{
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  }
  
  //Control the motor
  analogWrite(en, speed);
  
}

// float pidController(int target, float kp, float kd, float ki){
//   //Measure the time elapsed since the last iteration
//   long currentTime = micros();
//   float deltaT = ((float)(currentTime - previousTime)) / 1.0e6;

//   //Compute the error, derivative, and integral
//   int e = encoderCount - target;
//   float eDerivative = (e - ePrevious) / deltaT;
//   eIntegral = eIntegral + e * deltaT;

//   //Compute the PID control signal
//   float u = (kp * e) + (kd * eDerivative) + (ki * eIntegral);

//   //Update variables for the next iteration
//   previousTime = currentTime; 
//   ePrevious = e; 

//   return u;
// }
