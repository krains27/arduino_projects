#include <PID_v1.h>
#include <util/atomic.h>

// Define control pins
int enA = 4;
int in1 = 5;
int in2 = 6;
int enB = 7;
int in3 = 8;
int in4 = 9;

int ctrlBtn = 53;

// Forward ultrasonic sensor
int trigPin = 50;
int echoPin = 52;

// Pin numbers for encoder
#define encoderPinA1 3
#define encoderPinB1 2
#define encoderPinA2 18
#define encoderPinB2 19
// #define maxSpeed = 255;
#define maxSpeed 100
#define usDelay 250
#define closeDistance 30

//Encoder counts
volatile long encoderCountA = 0;
volatile long encoderCountB = 0;

double setpointA, setpointB, inputA, inputB, outputA, outputB; 
double kp = .8, ki = .15, kd = .25;

// PID myPIDA(&inputA, &outputA, &setpointA, kp, ki, kd, DIRECT);
// PID myPIDB(&inputB, &outputB, &setpointB, kp, ki, kd, DIRECT);

void setup(){
  Serial.begin(9600);
    //Set pin modes
  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(encoderPinA1, INPUT_PULLUP);
  pinMode(encoderPinB1, INPUT_PULLUP);
  pinMode(encoderPinA2, INPUT_PULLUP);
  pinMode(encoderPinB2, INPUT_PULLUP);
  pinMode(ctrlBtn, INPUT_PULLUP);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  setpointA = 100;
  setpointB = encoderCountA; 
  
  // myPIDA.SetMode(AUTOMATIC);
  // myPIDB.SetMode(AUTOMATIC);

  //Encoder interrupt
  attachInterrupt(digitalPinToInterrupt(encoderPinB1), handleEncoderA, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderPinA2), handleEncoderB, RISING);

}

void loop(){
  inputA = encoderCountA;
  inputB = encoderCountB;
  setpointB = (encoderCountA < setpointA) ? encoderCountA : setpointA;
  int distance = 0;
  long int duration = 0;
  bool noObject = true; 
  
  // myPIDA.Compute();
  // myPIDB.Compute();


   //Print statements for debugging
  // Serial.print(setpointA);
  // Serial.print(", ");
  // Serial.print(outputA);
  // Serial.print(", ");
  // Serial.print(encoderCountA);

  // Serial.print("-----");
  // Serial.print(setpointB);
  // Serial.print(", ");
  // Serial.print(outputB);
  // Serial.print(", ");
  // Serial.println(encoderCountB);




   //Control motor
  // moveMotor(enA, in1, in2, 75);




  // distance = getUSDistance(trigPin, echoPin);
  // Serial.println(getUSDistance(trigPin, echoPin));
  // delay(2000);

  // distance = getUSDistance(trigPin, echoPin);
  noObject = getUSDistance(trigPin, echoPin) >= closeDistance ? true : false;

  Serial.println(noObject);
  moveMotor(enA, in1, in2, 0);
  moveMotor(enB, in3, in4, 0);

  while(digitalRead(ctrlBtn) == LOW && noObject)
  {
    Serial.println("Running Code");
    moveMotor(enA, in1, in2, 75);
    moveMotor(enB, in3, in4, 75);

    noObject = getUSDistance(trigPin, echoPin) >= closeDistance ? true : false;  
    
    if(!noObject)
    {
      Serial.println("Reverse");
      moveMotor(enA, in1, in2, -100);
      moveMotor(enB, in3, in4, -100);
      delay(1000);
      moveMotor(enA, in1, in2, 0);
      moveMotor(enB, in3, in4, 0);

      while(!noObject)
    {
      Serial.println("Turning Left");
      moveMotor(enA, in1, in2, 100);
      moveMotor(enB, in3, in4, 0);

      delay(1000);
      moveMotor(enA, in1, in2, 0);

      noObject = getUSDistance(trigPin, echoPin) >= closeDistance ? true : false;
    }
    }
    
    
  }

    // if(!noObject)
    // {
    //   encoderCountA = 0; 

    //   while(encoderCountA < 10)
    //   {
    //     moveMotor(enA, in1, in2, -25);
    //     Serial.println("Running Code in loop");
    //   }

    //   moveMotor(enA, in1, in2, 0);
            
    // }
    moveMotor(enA, in1, in2, 0);
    moveMotor(enB, in3, in4, 0);

    // stopMotor(in1, in2);
    // stopMotor(in3, in4);


    // distance = getUSDistance(trigPin, echoPin);

    // Serial.println(distance);

    // delay(2000);



  
 


  // if(encoderCountA > 200)
  // {
  //   moveMotor(enA, in1, in2, 0);
  // }
  // else
  // {
  //   moveMotor(enA, in1, in2, 75);
  // }                                  
  // // moveMotor(enB, in3, in4, 0);

  // if(encoderCountB > 200)
  // {
  //   moveMotor(enB, in3, in4, 0);  
  //   Serial.println(encoderCountB);
       
  // }
  // else
  // {
  //   moveMotor(enB, in3, in4, 75);
    
  // }
  



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

int getUSDistance(int tPin, int ePin)
{
  long duration = 0;
  int distance = 0;

  digitalWrite(tPin, LOW);
  delayMicroseconds(2);
  digitalWrite(tPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(tPin, LOW);

  duration = pulseIn(ePin, HIGH);

  distance = (duration * 0.0343) / 2;

  delay(usDelay);

  return distance;
}

// Function called during interrupts
void handleEncoderA(){
  encoderCountA++;
  // if(digitalRead(encoderPinB1) > digitalRead(encoderPinA1)){  
  //   encoderCountA++;    
  // }
  // else{    
  //   encoderCountA--;
  // }   
}

void handleEncoderB(){
  encoderCountB++;
  // if(digitalRead(encoderPinB2) > digitalRead(encoderPinA2)){  
  //   encoderCountB++;    
  // }
  // else{    
  //   encoderCountB--;
  //   Serial.println("------------------------------------------------------");

  // }
}

//Function to move motor
void moveMotor(int en, int inA, int inB, float u){
  //Maximum motor speed
  float speed = fabs(u); 
  if(speed > maxSpeed){
    speed = maxSpeed;
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

//Function to stop motor
void stopMotor(int inA, int inB)
{
  digitalWrite(inA, HIGH);
  digitalWrite(inB, HIGH); 
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
