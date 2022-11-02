#include <Servo.h> // Include the servo library


/**************** GYRO BEGINNING *******************/
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif


// class default I2C address is 0x68
  MPU6050 mpu;
  #define OUTPUT_READABLE_YAWPITCHROLL
  #define OUTPUT_READABLE_EULER
 
   // MPU control/status vars
  bool dmpReady = false;  // set true if DMP init was successful
  uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
  uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
  uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
  uint16_t fifoCount;     // count of all bytes currently in FIFO
  uint8_t fifoBuffer[64]; // FIFO storage buffer

  // orientation/motion vars
  Quaternion q;           // [w, x, y, z]         quaternion container
  VectorInt16 aa;         // [x, y, z]            accel sensor measurements
  VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
  VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
  VectorFloat gravity;    // [x, y, z]            gravity vector
  float euler[3];         // [psi, theta, phi]    Euler angle container
  float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
  
  // ================================================================
  // ===               INTERRUPT DETECTION ROUTINE                ===
  // ================================================================
volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high

void dmpDataReady() {
    mpuInterrupt = true;
}
/*--------------------- GYRO ENDING ----------------------------**/


//**********5 Channel IR Sensor Connection**********//
#define ir1 8
#define ir2 9
#define ir3 10
#define ir4 11
#define ir5 12
//*************************************************//

#define L_PIN  44
#define R_PIN  46

int enL = 7;    //Left Enable
int enR = 2;    //Right Enable
// left Side
int RF = 4;
int RB = 3;

// LEFT Motor
int LF = 5;
int LB = 6;
int L_Speed,R_Speed,RS_Speed,LS_Speed;
int servo_speed,motor_speed;
int SR_turn_Speed,SL_turn_Speed;

bool s1 = 0;
bool s2 = 0;
bool s3 = 0;
bool s4 = 0;
bool s5 = 0;
  
Servo left_servo,right_servo; 

void setup() {
  Serial.begin(115200);
  pinMode(ir1, INPUT);
  pinMode(ir2, INPUT);
  pinMode(ir3, INPUT);
  pinMode(ir4, INPUT);
  pinMode(ir5, INPUT);

  //Initialize motor pins
  pinMode(R_Speed, OUTPUT);
  pinMode(L_Speed, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);

 servo_init();
 gyro_init();


 servo_speed = 15;
 motor_speed = 150;
 
  R_Speed = motor_speed;
  L_Speed = motor_speed;
// RS_Speed = 90-15; 
// LS_Speed = 90+15;
////
// SR_turn_Speed = 90-10;
// SL_turn_Speed = 90+10;
//  
//robotForward();
//  delay(5000);
//  robotStop();
// delay(5000);
}

void loop() {
  s1 = !digitalRead(ir1);  //Left Most Sensor
  s2 = !digitalRead(ir2);  //Left Sensor
  s3 = !digitalRead(ir3);  //Middle Sensor
  s4 = !digitalRead(ir4);  //Right Sensor
  s5 = !digitalRead(ir5);  //Right Most Sensor

  Serial.print(s1);
  Serial.print(s2);
  Serial.print(s3);
  Serial.print(s4);
  Serial.print(s5);
  Serial.println();
//readGyro();
 // delay(100);
   robotForward();
//    delay(5000);
//    robotStop();
//    delay(2000);
//    robotLeft();
//    delay(5000);
//    robotStop();
//    delay(2000);
//    robotRight();
//    delay(5000);
//    robotStop();
//    delay(2000);
//servoForward(); //tested: WORKING
//servoLeft();    //tested: WORKING
//servoRight();   //tested: WORKING
//motorForward(); //tested: WORKING
//motorRight();   //tested: WORKING
//motorLeft();    //tested: WORKING

if (!s1 && !s2 && s3 && !s4 && !s5){
  robotForward();
  delay(100);
}

//*******************  left movements *******************
if (s1 && s2 && s3 && s4 && !s5) {
  robotStop();
  delay(250);
  robotLeft();
  delay(250);
  
}
if (s1 && s2 && s3 && !s4 && !s5) {

  robotStop();
  delay(100);
  robotLeft();
  delay(100);
}
if (s1 && s2 && !s3 && !s4 && !s5) {

  robotStop();
  delay(100);
  robotLeft();
  delay(100);
}

if (s1 && s2 && s3 && !s4 && !s5) {
  robotStop();
  delay(100);
  robotLeft();
  delay(100);
}
//--------------------------------------------

//**************   Right Movements *********************
//if (!s1 && !s2 && !s3 && !s4 && s5) {
//  robotStop();
//  delay(250);
//  robotRight();
//}
if (!s1 && !s3 &&  s5) {
  robotStop();
  delay(100);
  robotRight();
  delay(100);
}

//if (!s1 && !s2 && !s3 && s4 && s5) {
//  robotStop();
//  delay(250);
//  robotRight();
//}

if (!s1 && !s2 && s3 && s4 && s5) {
  robotStop();
  delay(100);
  motorRight();
  delay(100);
}
if (!s1 && s2 && s3 && s4 && s5) {
  robotStop();
  delay(100);
  robotRight();
  delay(100);
}

//------------------------------------------------
  if (!s1 && !s2 && s3 && !s4 && !s5){
      motorForward();
      servoForward();
      delay(250);
//    
  }
   if (!s1 && !s2 && !s3 && !s4 && !s5){

      motorStop();
      servoStop();
  }
//    if (s1 && s2 && s3 && s4 && s5){
//
//      motorStop();
//      servoStop();
//  }
}

void servo_init(){
   left_servo.attach(L_PIN);  // attaches the servo on pin 9 to the servo object
  right_servo.attach(R_PIN);  // attaches the servo on pin 10 to the servo object
  left_servo.write(90);   // stop the left motor
  right_servo.write(90);  //stop the right motor

}
void servoForward(){
  //Tested: WORKING GOOD to MOVE FORWARD
  left_servo.write(90+servo_speed);
  right_servo.write(90-servo_speed);
}

void  servoStop(){
 left_servo.write(90);
 right_servo.write(90);
}

void servoLeft(){
  //Tested: WORKING GOOD to TURN LEFT
  left_servo.write(90 - servo_speed);
 right_servo.write(90 - servo_speed);
 delay(250);
}
void servoRight(){
  //Tested: WORKING GOOD to TURN RIGHT
  left_servo.write(90 + servo_speed);
 right_servo.write(90 + servo_speed);
 delay(250);
}

void motorForward(){
//  Tested: WORKING GOOD for moving forward
    analogWrite(enR,motor_speed);
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);
    
    analogWrite(enL,motor_speed);  
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
}

void motorStop(){
    analogWrite(enR,0);
    digitalWrite(RF, LOW);
    digitalWrite(RB, LOW);
    
    analogWrite(enL,0);  
    digitalWrite(LF, LOW);
    digitalWrite(LB, LOW);
}
void moveBack(){
  
     analogWrite(enR,motor_speed);
    digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);
    
    analogWrite(enL,motor_speed);  
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);
    
}

void motorRight(){
   
     analogWrite(enR,motor_speed);
     digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);
    
    analogWrite(enL,motor_speed);  
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
}

void motorLeft(){
  
     analogWrite(enR,motor_speed);
     digitalWrite(RF, HIGH);
     digitalWrite(RB, LOW);
    
     analogWrite(enL,motor_speed);  
     digitalWrite(LF, LOW);
     digitalWrite(LB, HIGH);
}

void robotForward(){
  motorForward();
  servoForward();
}
void robotLeft(){
  motorLeft();
  servoLeft();
}
void robotRight(){
  motorRight();
  servoRight();
}

void robotStop(){
  motorStop();
  servoStop();
}

void gyro_init(){
  
     #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

   
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));


    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void readGyro(){
 // if programming failed, don't try to do anything
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
       
    }

    // reset interrupt flag and get INT_STATUS byte
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();

    // get current FIFO count
    fifoCount = mpu.getFIFOCount();

    // check for overflow (this should never happen unless our code is too inefficient)
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        // reset so we can continue cleanly
        mpu.resetFIFO();
        //Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
    } else if (mpuIntStatus & 0x02) {
        // wait for correct available data length, should be a VERY short wait
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // read a packet from FIFO
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        
        // track FIFO count here in case there is > 1 packet available
        // (this lets us immediately read more without waiting for an interrupt)
        fifoCount -= packetSize;


        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print((ypr[0] * 180/M_PI)-31.00);
            Serial.print("\t");
            Serial.print((ypr[1] * 180/M_PI)+3.00);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif
    }
}
