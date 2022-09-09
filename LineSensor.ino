/* INCLUDES*/

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/**************************************************/


/* DECLARATIONS and INITIALIZATIONS */
int AN[8]  = {0};       //Storage for ANALOG values from the line Sensor
char result[50] = "";   //Store string vlaues of the analog inputs
char gyroVal[16];
int enL = 7;    //Left Enable
int enR = 2;    //Right Enable
// left Side
int RF = 3;
int RB = 4;

// LEFT Motor
int LF = 5;
int LB = 6;
int L_Speed,R_Speed;

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

void setup() {
  //Initialise the Serial Monitor
  Serial.begin(115200);
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #endif
    
  //Initialize motor pins
  pinMode(R_Speed, OUTPUT);
  pinMode(L_Speed, OUTPUT);
  pinMode(RB, OUTPUT);
  pinMode(RF, OUTPUT);
  pinMode(LF, OUTPUT);
  pinMode(LB, OUTPUT);
  R_Speed = 255;
  L_Speed = 255;

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
        attachInterrupt(0, dmpDataReady, RISING);
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

void loop() {
  
  updateLineValues();
  readGyro();
//  moveForward();
//  delay(5000);
//  moveStop();
//  delay(500);
//  moveBack();
//  delay(5000);
//  moveStop();
//  delay(500);
//  turnRight();
//  delay(3000);
//  turnLeft();
//  delay(3000);
}

/*This function is used to move motor 
*wheels forward with a maximum speed
*****************************************************************************/
void moveForward(){
    analogWrite(enR,R_Speed);
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);
    
    analogWrite(enL,L_Speed);  
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
}
void moveStop(){
    analogWrite(enR,0);
    digitalWrite(RF, LOW);
    digitalWrite(RB, LOW);
    
    analogWrite(enL,0);  
    digitalWrite(LF, LOW);
    digitalWrite(LB, LOW);
}
void moveBack(){
     analogWrite(enR,R_Speed);
    digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);
    
    analogWrite(enL,L_Speed);  
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);
}
void turnRight(){
    analogWrite(enR,150);
    digitalWrite(RF, LOW);
    digitalWrite(RB, HIGH);
    
    analogWrite(enL,L_Speed);  
    digitalWrite(LF, HIGH);
    digitalWrite(LB, LOW);
}
void turnLeft(){
     analogWrite(enR,R_Speed);
    digitalWrite(RF, HIGH);
    digitalWrite(RB, LOW);
    
    analogWrite(enL,150);  
    digitalWrite(LF, LOW);
    digitalWrite(LB, HIGH);
}



void readGyro(){
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

//         #ifdef OUTPUT_READABLE_EULER
//            // display Euler angles in degrees
//            mpu.dmpGetQuaternion(&q, fifoBuffer);
//            mpu.dmpGetEuler(euler, &q);
//            Serial.print("euler\t");
//            Serial.print(euler[0] * 180/M_PI);
//            Serial.print("\t");
//            Serial.print(euler[1] * 180/M_PI);
//            Serial.print("\t");
//            Serial.println(euler[2] * 180/M_PI);
//        #endif

        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            // display Euler angles in degrees
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
            
        #endif
    }
}
/*This function is used to obtain analog values
*from line sensor and store them in an array
*****************************************************************************/
void updateLineValues(){
  
  AN[0] = analogRead(A0);
  AN[1] = analogRead(A1);
  AN[2] = analogRead(A2);
  AN[3] = analogRead(A3);
  AN[4] = analogRead(A4);
  AN[5] = analogRead(A5);
  AN[6] = analogRead(A6);
  AN[7] = analogRead(A7);

    sprintf(result,"|%5d|%5d|%5d|%5d|%5d|%5d|%5d|%5d|",AN[0],AN[1],AN[2],AN[3],AN[4],AN[5],AN[6],AN[7]);
    Serial.println(result);
    //delay(100);
}
