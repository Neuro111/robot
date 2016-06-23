#include <Wire.h>
#include <Encoder.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "motorcontroller.h"
#include "regulatorPID.h"

MPU6050 mpu; // adres 0x68 (AD0 LOW)


#define dirR 4
#define pwmR 5
#define pwmL 6
#define dirL 7

Encoder encLeft(2, 9);
Encoder encRight(3, 8);

double encPosition = 0;
double encOrientation = 0;
double encRotation = 0;
double encVelocity = 0;
double encVelocityFiltered = 0;
double filterLPF = 0.08;
long lastTime = 0;

MotorController motor(pwmR, dirR, pwmL, dirL);

// BALANCE PID
double requestedTheta = 0;
double balanceKp = 15;
double balanceKi = 100;
double balanceKd = 0.5;
double angleTheta, outputPWM;
int minAbsSpeed = 0;

regulatorPID balancePID(&angleTheta, &outputPWM, &requestedTheta, balanceKp, balanceKi, balanceKd, REVERSE);

// Velocity PID
double requestedVelocity = 0;
double velocityKp = 20;
double velocityKi = 15;
double velocityKd = 0;
double velocity = 0;


regulatorPID velocityPID(&velocity, &requestedTheta, &requestedVelocity, velocityKp, velocityKi, velocityKd, DIRECT);

// Position PID
double requestedPosition = 0;
double positionKp = 1;
double positionKi = 0;
double positionKd = 0;
double position = 0;


regulatorPID positionPID(&position, &requestedVelocity, &requestedPosition, positionKp, positionKi, positionKd, DIRECT);


// Rotation PID
double requestedRotation = 0;
double rotationKp = 2;
double rotationKi = 0;
double rotationKd = 0;
double outputTurn;

regulatorPID rotationPID(&encRotation, &outputTurn, &requestedRotation, rotationKp, rotationKi, rotationKd, DIRECT);


// TURN PID
double requestedTurn = 0;
double turnKp = 0;
double turnKi = 0;
double turnKd = 0;
//double outputTurn;

regulatorPID turnPID(&encOrientation, &requestedRotation, &requestedTurn, turnKp, turnKi, turnKd, DIRECT);


#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

int16_t ax, ay, az;
int16_t gx, gy, gz;

bool oneTime = false;


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}

void setup() {

  Wire.begin();
  TWBR = 24;

  Serial.begin(115200);


  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  delay(2);
  //  // wait for ready
  //  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  //  while (Serial.available() && Serial.read()); // empty buffer
  //  while (!Serial.available());                 // wait for data
  //  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  //  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_250);
  //  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);

  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(25);
  mpu.setYGyroOffset(12);
  mpu.setZGyroOffset(-9);
  //-4503  -1159 830 28  8 -10

  mpu.setXAccelOffset(-4503);
  mpu.setYAccelOffset(-1159);
  mpu.setZAccelOffset(830);
  //  Your offsets:  -4496 -1171 828 26  5 -10
  //
  //Data is printed as: acelX acelY acelZ giroX giroY giroZ

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(10), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();


    balancePID.SetMode(AUTOMATIC);
    balancePID.SetSampleTime(3);
    balancePID.SetOutputLimits(-255, 255);
    balancePID.SetITermLimits(-255, 255);

    velocityPID.SetMode(AUTOMATIC);
    velocityPID.SetSampleTime(3);
    velocityPID.SetOutputLimits(-30, 30); // maksymalny kąt, którym robot będzie hamował
    velocityPID.SetITermLimits(-3, 3);  // maksymalny kąt, w którym robot będzie próbował balansować przy zmienionym obciążeniu
        
    positionPID.SetMode(AUTOMATIC);
    positionPID.SetSampleTime(100);
    positionPID.SetOutputLimits(-0.2, 0.2); // maksymalna prędkość którą robot będzie wracał do pozycji
    positionPID.SetITermLimits(-0.2, 0.2);

    turnPID.SetMode(AUTOMATIC);
    turnPID.SetSampleTime(100);
    turnPID.SetOutputLimits(-50, 50); 
    turnPID.SetITermLimits(-50, 50);

    rotationPID.SetMode(AUTOMATIC);
    rotationPID.SetSampleTime(10);
    rotationPID.SetOutputLimits(-50, 50); 
    rotationPID.SetITermLimits(-50, 50);
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));

  }
  // configure LED for output

  lastTime = millis();
  //  long actTimeMillis = millis();
  // int dtMillis = actTimeMillis - lastTimeMillis;

  long newLeft, newRight;
  newLeft = encLeft.read();
  newRight = encRight.read();
  encPosition = (double)(newLeft + newRight) / 2400; //obroty
  encOrientation = (double)(newLeft - newRight) * 360 / 3840;

  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
//  while (!mpuInterrupt && fifoCount < packetSize) {
    if (oneTime) {
      oneTime = false;



      /* Print Data */
#if 0 // Set to 1 to activate
      mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
      Serial.print(ax); Serial.print("\t");
      Serial.print(ay); Serial.print("\t");
      Serial.print(az); Serial.print("\t");
      Serial.print(gx); Serial.print("\t");
      Serial.print(gy); Serial.print("\t");
      Serial.print(gz); Serial.print("\t");
      // Serial.print("\t");
#endif

    }
    if (abs(ypr[1]) > 0.6)
    {
      // motor.stopMoving();
      angleTheta = requestedTheta;
      motor.move(0);
    }
    else {
      angleTheta = ypr[1] * 180 / M_PI;
      positionPID.Compute();
      rotationPID.Compute();
      turnPID.Compute();
      velocityPID.Compute();
      balancePID.Compute();

      motor.moveTurn(outputPWM, outputTurn);
    }

    long actTime = micros();
    if ((actTime - lastTime) > 5)
    {
      int dt = actTime - lastTime;
      lastTime = actTime;

      long newLeft, newRight;
      newLeft = encLeft.read();
      newRight = encRight.read();
      double newPosition = (double)(newLeft + newRight) / 2400;   // obroty kół
//      Serial.print(newLeft);
//      Serial.print("\t");
//      Serial.println(newRight);
      encVelocity = (double)(newPosition - encPosition) * 188500 / dt;  // obr/min: (double)(newPosition - encPosition) * 60000000 / dt;
      encVelocityFiltered = encVelocityFiltered * (1 - filterLPF) + encVelocity * filterLPF;

      double newOrientation = (double)(newLeft - newRight) * 360 / 3718; //3840; kąt w stopniach
      double encRotationRAW = (double)(newOrientation - encOrientation) * 1000000/dt;  //stopnie na sekundę

    //  double rotationFilter = encRotation
      encRotation = encRotation * (1- 0.01) + encRotationRAW * 0.01;

      position = (double) newPosition * 0.1885;
      velocity = encVelocityFiltered;
      encPosition = newPosition;
      encOrientation = newOrientation;
    }


    if (Serial.available() )
    {
      char inChar = (char)Serial.read();
      char tab = (char)Serial.read();
      float data = Serial.parseFloat();
      char newLine = (char)Serial.read();

      switch (inChar)
      {
        case 'u':
        {
          double reqSpeed = data;
        //  positionPID.SetOutputLimits(-itermlim, itermlim);
          requestedVelocity = reqSpeed /100;
        }
         // requestedTheta = data;
          break;
        case 'o':
        {
          double reqRotation = data;
          requestedRotation = reqRotation;
        //  positionPID.SetOutputLimits(-itermlim, itermlim);
         // requestedVelocity = reqSpeed /100;
        }
         // requestedTheta = data;
          break;
          case 'a':
        {
          bool checked = data;
          positionPID.SetMode(checked? AUTOMATIC:MANUAL);
        }
         // requestedTheta = data;
          break;
          case 'g':
        {
          bool checked = data;
          turnPID.SetMode(checked? AUTOMATIC:MANUAL);
        }
         // requestedTheta = data;
          break;
        case 's':
        {
          double itermlim = data;
        //  positionPID.SetOutputLimits(-itermlim, itermlim);
          positionPID.SetOutputLimits(-itermlim, itermlim);
        }
         // requestedTheta = data;
          break;
        case 'p':
          balanceKp = data;
          balancePID.SetTunings(balanceKp, balanceKi, balanceKd);
          break;
        case 'i':
          balanceKi = data;
          balancePID.SetTunings(balanceKp, balanceKi, balanceKd);
          break;
        case 'd':
          balanceKd = data;
          balancePID.SetTunings(balanceKp, balanceKi, balanceKd);
          break;
        case 'h':
          turnKp = data;
          turnPID.SetTunings(turnKp, turnKi, turnKd);
          break;
        case 'j':
          turnKi = data;
          turnPID.SetTunings(turnKp, turnKi, turnKd);
          break;
        case 'k':
          turnKd = data;
          turnPID.SetTunings(turnKp, turnKi, turnKd);
          break;
        case 'l':
          rotationKp = data;
          rotationPID.SetTunings(rotationKp, rotationKi, rotationKd);
          break;
        case 'z':
          rotationKi = data;
          rotationPID.SetTunings(rotationKp, rotationKi, rotationKd);
          break;
        case 'x':
          rotationKd = data;
          rotationPID.SetTunings(rotationKp, rotationKi, rotationKd);
          break;
        case 'm':
        {
          double itermlim = data;
          velocityPID.SetITermLimits(-itermlim, itermlim);
        }
         // minAbsSpeed = (int)data;
          break;
        case 'f':
          filterLPF = data;
          break;
        case 'q':
          velocityKp = data;
          velocityPID.SetTunings(velocityKp, velocityKi, velocityKd);
          break;
        case 'w':
          velocityKi = data;
          velocityPID.SetTunings(velocityKp, velocityKi, velocityKd);
          break;
        case 'e':
          velocityKd = data;
          velocityPID.SetTunings(velocityKp, velocityKi, velocityKd);
          break;
        case 'r':
          positionKp = data;
          positionPID.SetTunings(positionKp, positionKi, positionKd);
          break;
        case 't':
          positionKi = data;
          positionPID.SetTunings(positionKp, positionKi, positionKd);
          break;
        case 'y':
          positionKd = data;
          positionPID.SetTunings(positionKp, positionKi, positionKd);
          break;
        default:
          break;
      }
      //      Serial.print(inChar);
      //      Serial.print(tab);
      //      Serial.print(data);
      //      Serial.print(newLine);
    }
    // other program behavior stuff here
    // .
    // .
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
    // .
    // .
    // .

 // }
  oneTime = true;
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);

    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

#if 0//OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees

    Serial.print("\r\nypr\t");
    Serial.print(ypr[0] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI);
    Serial.print("\t");
    Serial.print(ypr[2] * 180 / M_PI);
    Serial.print("\t\r\n");
#endif


#if 0
    Serial.print("ypr"); Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI); Serial.print("\t");
    Serial.print(requestedTheta); Serial.print("\t");
    Serial.print(angleTheta); Serial.print("\t");
    Serial.print(outputPWM); Serial.println("\t");
#endif

#if 0
    Serial.print("ypr"); Serial.print("\t");
    Serial.print(ypr[0] * 180 / M_PI); Serial.print("\t");
    Serial.print(encOrientation); Serial.print("\t");
    Serial.print(encVelocity); Serial.print("\t");
    Serial.print(encVelocityFiltered); Serial.println("\t");
#endif


#if 1
    Serial.print("ypr"); Serial.print("\t");
    Serial.print(ypr[1] * 180 / M_PI); Serial.print("\t");
    Serial.print(requestedTheta); Serial.print("\t");
    Serial.print(requestedRotation); Serial.print("\t");
    Serial.print(encRotation); Serial.println("\t");
    //&angleTheta, &outputPWM, &requestedTheta
#endif
    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);

    // delay(2);
  }
}
