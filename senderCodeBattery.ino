#include <SPI.h>
#include <LoRa.h>
#include <cstdlib> 


#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
 * ========================================================================= */

/* =========================================================================
   NOTE: Arduino v1.0.1 with the Leonardo board generates a compile error
   when using Serial.write(buf, len). The Teapot output uses this method.
   The solution requires a modification to the Arduino USBAPI.h file, which
   is fortunately simple, but annoying. This will be fixed in the next IDE
   release. For more info, see these links:

   http://arduino.cc/forum/index.php/topic,109987.0.html
   http://code.google.com/p/arduino/issues/detail?id=958
 * ========================================================================= */



// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
//#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
//#define OUTPUT_TEAPOT



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
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };



// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

float sensorValue;
float sensorValueR;
int digitalValue;

#define MQ2pin (0)

float sensorValue2; // assign variable to store sensor value
float x = 0.0;
float y = 0.0;
float z = 0.0; 

float xprev = 0.0;
float yprev = 0.0;
float zprev = 0.0; 

float xdiff = 0.0;
float ydiff = 0.0;
float zdiff = 0.0; 

int changeCO;
int changeX;
int changeY;
int changeZ;

bool firstRun = false;
void setup() {
    pinMode(4, OUTPUT);
    pinMode(5, OUTPUT);
    pinMode(3, OUTPUT);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        //TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    // (115200 chosen because it is required for Teapot Demo output, but it's
    // really up to you depending on your project)
    //Serial.begin(115200);
   // Serial.begin(20000000);
   // while (!Serial); // wait for Leonardo enumeration, others continue immediately

    // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3v or Ardunio
    // Pro Mini running at 3.3v, cannot handle this baud rate reliably due to
    // the baud timing being too misaligned with processor ticks. You must use
    // 38400 or slower in these cases, or use some kind of external separate
    // crystal solution for the UART timer.

    // initialize device
    //Serial.println(F("Initializing I2C devices..."));
    //Serial.println("LoRa SENDER FERN DONT BURN");
    mpu.initialize();

    // verify connection
    //Serial.println(F("Testing device connections..."));
    //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for ready
   // Serial.println(F("\nSend any character to begin DMP programming and demo: "));//
    //while (Serial.available() && Serial.read()); // empty buffer
    //while (!Serial.available());                 // wait for data
    //while (Serial.available() && Serial.read()); // empty buffer again

    // load and configure the DMP
    //Serial.println(F("Initializing DMP..."));
    
    devStatus = mpu.dmpInitialize();

    // supply your own gyro offsets here, scaled for min sensitivity
    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    // make sure it worked (returns 0 if so)
    if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        //Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        // enable Arduino interrupt detection
        //Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
        attachInterrupt(0, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        // set our DMP Ready flag so the main loop() function knows it's okay to use it
      //  Serial.println(F("DMP ready! Waiting 30 sec for sensor warmup"));
        dmpReady = true;

        // get expected DMP packet size for later comparison
        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
       // Serial.print(F("DMP Initialization failed (code "));
        //Serial.print(devStatus);
        //Serial.println(F(")"));
    }

    // configure LED for output
    pinMode(LED_PIN, OUTPUT);
    pinMode(2, INPUT);

    if (!LoRa.begin(915E6)) {
      //Serial.println("Starting LoRa failed!");
      while (1);
    }
    delay(30000); //let gyro stablize and air qual. sensors warm up
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
    // if programming failed, don't try to do anything
  //MQ135
    sensorValue = analogRead(2); // read analog input pin 0
    //digitalValue = digitalRead(2);
    sensorValueR = round(sensorValue * 100.0) / 100.0;
    //Serial.println(digitalValue, DEC);
    //delay(1000); // wait 100ms for next reading
  
  // MQ2
  
    sensorValue2 = analogRead(MQ2pin); // read analogue input pin 0
    //Serial.print("Sensor Value 2: ");
    
    //delay(2000); // wait 2 seconds for the next reading
  
    if (!dmpReady) return;

    // wait for MPU interrupt or extra packet(s) available
    while (!mpuInterrupt && fifoCount < packetSize) {
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
            //Serial.print("ypr\t");
            x = (ypr[0] * 180/M_PI);
            //Serial.println(x);
            //Serial.print(" ");
            //Serial.print("\t");
            y = (ypr[1] * 180/M_PI);
            //Serial.print(ypr[1] * 180/M_PI);
            //Serial.print("\t");
            z = (ypr[2] * 180/M_PI);
            //Serial.println(ypr[2] * 180/M_PI);
            if (packetSize) {
                // received a packet
              //Serial.print("Sending packet: ");
              //Serial.println(" ");
              //Serial.println("next:");
              //Serial.println(sensorValueR); // prints the value read
              //Serial.print(sensorValue2); //(sensor value is displayed on the monitor)
              //Serial.println(" ");
              //Serial.println(x);
              //Serial.println(y);
              //Serial.println(z);

              // send packet
              LoRa.beginPacket();
              LoRa.print("<Pole 1, ");
              LoRa.print(x);
              LoRa.print(", ");
              LoRa.print(y);
              LoRa.print(", ");
              LoRa.print(z);
              LoRa.print(", ");
              //LoRa.print(sensorValueR);
              //LoRa.print(", ");
              LoRa.print(sensorValue2);
              LoRa.print(",");
              LoRa.print(changeCO);
              LoRa.print(",");
              LoRa.print(changeX);
              LoRa.print(",");
              LoRa.print(changeY);
              LoRa.print(",");
              LoRa.print(changeZ);
              LoRa.print(",");
              LoRa.print(">");
              LoRa.endPacket();
              
              LoRa.print(">");
              LoRa.endPacket();
              /*digitalWrite(3, HIGH);
              digitalWrite(4, HIGH);   
              digitalWrite(5, HIGH);   
              delay (400);
              digitalWrite(3, LOW);
              digitalWrite(4, LOW);   
              digitalWrite(5, LOW); */
              delay(1000);
            }
        changeCO = 0;
        changeX = 0;
        changeY = 0;
        changeZ = 0;
        xdiff = (xprev-x);
        ydiff = (yprev-y);
        zdiff = (yprev-y);
        
        #endif
        if (((sensorValue2 > 200) || (abs(xdiff) > 30) ||(abs(ydiff) > 30) || (abs(zdiff) >30)) && firstRun)  {
            for (int i=0; i<3; i++) {
              digitalWrite(3, HIGH);
              digitalWrite(4, HIGH);   
              digitalWrite(5, HIGH); 
              delay(200);
              digitalWrite(3, LOW);
              digitalWrite(4, LOW);   
              digitalWrite(5, LOW);
              delay(200); 
            }
          }

    if (sensorValue2 > 100 && firstRun) {
      changeCO = 1;
    }
    if ((abs(xdiff) > 30) && firstRun) {
      changeX = 1;
    }
    if ((abs(ydiff) > 30) && firstRun) {
      changeY = 1;
    }
    if ((abs(zdiff) > 30) && firstRun) {
      changeZ = 1;
    }
        // blink LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
        xprev = x;
        yprev = y;
        zprev = z;
        firstRun = true;
        /* 
        Serial.println("----prev");
        Serial.println(xdiff);
        Serial.println(ydiff);
        Serial.println(zdiff);
        */
        
    }
}
