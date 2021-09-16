/*
 Name:		slc_vs.ino
 Created:	5/3/2020 12:58:21 PM
 Author:	aderh
 Note: This code is developed using Jeff Rowberg's library on MPU6050 development.
 Please check that out here: https://github.com/jrowberg/i2cdevlib
*/

#include "Arduino.h"
#include "Wire.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Servo.h"

MPU6050 accelgyro;

int16_t ax, ay, az;

int pitchAngle, rollAngle; // + 2 bytes

long startTime;
long endTime;
long deltaTime;

#define MPU_ADDR 0x68

#define DEBUG_LOOP_TIME
#define OUTPUT_USE_SERIAL
// #define OUTPUT_READABLE_ACCELGYRO
// #define OUTPUT_PITCH_ANGLE

#define LED_PIN 13
bool blinkState = false;

#define P_SERVO_ZERO 80
#define R_SERVO_ZERO 90 // Unknown, not tested
int ANGLE_LOW = P_SERVO_ZERO - 90;
int ANGLE_HIGH = P_SERVO_ZERO + 90;

Servo pServo; // Pitch Servo + 3 bytes program space
Servo rServo; // roll servo  + 3 bytes

void setup()
{
#ifdef OUTPUT_USE_SERIAL
    Serial.begin(115200);
#endif

    pServo.attach(11);
    pServo.write(P_SERVO_ZERO);
    rServo.attach(12);
    rServo.write(R_SERVO_ZERO);

    Wire.begin();
    Wire.setClock(400000);

    // initialize device
    accelgyro.initialize();
    accelgyro.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
    accelgyro.setDLPFMode(3);

    delay(40); // time delay for DLPF initial values
}

inline float atand(int y, int x)
{
    return atan2(y, x) * 57.29577795f;
}

void loop()
{
    // read raw accel/gyro measurements from device
#ifdef DEBUG_LOOP_TIME
    startTime = micros();
#endif

    // Get accelerometer data - 296us in fast i2C mode
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true);
    ax = (Wire.read() << 8 | Wire.read());
    ay = (Wire.read() << 8 | Wire.read());
    az = (Wire.read() << 8 | Wire.read());

#ifdef OUTPUT_READABLE_ACCELGYRO // display tab-separated accel/gyro x/y/z values -- from I2Cdevlib
    Serial.print("a/g:\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.println(az);
#endif

    pitchAngle = floor(-atand(ay, az)); // 192us
    rollAngle = floor(-atand(ax, az));
    pServo.write(P_SERVO_ZERO + pitchAngle); // 34us
    rServo.write(R_SERVO_ZERO + rollAngle);

#ifdef OUTPUT_PITCH_ANGLE
    Serial.print("p:");
    Serial.println(pitchAngle);
#endif

#ifdef DEBUG_LOOP_TIME
    Serial.print("dt:");
    Serial.println(micros() - startTime); // + 28us when on
#endif
}
