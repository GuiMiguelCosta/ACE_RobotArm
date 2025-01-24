#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

#define SERVO_MIN 400  
#define SERVO_MAX 2425  

#define STEP_DELAY 30

#define MAX_HEIGHT -90
#define MIN_HEIGHT 90

#define OPEN_CLAW -45
#define CLOSED_CLAW 0

#define BASE_SERVO 0
#define ELBOW_SERVO 1
#define HEIGHT_SERVO 2
#define CLAW_SERVO 3

#define SEGMENT_1_LENGTH 9
#define SEGMENT_2_LENGTH 12

#define PCA9685_I2C_ADDRESS 0x40
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, Wire);


class Kinematics {

    public:

        int getPwmForAngle(int servo, int theta);
        void moveServoToAngle(int servo_num, float target_angle);
        void moveTwo(float theta1, float theta2);
        void OpenClaw();
        void CloseClaw();
        void goUp();
        void goDown();
        void pickUp();
        void dropDown();
        void moveToPos(float x, float y);
        int find_ServoDriver(int addr);
        /*
        int theta_step = 2;
        int curr_theta[4] = {0, 90, -90, 0};
        int target_theta[4] = {curr_theta[0], curr_theta[1], curr_theta[2], curr_theta[3]};
        int n_servos = 4;
        int desired_pos[2];*/
};