#include <Adafruit_PWMServoDriver.h>
#include "config.h"

//DEFINE SERVO LIMITS
#define SERVO_MIN 400
#define SERVO_MAX 2425
#define STEP_DELAY 20
#define MAX_HEIGHT -90
#define MIN_HEIGHT 90
#define OPEN_CLAW -75
#define CLOSED_CLAW 0

//DEFINE SERVO POSITION IN PCA
#define BASE_SERVO 0
#define ELBOW_SERVO 1
#define HEIGHT_SERVO 2
#define CLAW_SERVO 3

//DEFINE PHYSICAL LENGTH
#define SEGMENT_1_LENGTH 9
#define SEGMENT_2_LENGTH 14

//DEFINE PCA ADDRESS
#define PCA9685_I2C_ADDRESS 0x40

//DEFINE CONTROL VARIABLES FOR SERVO LIMITATIONS
#define MIN_T1 -90
#define MAX_T1 90
#define TOLERANCE 1
#define MAX_T2 90
#define MIN_T2 -50

//DEFINE RADIANS TO DEGREE TRANSFORMATION AND VICE VERSA
#define RAD_TO_DEG 57.2974693618
#define DEG_TO_RAD 0.01745277777

extern Adafruit_PWMServoDriver pca9685;

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

        void kinematics_setup();

        bool isPositionReachable(float h, float theta1, float theta2);

        float theta_step = 2;
        float curr_theta[4] = {0, 90, -90, 0};
        float curr_pos[2] = {0,SEGMENT_1_LENGTH+SEGMENT_2_LENGTH};
        float target_theta[4] = {curr_theta[0], curr_theta[1], curr_theta[2], curr_theta[3]};
        int n_servos = 4;
        float desired_pos[2];

        static Adafruit_PWMServoDriver pca9685;
};
