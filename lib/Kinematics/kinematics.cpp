#include <kinematics.h>

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

int theta_step = 2;
int curr_theta[4] = {0, 90, -90, 0};
int target_theta[4] = {curr_theta[0], curr_theta[1], curr_theta[2], curr_theta[3]};
int n_servos = 4;

int desired_pos[2];

int getPwmForAngle(int servo, int theta) 
{
    if (theta <= 90 && theta >= -90)
        if (servo == 0) return ((theta + 90) * 1950 / 180 + 350);
        else return ((theta + 90) * 1950 / 180 + 450);
    else return getPwmForAngle(0, 0);
}

void moveServoToAngle(int servo_num, float target_angle) 
{
    Serial.print("Servo: ");
    Serial.println(servo_num);
    Serial.print("Current angle: ");
    Serial.println(curr_theta[servo_num]);
    Serial.print("Target angle: ");
    Serial.println(target_angle);

    int theta = (int)target_angle;

    if (curr_theta[servo_num] < theta) {
        for (int i = curr_theta[servo_num]; i <= theta; i += theta_step) 
        {
            float pwm = getPwmForAngle(servo_num, i);
            pca9685.writeMicroseconds(servo_num, pwm);
            delay(STEP_DELAY/2);
        }
    } else {
        for (int i = curr_theta[servo_num]; i >= theta; i -= theta_step) 
        {
            float pwm = getPwmForAngle(servo_num, i);
            pca9685.writeMicroseconds(servo_num, pwm);
            delay(STEP_DELAY/2);
        }
    }
    curr_theta[servo_num] = theta;
    Serial.println("Reached target angle");
}

void moveTwo(float theta1, float theta2) 
{
    int base_start = curr_theta[BASE_SERVO];
    int elbow_start = curr_theta[ELBOW_SERVO];
    int base_target = (int)theta1;
    int elbow_target = (int)theta2;

    int base_steps = abs(base_target - base_start) / theta_step;
    int elbow_steps = abs(elbow_target - elbow_start) / theta_step;
    int max_steps = max(base_steps, elbow_steps);

    for (int step = 0; step <= max_steps; step++) 
    {
        int base_angle = base_start + (step * (base_target - base_start)) / max_steps;
        int elbow_angle = elbow_start + (step * (elbow_target - elbow_start)) / max_steps;

        float base_pwm = getPwmForAngle(BASE_SERVO, base_angle);
        float elbow_pwm = getPwmForAngle(ELBOW_SERVO, elbow_angle);
        pca9685.writeMicroseconds(BASE_SERVO, base_pwm);
        pca9685.writeMicroseconds(ELBOW_SERVO, elbow_pwm);

        delay(STEP_DELAY); 
    }

    curr_theta[BASE_SERVO] = base_target;
    curr_theta[ELBOW_SERVO] = elbow_target;

    Serial.println("Servos atingiram os ângulos alvo:");
    Serial.print("BASE_SERVO: ");
    Serial.println(base_target);
    Serial.print("ELBOW_SERVO: ");
    Serial.println(elbow_target);
}

void OpenClaw()
{
    moveServoToAngle(CLAW_SERVO,OPEN_CLAW);
}

void CloseClaw()
{
    moveServoToAngle(CLAW_SERVO,CLOSED_CLAW);
}

void goDown()
{
    moveServoToAngle(HEIGHT_SERVO,MIN_HEIGHT);
}

void goUp()
{
    moveServoToAngle(HEIGHT_SERVO,MAX_HEIGHT);
}

void pickUp()
{
    OpenClaw();
    goDown();
    CloseClaw();
    goUp();
}
void dropDown()
{
    goDown();
    OpenClaw();
    goUp();
    CloseClaw();
}

void moveToPos(float x, float y)
{
    const float rad_to_deg = 180.0 / 3.1415;

    float H = sqrt(x * x + y * y);

    if (H > SEGMENT_1_LENGTH + SEGMENT_2_LENGTH || H <= 0) 
    {
        Serial.println("Target position is out of reach!");
        return;
    }

    float cosTheta2 = (-H * H + SEGMENT_1_LENGTH * SEGMENT_1_LENGTH + SEGMENT_2_LENGTH * SEGMENT_2_LENGTH) / (2 * SEGMENT_1_LENGTH * SEGMENT_2_LENGTH);
    float theta2 = acos(cosTheta2) * rad_to_deg; 

    float angleToTarget = atan2(y, x) * rad_to_deg; 
    float cosAlpha = (H * H + SEGMENT_1_LENGTH * SEGMENT_1_LENGTH - SEGMENT_2_LENGTH * SEGMENT_2_LENGTH) / (2 * H * SEGMENT_1_LENGTH);
    float alpha = acos(cosAlpha) * rad_to_deg; 

    float theta1 = angleToTarget + alpha;

    float thetaS1 = theta1 - 90;
    float thetaS2 = theta2 - 90;

    if(thetaS1<-90) 
    {
        thetaS1 = -90;
    }
    if(thetaS2<-90) 
    {
        thetaS2 = -90;
    }
    if(thetaS1>90) 
    {
        thetaS1 = 90;
    }
    if(thetaS2>90) 
    {
        thetaS2 = 90;
    }

    // Move servos
    moveTwo(thetaS1,thetaS2);
}

int find_ServoDriver(int addr) 
{
    Wire.beginTransmission(addr);
    Wire.write(PCA9685_MODE1);
    Wire.write(MODE1_RESTART);

    int err = Wire.endTransmission();
    return !err;
}