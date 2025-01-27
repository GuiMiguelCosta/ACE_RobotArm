#include "kinematics.h"

Adafruit_PWMServoDriver Kinematics::pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS);


int Kinematics::getPwmForAngle(int servo, int theta) 
{
    if (theta <= 90 && theta >= -90)
        if (servo == 0) return ((theta + 90) * 1950 / 180 + 350);
        else return ((theta + 90) * 1950 / 180 + 450);
    else return getPwmForAngle(0, 0);
}

void Kinematics::moveServoToAngle(int servo_num, float target_angle) 
{
    int theta = (int)target_angle;

    unsigned long lastTime = millis();

    if (curr_theta[servo_num] < theta) {
        for (int i = curr_theta[servo_num]; i <= theta; i += theta_step) {
            float pwm = getPwmForAngle(servo_num, i);
            pca9685.writeMicroseconds(servo_num, pwm);
            
            while (millis() - lastTime < STEP_DELAY) {
            }
            lastTime = millis();
        }
    } else {
        for (int i = curr_theta[servo_num]; i >= theta; i -= theta_step) {
            float pwm = getPwmForAngle(servo_num, i);
            pca9685.writeMicroseconds(servo_num, pwm);
            
            while (millis() - lastTime < STEP_DELAY) {
             
            }
            lastTime = millis();
        }
    }
    curr_theta[servo_num] = theta;
}

void Kinematics::moveTwo(float theta1, float theta2) 
{
    int base_start = curr_theta[BASE_SERVO];
    int elbow_start = curr_theta[ELBOW_SERVO];
    int base_target = (int)theta1;
    int elbow_target = (int)theta2;

    int base_steps = abs(base_target - base_start) / theta_step;
    int elbow_steps = abs(elbow_target - elbow_start) / theta_step;
    int max_steps = max(base_steps, elbow_steps);

    unsigned long startMillis = millis(); 
    unsigned long elapsedMillis = 0;
    int step = 0;

    while (step <= max_steps) {
        elapsedMillis = millis() - startMillis; 

        if (elapsedMillis >= (long unsigned int)(step * STEP_DELAY)) {
            int base_angle = base_start + (step * (base_target - base_start)) / max_steps;
            int elbow_angle = elbow_start + (step * (elbow_target - elbow_start)) / max_steps;

            float base_pwm = getPwmForAngle(BASE_SERVO, base_angle);
            float elbow_pwm = getPwmForAngle(ELBOW_SERVO, elbow_angle);
            pca9685.writeMicroseconds(BASE_SERVO, base_pwm);
            pca9685.writeMicroseconds(ELBOW_SERVO, elbow_pwm);

            step++;  
        }
    }

    curr_theta[BASE_SERVO] = base_target;
    curr_theta[ELBOW_SERVO] = elbow_target;
}

void Kinematics::OpenClaw()
{
    moveServoToAngle(CLAW_SERVO,OPEN_CLAW);
}

void Kinematics::CloseClaw()
{
    moveServoToAngle(CLAW_SERVO,CLOSED_CLAW);
}

void Kinematics::goDown()
{
    moveServoToAngle(HEIGHT_SERVO,MIN_HEIGHT);
}

void Kinematics::goUp()
{
    moveServoToAngle(HEIGHT_SERVO,MAX_HEIGHT);
}

void Kinematics::pickUp()
{
    OpenClaw();
    goDown();
    CloseClaw();
    goUp();
}

void Kinematics::dropDown()
{
    goDown();
    OpenClaw();
    goUp();
    CloseClaw();
}

void Kinematics::moveToPos(float x, float y)
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


    // Move servos
    if(isPositionReachable(H,thetaS1,thetaS2)) 
    {
        moveTwo(thetaS1,thetaS2);
        curr_pos[0] = desired_pos[0];
        curr_pos[1] = desired_pos[1];
    }
    else Serial.println("Position not reachable due to physical constraints");
}

int Kinematics::find_ServoDriver(int addr) 
{
    Wire.beginTransmission(addr);
    Wire.write(PCA9685_MODE1);
    Wire.write(MODE1_RESTART);

    int err = Wire.endTransmission();
    return !err;
}

void Kinematics::kinematics_setup()
{   
    while (!find_ServoDriver(PCA9685_I2C_ADDRESS)) {
        Serial.println("No PCA9685 found ... check your connections");
        delay(200);
    }
    delay(10);
    Serial.println("Found PCA9685");

    pca9685.begin();
    pca9685.setPWMFreq(50);

    moveTwo(0,90);
    goUp();
    CloseClaw();
}

bool Kinematics::isPositionReachable(float h, float theta1, float theta2)
{
    if(h<=0 || h>(SEGMENT_1_LENGTH+SEGMENT_2_LENGTH+TOLERANCE))
    {
        Serial.println("Not reachable due to Distance");
        return false;
    }
    if(theta1>(MAX_T1+TOLERANCE) || theta1<(MIN_T1-TOLERANCE))
    {
        Serial.println("Not reachable due to Theta1");
        return false;
    }
    if(theta2>(MAX_T2+TOLERANCE) || theta2<(MIN_T2-TOLERANCE))
    {
        Serial.println("Not reachable due to Theta2");
        return false;
    }
    return true;
}