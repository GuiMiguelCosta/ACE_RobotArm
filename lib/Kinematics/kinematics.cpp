#include "kinematics.h"

Adafruit_PWMServoDriver Kinematics::pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS);

/*---------------------------------------------------------------------------------------------------------------
 * Method that computes a custom PWM signal for each servo based on it's calibration and the desired angle
 * @param servo, the servo
 * @param theta, the desired angle
 * @return the pwm value for that angle for that servo
 ---------------------------------------------------------------------------------------------------------------*/
int Kinematics::getPwmForAngle(int servo, int theta) 
{
    if (theta <= 90 && theta >= -90)
        if (servo == 0) return ((theta + 90) * 1950 / 180 + 350);
        else return ((theta + 90) * 1950 / 180 + 450);
    else return getPwmForAngle(0, 0);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that moves a servo to a desired angle
 * @param servo_num, the servo
 * @param target_angle, the desired angle
 ---------------------------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------------------------
 * Method that attempts the movement of both the base and elbow servos at the same time
 * @param theta1, the angle for the base servo
 * @param theta2, the angle for the elbow servo
 ---------------------------------------------------------------------------------------------------------------*/
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

/*
 * Method that opens the claw servo to a defined angle
 */
void Kinematics::OpenClaw()
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Opening claw");
    Serial.println("--------------------------------------------------");
    #endif
    moveServoToAngle(CLAW_SERVO,OPEN_CLAW);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that closes the claw servo to a defined angle
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::CloseClaw()
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Closing claw");
    Serial.println("--------------------------------------------------");
    #endif
    moveServoToAngle(CLAW_SERVO,CLOSED_CLAW);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that commands the height servo to go down
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::goDown()
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Going down");
    Serial.println("--------------------------------------------------");
    #endif
    moveServoToAngle(HEIGHT_SERVO,MIN_HEIGHT);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that commands the height servo to go up
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::goUp()
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Going up");
    Serial.println("--------------------------------------------------");
    #endif
    moveServoToAngle(HEIGHT_SERVO,MAX_HEIGHT);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method to pick up a piece at the current position with the following workflow:
 * Opens the claw -> Goes Down -> Closes the claw -> Goes up
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::pickUp()
{
    OpenClaw();
    goDown();
    CloseClaw();
    goUp();
}

/*---------------------------------------------------------------------------------------------------------------
 * Method to drop a piece at the current position with the following workflow:
 * Goes Down -> Opens the Claw -> Goes Up -> Closes the claw
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::dropDown()
{
    goDown();
    OpenClaw();
    goUp();
    CloseClaw();
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that moves the servos to a desired position
 * NOTE: THE REFERENCE FOR THESE POSITIONS IS THE BASE_SERVO ROTATION POINT (POINT 0,0 IS THE BASE_SERVO ROTATION POINT)
 * @param x, the desired x position in cm
 * @param y, the desired y position in cm
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::moveToPos(float x, float y)
{
    desired_pos[0] = x;
    desired_pos[1] = y;

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
        curr_theta[0] = thetaS1;
        curr_theta[1] = thetaS2;
        #ifdef DEBU_MODE
        Serial.println("--------------------------------------------------");
        Serial.print("Reached position with x: "); Serial.println(curr_pos[0]);
        Serial.print("Reached position with y: "); Serial.println(curr_pos[1]);
        Serial.print("Reached position with base_theta: "); Serial.println(curr_theta[0]);
        Serial.print("Reached position with elbow_theta: "); Serial.println(curr_theta[1]);
        Serial.println("--------------------------------------------------");
        #endif
    }
    else Serial.println("Position not reachable due to physical constraints");
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that attemps to find the PCA by it's address to confirm correct connections
 * @param addr, the address of the PCA
 ---------------------------------------------------------------------------------------------------------------*/
int Kinematics::find_ServoDriver(int addr) 
{
    Wire.beginTransmission(addr);
    Wire.write(PCA9685_MODE1);
    Wire.write(MODE1_RESTART);

    int err = Wire.endTransmission();
    return !err;
}


/*---------------------------------------------------------------------------------------------------------------
 * Method that sets up the dependencies for the kinematics library and sends servos to default positions with the
 * following workflow:
 * 
 * Tries to find the servo driver
 * Starts the PCA
 * Sets the PWM frequency
 * Sends servos to default positions
 ---------------------------------------------------------------------------------------------------------------*/
void Kinematics::kinematics_setup()
{   
    #ifdef DEBU_MODE
    Serial.println("Setting up kinematics dependencies...");
    #endif
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
    #ifdef DEBU_MODE
    Serial.println("Kinematics dependencies Setup");
    #endif
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that checks if a position is reachable based on the distance and theta values
 * 
 * @param h, distance to desired position in cm
 * @param theta1, angle of base servo in degrees
 * @param theta2, angle of elbow servo in degrees
 * @return true if parameters are according to physical constraints, false if not
 ---------------------------------------------------------------------------------------------------------------*/
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