#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>

// Definições do servo motor
#define SERVO_MIN 400  // Posição mínima do pulso para o servo
#define SERVO_MAX 2425  // Posição máxima do pulso para o servo
//
#define MAX_HEIGHT -90
#define MIN_HEIGHT 0

#define OPEN_CLAW -45
#define CLOSED_CLAW 0

#define BASE_SERVO 0
#define ELBOW_SERVO 1
#define HEIGHT_SERVO 2
#define CLAW_SERVO 3

#define SEGMENT_1_LENGTH 9
#define SEGMENT_2_LENGTH 12

// Endereço padrão do PCA9685 (0x40)
#define PCA9685_I2C_ADDRESS 0x40
Adafruit_PWMServoDriver pca9685 = Adafruit_PWMServoDriver(PCA9685_I2C_ADDRESS, Wire);

// objects declaration
VL53L0X tofsensor;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int theta_step = 2;
int curr_theta[4] = {0, 0, -90, 0};
int target_theta[4] = {curr_theta[0], curr_theta[1], curr_theta[2], curr_theta[3]};
int n_servos = 4;

int desired_pos[2];

typedef enum
{
    REST,
    MOVE,
    PICKUP,
    CHECK_COLOR,
    DROP,
    SCAN_GRID
};

struct sm 
{
    int curr_state = REST;
    int next_state = REST;
};

sm state_machine;

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
            delay(15);
        }
    } else {
        for (int i = curr_theta[servo_num]; i >= theta; i -= theta_step) 
        {
            float pwm = getPwmForAngle(servo_num, i);
            pca9685.writeMicroseconds(servo_num, pwm);
            delay(15);
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

    for (int step = 0; step <= max_steps; step++) {
        // Calcula o ângulo atual para cada servo baseado no progresso da iteração
        int base_angle = base_start + (step * (base_target - base_start)) / max_steps;
        int elbow_angle = elbow_start + (step * (elbow_target - elbow_start)) / max_steps;

        // Envia os sinais PWM correspondentes aos ângulos calculados
        float base_pwm = getPwmForAngle(BASE_SERVO, base_angle);
        float elbow_pwm = getPwmForAngle(ELBOW_SERVO, elbow_angle);
        pca9685.writeMicroseconds(BASE_SERVO, base_pwm);
        pca9685.writeMicroseconds(ELBOW_SERVO, elbow_pwm);

        delay(15); // Aguarda entre os passos
    }

    // Atualiza os ângulos atuais dos servos
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

void moveToPos(float x, float y)
{
    const float rad_to_deg = 180.0 / 3.1415;

    // Calculate H
    float H = sqrt(x * x + y * y);

    // Validate reachability
    if (H > SEGMENT_1_LENGTH + SEGMENT_2_LENGTH || H <= 0) 
    {
        Serial.println("Target position is out of reach!");
        return;
    }

    // Law of cosines for the second angle (theta2)
    float cosTheta2 = (-H * H + SEGMENT_1_LENGTH * SEGMENT_1_LENGTH + SEGMENT_2_LENGTH * SEGMENT_2_LENGTH) / (2 * SEGMENT_1_LENGTH * SEGMENT_2_LENGTH);
    float theta2 = acos(cosTheta2) * rad_to_deg; // Convert to degrees

    // Law of cosines and trigonometry for the first angle (theta1)
    float angleToTarget = atan2(y, x) * rad_to_deg; // Angle to target point
    float cosAlpha = (H * H + SEGMENT_1_LENGTH * SEGMENT_1_LENGTH - SEGMENT_2_LENGTH * SEGMENT_2_LENGTH) / (2 * H * SEGMENT_1_LENGTH);
    float alpha = acos(cosAlpha) * rad_to_deg; // Convert to degrees

    float theta1 = angleToTarget + alpha;

    // Adjust theta2 to servo convention (depending on linkage orientation)
    float thetaS1 = theta1 - 90;
    float thetaS2 = theta2 - 180; // Assuming the servo convention where 0 degrees is fully extended

    // Adjust and constrain angles
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
    //moveServoToAngle(0, thetaS1);
    //moveServoToAngle(1, thetaS2);
    moveTwo(thetaS1,thetaS2);
}

String getColor() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    Serial.print("Raw R: "); Serial.print(r);
    Serial.print(" G: "); Serial.print(g);
    Serial.print(" B: "); Serial.print(b);
    Serial.print(" C: "); Serial.println(c);

    // Normalize values
    float sum = r + g + b;         
    if (sum == 0) return "Unknown";

    float normR = r / sum;
    float normG = g / sum;
    float normB = b / sum;

    // Determine dominant color
    if (normR > normG && normR > normB) return "Red";
    else if (normG > normR && normG > normB) return "Green";
    else if (normB > normR && normB > normG) return "Blue";
    else return "Unknown";
}

int find_ServoDriver(int addr) 
{
    Wire.beginTransmission(addr);
    Wire.write(PCA9685_MODE1);
    Wire.write(MODE1_RESTART);

    int err = Wire.endTransmission();
    return !err;
}

void setup() 
{
    Serial.begin(9600);

    /*
    // Inicialize Wire1
    Wire1.begin();
    Serial.println("Wire1 initialized");

    // Inicialize o sensor VL53L0X no barramento Wire1
    tofsensor.setBus(&Wire1);
    if (!tofsensor.init()) {
        Serial.println("Failed to initialize VL53L0X on Wire1");
        while (1);
    }
    tofsensor.setTimeout(500);
    tofsensor.startContinuous();
    */

    // Inicialize o barramento padrão para os outros dispositivos
    Wire.begin();
    
    while (!find_ServoDriver(PCA9685_I2C_ADDRESS)) 
    {
        Serial.println("No PCA9685 found ... check your connections");
        delay(200);
    }
    delay(10);
    Serial.println("Found PCA9685");

    pca9685.begin();
    pca9685.setPWMFreq(50);

    for (int i = 0; i < n_servos; i++) {
        pca9685.writeMicroseconds(i, getPwmForAngle(i, curr_theta[i]));
        Serial.print("Setting default thetha for ");
        Serial.println(i);
    }
}

void loop() 
{
    if (Serial.available() > 0) {
        String input = Serial.readStringUntil('\n');  // Read a line from Serial
        input.trim();  // Remove whitespace and newlines

        if (input == "A") 
        {  
            moveToPos(desired_pos[0],desired_pos[1]);
        } 
        if (input == "R") 
        {  
            moveToPos(0,SEGMENT_1_LENGTH+SEGMENT_2_LENGTH);
        } 
        if (input == "O") 
        {  
            OpenClaw();
        } 
        if (input == "C") 
        {  
            CloseClaw();
        } 
        else if(input=="+")
        {
            goUp();
        }
        else if(input=="-")
        {
            goDown();
        }
        else if(input=="P")
        {
            OpenClaw();
            goDown();
            CloseClaw();
            goUp();
        }
        else if (input == "S") 
        {
            Serial.println(getColor());
        }
        else 
        {
            int commaIndex = input.indexOf(',');
            if (commaIndex != -1) 
            {
                desired_pos[0] = input.substring(0, commaIndex).toFloat();
                desired_pos[1] = input.substring(commaIndex + 1).toFloat();
            }
        }

        /*
        int distance = tofsensor.readRangeContinuousMillimeters();
        if (!tofsensor.timeoutOccurred()) {
            Serial.print("Distance: ");
            Serial.print(distance);
            Serial.println(" mm");
        }*/
    }
}