#include <Arduino.h>
#include <Wire.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Adafruit_PWMServoDriver.h>
#include "kinematics.h"

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

VL53L0X tofsensor;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

int theta_step = 2;
int curr_theta[4] = {0, 90, -90, 0};
int target_theta[4] = {curr_theta[0], curr_theta[1], curr_theta[2], curr_theta[3]};
int n_servos = 4;

int desired_pos[2];

struct sm 
{
    int curr_state = REST;
    int next_state = REST;
};

sm state_machine;


// states
typedef enum{
    REST,
    MOVE,
    PICKUP,
    CHECK_COLOR,
    DROP,
    SCAN_GRID
} state_t;

// finite state machine
typedef struct{
    state_t state, new_state;
    unsigned long tis, tes;
} fsm_t;

// finite state machines declaration
fsm_t scanner, grabber, COLOR, one, two, three, control_fsm;

// set state function
void set_state(fsm_t& fsm, state_t new_state) {
    if(fsm.state != new_state){
        fsm.state = new_state;
        fsm.tes = millis();
        fsm.tis = 0;
    }
}

// objects declaration
VL53L0X tofsensor;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
/*ServoControl control;*/
Kinematics kinematics;

// aux variables


// functions declaration
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

void setup() 
{
    Serial.begin(9600);
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
        String input = Serial.readStringUntil('\n'); 
        input.trim(); 

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
            pickUp();
        }
        else if(input=="D")
        {
            dropDown();
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
    }
}


String getColor() {
    uint16_t r, g, b, c;
    tcs.getRawData(&r, &g, &b, &c);

    Serial.print("Raw R: "); Serial.print(r);
    Serial.print(" G: "); Serial.print(g);
    Serial.print(" B: "); Serial.print(b);
    Serial.print(" C: "); Serial.println(c);

    float sum = r + g + b;         
    if (sum == 0) return "Unknown";

    float normR = r / sum;
    float normG = g / sum;
    float normB = b / sum;

    if (normR > normG && normR > normB) return "Red";
    else if (normG > normR && normG > normB) return "Green";
    else if (normB > normR && normB > normG) return "Blue";
    else return "Unknown";
}


