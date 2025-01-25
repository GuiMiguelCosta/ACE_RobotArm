#include <Arduino.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include "kinematics.h"

#define TIME_BETWEEN_CYCLES 50
uint32_t now = 0;
uint32_t last_cycle = 0;

typedef enum {
    REST,
    MOVE,
    PICKUP,
    CHECK_COLOR,
    DROP,
    SCAN_GRID,
    CONTROL
} state_t;

const char* stateToString(state_t state) {
    switch (state) {
        case REST: return "REST";
        case MOVE: return "MOVE";
        case PICKUP: return "PICKUP";
        case CHECK_COLOR: return "CHECK_COLOR";
        case DROP: return "DROP";
        case SCAN_GRID: return "SCAN_GRID";
        case CONTROL: return "CONTROL";
        default: return "UNKNOWN";
    }
}

// Finite state machine
typedef struct {
    state_t state, new_state;
    unsigned long tis, tes;
} fsm_t;

fsm_t state_machine;

// Set state function
void set_state(fsm_t& fsm, state_t new_state) {
    if (fsm.state != new_state) {
        Serial.print("Switching from ");
        Serial.print(stateToString(fsm.state));  
        Serial.print(" to ");
        Serial.println(stateToString(new_state)); 

        fsm.state = new_state;
        fsm.tes = millis();
        fsm.tis = 0;
    }
}

// Objects declaration
VL53L0X tofsensor;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);
Kinematics kinematics;

char input;

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

void setup() {
    Serial.begin(9600);
    kinematics.kinematics_setup();
    set_state(state_machine,CONTROL);
}

void loop() 
{
    uint32_t now = millis();
    if(now - last_cycle > TIME_BETWEEN_CYCLES)
    {
        //UPDATE CYCLE VARIABLES
        last_cycle = now;
        //LER ENTRADAS
        if(Serial.available()>0)
        {
            input = Serial.read(); 
        }
        //VERIFICAR TRANSIÇÕES
        if(state_machine.state==CONTROL)
        {
            if(input=='E')
            {
                set_state(state_machine,REST);
            }
        }

        //ESCREVER OUTPUTS
        switch (state_machine.state)
        {
            case CONTROL:
                if (input == 'A') {  
                    
                    kinematics.moveToPos(kinematics.desired_pos[0], kinematics.desired_pos[1]);
                } 
                else if (input == 'R') {  
                    kinematics.moveToPos(0, SEGMENT_1_LENGTH + SEGMENT_2_LENGTH);
                } 
                else if (input == 'O') {  
                    kinematics.OpenClaw();
                } 
                else if (input == 'C') {  
                    kinematics.CloseClaw();
                } 
                else if (input == '+') {
                    kinematics.goUp();
                }
                else if (input == '-') {
                    kinematics.goDown();
                }
                else if (input == 'P') {
                    kinematics.pickUp();
                }
                else if (input == 'D') {
                    kinematics.dropDown();
                }
                else if (input == 'S') {
                    Serial.println(getColor());
                }
                else 
                {
                    
                }
            break;
            
            default:
                break;
        }

        //reset input
        input = ' ';
    }
    
}
