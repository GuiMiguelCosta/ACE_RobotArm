#include <Arduino.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include "kinematics.h"
#include "Sensors.h"

//#define DEBUG

#define TIME_BETWEEN_CYCLES 50
uint32_t now = 0;
uint32_t last_cycle = 0;

bool automaticMode = false;
bool pieceFound = false;
bool reachedPosition = false;
bool pickupComplete = false;
bool colorChecked = false;
bool dropped = false;

#define DEFAULT_COLOR_CHECK_X 10
#define DEFAULT_COLOR_CHECK_Y 10
float colorCheckX = DEFAULT_COLOR_CHECK_X;
float colorCheckY = DEFAULT_COLOR_CHECK_Y;
String color = "Unknown";

#define MAX_PIECE_DISTANCE 21

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

Kinematics kinematics;

void scan()
{
    kinematics.moveToPos(SEGMENT_1_LENGTH+SEGMENT_2_LENGTH,0);
    for (int theta = -90; theta < 90; theta++)
    {
        kinematics.moveServoToAngle(BASE_SERVO,theta);
        //reads distance in mm
        uint16_t distance = Sensors::readTofDistance();
        //passes to cm
        distance = distance / 10;
        //compares
        if(distance<=MAX_PIECE_DISTANCE)
        {
            float phi = 90+theta;
            float deg_to_rad = 3.1415/180;
            int valor = distance;
            float x = cos(phi*deg_to_rad) * valor;
            float y = sin(phi*deg_to_rad) * valor;
            kinematics.desired_pos[0] = x;
            kinematics.desired_pos[1] = y;
            pieceFound = true;
            break;
        }
        delay(50);
    }
}

char input = ' ';

void setup() {

    Serial.begin(9600);

    Wire.begin();
    kinematics.kinematics_setup();
    Sensors::InitializeSensors();

    state_machine.new_state = CONTROL;
    set_state(state_machine,state_machine.new_state);
    Sensors::InitializeSensors();
}

void loop() 
{
    uint32_t now = millis();
    if(now - last_cycle > TIME_BETWEEN_CYCLES)
    {
        #ifdef DEBUG
        Serial.println("Entering New Cycle");
        Serial.print("Currently in state: "); Serial.println(stateToString(state_machine.state));
        #endif

        //UPDATE CYCLE VARIABLES
        last_cycle = now;
        //LER ENTRADAS
        if(Serial.available()>0)
        {
            input = Serial.read(); 
        }
        if(input=='A') automaticMode=true;
        if(input=='Q') automaticMode=false;
        //VERIFICAR TRANSIÇÕES
        if(state_machine.state==CONTROL)
        {
            if(input=='E')
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from CONTROL triggered");
                #endif

                state_machine.new_state = REST;
            }
        }
        else if (state_machine.state == REST) 
        {
            if (automaticMode) 
            {
                state_machine.new_state = SCAN_GRID;
            } 
            if(input=='C')
            {
                #ifdef DEBUG
                Serial.println("Condition to enter CONTROL state from REST triggered");
                #endif

                state_machine.new_state = CONTROL;
            }
        }
        else if(state_machine.state==SCAN_GRID)
        {
            if(pieceFound)
            {
                pieceFound = false;
                state_machine.new_state = MOVE;
            }
            /*************************************************************************
            else if (!automaticMode)
            {
                state_machine.new_state = REST;
            } 
            } else {
            // Reentrar no mesmo estado para repetir o scan
                state_machine.new_state = SCAN_GRID;
            }************************************************************************/ 
        }
        else if(state_machine.state==MOVE)
        {
            if(reachedPosition)
            {
                reachedPosition = false;
                state_machine.new_state = PICKUP;
            }
        }
        else if(state_machine.state==PICKUP)
        {
            if(pickupComplete)
            {
                pickupComplete = false;
                state_machine.new_state = CHECK_COLOR;
            }
        }
        else if(state_machine.state==CHECK_COLOR)
        {
            if(colorChecked)
            {
                colorChecked = false;
                state_machine.new_state = DROP;
            }
        }
        else if(state_machine.state==DROP)
        {
            if(dropped)
            {
                dropped = false;
                state_machine.new_state = REST;
            }
        }

        set_state(state_machine , state_machine.new_state);

        //ESCREVER OUTPUTS
        switch (state_machine.state)
        {
            case SCAN_GRID:
                scan();
                break;
            case MOVE:
                kinematics.moveToPos(kinematics.desired_pos[0],kinematics.desired_pos[1]);
                if(kinematics.desired_pos[0]==kinematics.curr_pos[0] &&
                kinematics.desired_pos[1]==kinematics.curr_pos[1]) reachedPosition=true;
                break;
            case PICKUP:
                kinematics.pickUp();
                pickupComplete = true;
                break;
            case CHECK_COLOR:
                kinematics.moveToPos(colorCheckX,colorCheckY);
                color = Sensors::getColor();
                colorChecked = true;
                break;
            case DROP:
                kinematics.moveToPos(0,SEGMENT_1_LENGTH+SEGMENT_2_LENGTH);
                dropped=true;
                break;
            case CONTROL:
                if (input == 'R') {  
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
                    Serial.println(Sensors::getColor());
                }
                else if (input == 'T') {
                    Serial.println(Sensors::readTofDistance());
                }
                else if (input == 'Z') {
                    scan();
                    if(pieceFound)
                    {
                        kinematics.moveToPos(kinematics.desired_pos[0],kinematics.desired_pos[1]);
                        kinematics.pickUp();
                    }
                }
                else if (input=='M') 
                {
                    String inputBuffer = "";
                    int x = 0, y = 0;
                    bool validInput = false;

                    Serial.println("Enter X,Y values or press L to cancel:");

                    while (!validInput) {
                        if (Serial.available() > 0) {
                            char ch = Serial.read();

                            if (ch == 'L') {
                                Serial.println("Operation canceled.");
                                break;
                            }
                            else if (ch == '\n' || ch == '\r') {
                                int commaIndex = inputBuffer.indexOf(',');
                                if (commaIndex != -1) {
                                    x = inputBuffer.substring(0, commaIndex).toInt();
                                    y = inputBuffer.substring(commaIndex + 1).toInt();
                                    validInput = true;
                                } else {
                                    Serial.println("Invalid format. Use X,Y.");
                                    inputBuffer = "";
                                }
                            } else {
                                inputBuffer += ch;
                            }
                        }
                    }

                    if (validInput) {
                        Serial.print("Moving to position X: "); Serial.print(x); Serial.print(" Y: "); Serial.println(y);
                        kinematics.moveToPos(x, y);
                    }
                }
            break;

            default:
                break;
        }

        //reset input
        input = ' ';
    }
    
}
