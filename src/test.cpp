#include <Arduino.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include "kinematics.h"
#include "Sensors.h"
#include "config.h"

//DEFINE AND INITIALIZE CYCLE-RELATED VALUES AND VARIABLES
#define TIME_BETWEEN_CYCLES 50
uint32_t now = 0;
uint32_t last_cycle = 0;

//DEFINE TRANSITION VALUES AND VARIABLES
#define TIMEOUT 10000
bool automaticMode = false;
bool pieceFound = false;
bool reachedPosition = false;
bool pickupComplete = false;
bool colorChecked = false;
bool dropped = false;

//DEFINE COLOR CHECK POSITIONS
#define DEFAULT_COLOR_CHECK_X 10
#define DEFAULT_COLOR_CHECK_Y 10
float colorCheckX = DEFAULT_COLOR_CHECK_X;
float colorCheckY = DEFAULT_COLOR_CHECK_Y;
String color = "Unknown";

//DEFINE PHYSICAL LIMITS FOR A PIECE
#define MAX_PIECE_DISTANCE (SEGMENT_1_LENGTH+SEGMENT_2_LENGTH)

typedef enum {
    REST,
    MOVE,
    PICKUP,
    CHECK_COLOR,
    DROP,
    SCAN_GRID,
    CONTROL
} state_t;

// Finite state machine
typedef struct {
    state_t state, new_state;
    unsigned long tis, tes;
} fsm_t;

Kinematics kinematics;
fsm_t state_machine;
char input = ' ';

/*---------------------------------------------------------------------------------------------------------------
 * Converts a state's number into a string for information display
 * 
 * @param state (the state number)
 * @return A string that corresponds to the state's name
 ---------------------------------------------------------------------------------------------------------------*/
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

/*---------------------------------------------------------------------------------------------------------------
 * Sets a state machine's state to a new state as well as initializing it's time in state and time entering state
 * 
 * @param fsm: the state machine that will have it's state changed
 * @param new_state: the new state for the state machine
 ---------------------------------------------------------------------------------------------------------------*/
void set_state(fsm_t& fsm, state_t new_state) {
    if (fsm.state != new_state) 
    {   
        #ifdef DEBUG
        Serial.println("--------------------------------------------------");
        Serial.print("Switching from state: ");Serial.print(stateToString(fsm.state));Serial.print("to state: ");Serial.println(stateToString(new_state));
        Serial.println("--------------------------------------------------");
        #endif
        fsm.state = new_state;
        fsm.tes = millis();
        fsm.tis = 0;
    }
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that scans the range of the arm gradually increasing theta
 * In case it finds a piece within range, defines the desired position to those coordinates
 ---------------------------------------------------------------------------------------------------------------*/
void scan()
{
    #ifdef DEBUG
    Serial.println("Starting Scan...");
    #endif

    kinematics.moveToPos(SEGMENT_1_LENGTH+SEGMENT_2_LENGTH,0);
    for (int theta = -90; theta < 90; theta++)
    {
        kinematics.moveServoToAngle(BASE_SERVO,theta);
        //reads distance in mm
        float distance = Sensors::readTofDistance();
        //compares
        if(distance<=MAX_PIECE_DISTANCE*10)
        {
            //reads distance in mm
            float distance = Sensors::readTofDistance();

            //applies the offset for correct x and y calculation
            float phi = 90+theta;

            //calculate x and y
            float x = cos(phi*DEG_TO_RAD) * distance;
            float y = sin(phi*DEG_TO_RAD) * distance;
            //pass x and y to cm
            x = x/10;
            y = y/10;
            //store them in desired pos
            kinematics.desired_pos[0] = x;
            kinematics.desired_pos[1] = y;
            //signal that a piece was found
            pieceFound = true;
            #ifdef DEBUG
            Serial.println("--------------------------------------------------");
            Serial.print("Found a piece at theta: ");Serial.println(theta);
            Serial.print("Distance From Piece is: ");Serial.print(distance);Serial.println("mm");
            Serial.print("Calculated x is: ");Serial.println(x);
            Serial.print("Calculated y is: ");Serial.println(y);
            Serial.println("--------------------------------------------------");
            #endif
            break;
        }
        delay(50);
    }
    #ifdef DEBUG
    Serial.println("Scan Complete without Pieces Found");
    #endif
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that sets up all the initial dependencies and state
 * Starts the Wire and calls the kinematics' and sensor's setups 
 * Defines initial state
 ---------------------------------------------------------------------------------------------------------------*/
void setup() {

    Serial.begin(9600);

    Wire.begin();
    kinematics.kinematics_setup();
    Sensors::InitializeSensors();

    state_machine.new_state = REST;
    set_state(state_machine,state_machine.new_state);
}

/*---------------------------------------------------------------------------------------------------------------
 * Method that loops with each cycle
 * Has the workflow: 
 * Checks if the time past the last cycle has passed and if so:
 * Read Inputs
 * Defines Transition(s) to trigger
 * Sets new state based on transitions
 * Executes methods based on which state is active writing the outputs
 * Resets Inputs for next cycle
--------------------------------------------------------------------------------------------------------------- */
void loop() 
{
    uint32_t now = millis();
    if(now - last_cycle > TIME_BETWEEN_CYCLES)
    {
        //UPDATE CYCLE VARIABLES
        state_machine.tis += (now - last_cycle);
        last_cycle = now;
        //LER ENTRADAS
        if(Serial.available()>0)
        {
            input = Serial.read(); 
        }
        //BINDS FOR AUTOMATIC MODE (INDEPENDENT OF STATE)
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
                #ifdef DEBUG
                Serial.println("Condition to enter SCAN_GRID state from REST triggered");
                #endif
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
                #ifdef DEBUG
                Serial.println("Condition to enter MOVE state from SCAN_GRID triggered");
                #endif
                state_machine.new_state = MOVE;
            }
            else if (!automaticMode || state_machine.tis>TIMEOUT)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from SCAN_GRID triggered");
                #endif
                state_machine.new_state = REST;
            } 
            else 
            {
                state_machine.new_state = SCAN_GRID;
            }
        }
        else if(state_machine.state==MOVE)
        {
            if(reachedPosition)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter PICKUP state from MOVE triggered");
                #endif
                state_machine.new_state = PICKUP;
            }
            else if(state_machine.tis>TIMEOUT)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from MOVE triggered");
                #endif
                state_machine.new_state = REST;
            }
        }
        else if(state_machine.state==PICKUP)
        {
            if(pickupComplete)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter CHECK_COLOR state from PICKUP triggered");
                #endif
                state_machine.new_state = CHECK_COLOR;
            }
            else if(state_machine.tis>TIMEOUT)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from PICKUP triggered");
                #endif
                state_machine.new_state = REST;
            }
        }
        else if(state_machine.state==CHECK_COLOR)
        {
            if(colorChecked)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter DROP state from CHECK_COLOR triggered");
                #endif
                state_machine.new_state = DROP;
            }
            else if(state_machine.tis>TIMEOUT)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from CHECK_COLOR triggered");
                #endif
                state_machine.new_state = REST;
            }
        }
        else if(state_machine.state==DROP)
        {
            if(dropped)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from DROP triggered");
                #endif
                state_machine.new_state = REST;
            }
            else if(state_machine.tis>TIMEOUT)
            {
                #ifdef DEBUG
                Serial.println("Condition to enter REST state from DROP triggered");
                #endif
                state_machine.new_state = REST;
            }
        }

        set_state(state_machine , state_machine.new_state);

        //ESCREVER OUTPUTS
        switch (state_machine.state)
        {
            case REST:
                pieceFound = false;
                reachedPosition = false;
                pickupComplete = false;
                colorChecked = false;
                dropped = false;
                kinematics.moveToPos(0,SEGMENT_1_LENGTH+SEGMENT_2_LENGTH);
                break;
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
                //
                //DEFINE POSITION OF BOX WITH EACH COLOR HERE
                //
                colorChecked = true;
                break;
            case DROP:
                kinematics.moveToPos(kinematics.desired_pos[0],kinematics.desired_pos[1]);
                kinematics.dropDown();
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
                    Sensors::readTofDistance();
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
