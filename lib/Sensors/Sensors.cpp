#include "Sensors.h"

VL53L0X Sensors::tofsensor;
Adafruit_TCS34725 Sensors::tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

/*---------------------------------------------------------------------------------------------------------------
 * Initializes Sensor Depedencies by the following workflow:
 * 
 * Sets the TOF sensor's address to avoid conflict with the color sensor
 * Initializes the TOF sensor in continuous mode
 ---------------------------------------------------------------------------------------------------------------*/
void Sensors::InitializeSensors()
{
    #ifdef DEBU_MODE
    Serial.println("Setting up sensor dependencies...");
    #endif
    tofsensor.setAddress(TOF_SENSOR_ADDR);
    tofsensor.init();
    tofsensor.startContinuous();
    #ifdef DEBU_MODE
    Serial.println("Sensor dependencies Setup");
    #endif
}

/*---------------------------------------------------------------------------------------------------------------
 * Reads the inputs of the color sensor and returns the color in a string
 * 
 * @return String, the color read by the sensor
 ---------------------------------------------------------------------------------------------------------------*/
String Sensors::getColor() 
{
    #ifdef DEBU_MODE
    Serial.println("Checking Color...");
    #endif
    uint16_t r, g, b, c;

    if (!tcs.begin()) {
        Serial.println("Color sensor not initialized.");
        return "Error";
    }

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

    // Determina a cor dominante
    if (normR > normG && normR > normB) return "Red";
    else if (normG > normR && normG > normB) return "Green";
    else if (normB > normR && normB > normG) return "Blue";
    else return "Unknown";
}

/*---------------------------------------------------------------------------------------------------------------
 * Reads the distance sensor and returns the distance to the origin (0,0)
 * 
 * @return distance, the distance from 0,0 coordinates to the detected item in mm
 ---------------------------------------------------------------------------------------------------------------*/
uint16_t Sensors::readTofDistance()
{
    uint16_t distance = tofsensor.readRangeContinuousMillimeters();
    if (tofsensor.timeoutOccurred()) {
        Serial.println("Erro: Timeout no VL53L0X");
        return 0;
    }
    //These +10 are the 10mm that the base has to the sensor
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.print("Distance to base: ");Serial.println((distance+10));
    Serial.println("--------------------------------------------------");
    #endif
    return (distance+10);
}