#include "Sensors.h"

Adafruit_TCS34725 Sensors::tcs = Adafruit_TCS34725(TCS34725_INTEGRATIONTIME_50MS, TCS34725_GAIN_4X);

String Sensors::getColor() 
{
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
