#include <Arduino.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

class Sensors
{
    private:
        static VL53L0X tofsensor;
        static Adafruit_TCS34725 tcs;

    public:
        static String getColor();
};