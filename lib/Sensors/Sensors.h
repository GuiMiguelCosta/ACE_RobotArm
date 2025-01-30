#include <Arduino.h>
#include <VL53L0X.h>
#include <Adafruit_TCS34725.h>
#include <Wire.h>

//#define DEBU_MODE 1

//DEFINE TOF CUSTOM ADDRESS
#define TOF_SENSOR_ADDR 0x31

class Sensors
{
    private:
        static VL53L0X tofsensor;
        static Adafruit_TCS34725 tcs;

    public:
        static String getColor();
        static void InitializeSensors();
        static uint16_t readTofDistance();
};