#include <Arduino.h>

//DEFINE COLOR CHECK POSITIONS
#define DEFAULT_COLOR_CHECK_X 0
#define DEFAULT_COLOR_CHECK_Y 23

#define DEFAULT_GREEN_DEPOSIT_X 23
#define DEFAULT_GREEN_DEPOSIT_Y 0

#define DEFAULT_YELLOW_DEPOSIT_X 23
#define DEFAULT_YELLOW_DEPOSIT_Y 0

#define DEFAULT_BLUE_DEPOSIT_X 23
#define DEFAULT_BLUE_DEPOSIT_Y 0

#define DEFAULT_RED_DEPOSIT_X 23
#define DEFAULT_RED_DEPOSIT_Y 0

class ColorPositions
{
    private:
        float colorCheck[2] = {DEFAULT_COLOR_CHECK_X,DEFAULT_COLOR_CHECK_Y};
        float greenDeposit[2] = {DEFAULT_GREEN_DEPOSIT_X,DEFAULT_GREEN_DEPOSIT_Y};
        float yellowDeposit[2] = {DEFAULT_YELLOW_DEPOSIT_X,DEFAULT_YELLOW_DEPOSIT_Y};
        float blueDeposit[2] = {DEFAULT_BLUE_DEPOSIT_X,DEFAULT_BLUE_DEPOSIT_Y};
        float redDeposit[2] = {DEFAULT_RED_DEPOSIT_X,DEFAULT_RED_DEPOSIT_Y};
    public:
        void getCheckerPos(float* position);
        void getDepositPosition(float* position, String color);
        void changeCheckerPosition(float x, float y);
        void changeDepositPosition(float x, float y, int deposit);
};