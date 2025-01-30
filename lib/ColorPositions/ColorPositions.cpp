#include "ColorPositions.h"

void ColorPositions::getCheckerPos(float* position)
{
    position[0] = colorCheck[0];
    position[1] = colorCheck[1];
}
void ColorPositions::getDepositPosition(float* position, String color)
{
    if(color.equals("Green"))
    {
        position[0] = greenDeposit[0];
        position[1] = greenDeposit[1];
    }
    else if(color.equals("Yellow"))
    {
        position[0] = yellowDeposit[0];
        position[1] = yellowDeposit[1];
    }
    else if(color.equals("Blue"))
    {
        position[0] = blueDeposit[0];
        position[1] = blueDeposit[1];
    }
    else if(color.equals("Red"))
    {
        position[0] = redDeposit[0];
        position[1] = redDeposit[1];
    }
    else
    {
        position[0] = 0;
        position[1] = 0;
    }
}
void ColorPositions::changeCheckerPosition(float x, float y)
{
    colorCheck[0] = x;
    colorCheck[1] = y;
}
void ColorPositions::changeDepositPosition(float x, float y, int deposit)
{
    switch (deposit)
    {
    case 1:
        greenDeposit[0] = x;
        greenDeposit[1] = y;
        break;
    case 2:
        yellowDeposit[0] = x;
        yellowDeposit[1] = y;
        break;
    case 3:
        blueDeposit[0] = x;
        blueDeposit[1] = y;
        break;
    case 4:
        redDeposit[0] = x;
        redDeposit[1] = y;
        break;
    default:
        break;
    }
}