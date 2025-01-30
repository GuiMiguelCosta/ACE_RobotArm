#include "GridTranslation.h"

void GridTranslation::setNewGrid(float x, float y)
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Setting up new grid coordinates");
    Serial.println("--------------------------------------------------");
    #endif
    gridX = x;
    gridY = y;
}

void GridTranslation::changeGridDimensions(int new_distance)
{
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Setting up new grid distance between blocks");
    Serial.println("--------------------------------------------------");
    #endif
    dist_between_grids = new_distance;
}

void GridTranslation::getGridPos(int pos, float* vec)
{
    float position[2] = {0,0};
    switch (pos)
    {
    case 1:
        position[0] = gridX - dist_between_grids;
        position[1] = gridY + dist_between_grids;
        break;
    case 2:
        position[0] = gridX;
        position[1] = gridY + dist_between_grids;
        break;
    case 3:
        position[0] = gridX + dist_between_grids;
        position[1] = gridY + dist_between_grids;
        break;
    case 4:
        position[0] = gridX - dist_between_grids;
        position[1] = gridY;
        break;
    case 5:
        position[0] = gridX;
        position[1] = gridY;
        break;
    case 6:
        position[0] = gridX + dist_between_grids;
        position[1] = gridY;
        break;
    case 7:
        position[0] = gridX - dist_between_grids;
        position[1] = gridY - dist_between_grids;
        break;
    case 8:
        position[0] = gridX;
        position[1] = gridY - dist_between_grids;
        break;
    case 9:
        position[0] = gridX + dist_between_grids;
        position[1] = gridY - dist_between_grids;
        break;
    default:
        break;
    }
    #ifdef DEBU_MODE
    Serial.println("--------------------------------------------------");
    Serial.println("Getting coordinates for desired grid position...");
    Serial.print("Coordinates for position ");Serial.print(pos);Serial.println(":");
    Serial.print("X: ");Serial.println(position[0]);
    Serial.print("Y: ");Serial.println(position[1]);
    Serial.println("--------------------------------------------------");
    #endif
    vec[0] = position[0];
    vec[1] = position[1];
}
