#include <Arduino.h>

//#define DEBU_MODE 1

#define DEFAULT_GRID_X 10
#define DEFAULT_GRID_Y 10
#define DEFAULT_DBG 1

class GridTranslation
{
    private:
        float gridX = DEFAULT_GRID_X;
        float gridY = DEFAULT_GRID_Y;
        int dist_between_grids = DEFAULT_DBG;
    public:
        void setNewGrid(float x, float y);
        void changeGridDimensions(int new_distance);
        void getGridPos(int pos, float* vec);
};