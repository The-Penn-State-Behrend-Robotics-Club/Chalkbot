#include <iostream>
#include <fstream>
#include <thread>
#include <mutex>
#include <string>
#include "parser.h"

typedef struct mtrControl_t : public std::mutex
{
    int x,y,z;
    mtrControl_t(int numOfChalk){
        bool chalk[numOfChalk];
    }
};

bool draw(mtrControl_t *control, std::string gcodeFileName);
