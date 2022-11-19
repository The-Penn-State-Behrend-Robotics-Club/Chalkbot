#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <vector>
#include <string>

#include "markerTracking.h"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace ft;


int main(){

    fidSettings settings;
    settings.cl_ratioUpper = 54;
    settings.cl_ratioLower = 48;
    settings.wf_upper = 255;
    settings.wf_lower = 200;

    fiducialEngine eng(settings);
    
    if(eng.start()){
        cout << "Engine started!" << endl;
    } else {
        cout << "Engine failed to start." << endl;
        return -1;
    }
    for(int i = 1; i < 4; i++){
        Mat scene = imread("/home/adam/Chalkbot/images/Scene" + to_string(i) + "-2.tiff", IMREAD_COLOR);
        eng.feedImage(scene);
        eng.step();
        cout << eng.getClusters() << endl;

    }

    cout << "Finished" << endl;

}