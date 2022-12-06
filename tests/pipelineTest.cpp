#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <vector>
#include <string>
#include <filesystem>

#include "markerTracking.h"
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"

using namespace std;
using namespace ft;


int main(){

    String gitDirectory = get_current_dir_name();
    gitDirectory = gitDirectory.substr(0, gitDirectory.find("Chalkbot/"));

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
        Mat scene = imread(gitDirectory + "Chalkbot/images/Scene" + to_string(i) + "-2.tiff", IMREAD_COLOR);
        eng.feedImage(scene);
        while(eng.getClusters().size() == 0) eng.step();
        cout << eng.getClusters().size() << endl;

    }

    cout << "Finished" << endl;

}