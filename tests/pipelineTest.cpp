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

    cout << "gitDirectory: " << gitDirectory;
    gitDirectory = gitDirectory.substr(0, gitDirectory.find("Chalkbot") );
    cout << "-> " << gitDirectory << endl;
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
        string filename = "Scene" + to_string(i) + "-2.tiff";
        Mat scene = imread(gitDirectory + "Chalkbot/images/" + filename, IMREAD_COLOR);
        eng.feedImage(scene);
        eng.step();
        cout << filename << ": " << eng.getClusters().size() << " clusters" << endl;

    }

    eng.stop();

    cout << "Finished" << endl;

    return 0;

}