#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <vector>
#include <pthread.h>
#include <signal.h>

#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"

using namespace cv;

struct whiteFilter_args{
    bool debug;
    Mat sceneIn;
    Scalar upper;
    Scalar lower;
};
struct findCanidates_args{

};
struct findClusters_args{

};

cv::Point findMarker(cv::Mat *scene, bool preserveInput, bool debug = false);
