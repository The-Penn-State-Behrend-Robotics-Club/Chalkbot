#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"



using namespace std;
using namespace cv;


int main(){



    Mat markerImage;
    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(aruco::DICT_6X6_250);
    aruco::drawMarker(dictionary, 23, 200, markerImage, 1);
    imwrite("marker23.png", markerImage);

}