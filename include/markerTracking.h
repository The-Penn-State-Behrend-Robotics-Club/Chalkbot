#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include <vector>
#include <pthread.h>

#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"

cv::Point findMarker(cv::Mat *scene, bool preserveInput, bool debug = false);
