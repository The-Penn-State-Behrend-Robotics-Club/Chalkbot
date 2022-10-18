#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"



using namespace std;
using namespace cv;


int main(){

    Mat scene, white, canny_out, drawing, drawing_filtered;
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    scene = imread("images/Scene1.tiff", IMREAD_COLOR);
    if(scene.data == NULL){
        cout << "Image failed to read." << endl;
    }


    inRange(scene, Scalar(200,200,200), Scalar(255,255,255), white);

    

    Canny(white, canny_out, 100, 200);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours( canny_out, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    drawing = Mat::zeros( canny_out.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
        drawContours( drawing, contours, (int)i, Scalar(255,0,0), 2, LINE_8, hierarchy, 0 );
    }

    

    cout << "Number of countours: " << contours.size() << endl;
    int max = 0, min = INT32_MAX;
    for(int i = 0; i < contours.size(); i++){
        if(contours[i].size() > max) max = contours[i].size();
        if(contours[i].size() < min) min = contours[i].size();
    }
    cout << "Largest contour: " << max << endl;
    cout << "Smallest contour: " << min << endl;

    int thresh = max - 10;
    cout << "Filtering with threshold: " << thresh << endl;
    for(int i = 0; i < contours.size(); i++){
        vector<vector<Point>>::iterator itr = contours.begin();
        itr += i;
        if(contours[i].size() < thresh) contours.erase(itr);
    }

    cout << "Number of filtered countours: " << contours.size() << endl;
    max = 0;
    min = INT32_MAX;
    drawing_filtered = Mat::zeros( canny_out.size(), CV_8UC3 );
    for(int i = 0; i < contours.size(); i++){
        if(contours[i].size() > max) max = contours[i].size();
        if(contours[i].size() < min) min = contours[i].size();
        drawContours(drawing_filtered, contours, (int)i, Scalar(0,0,255), 2, LINE_8, hierarchy, 0 );
    }
    cout << "Largest filtered contour: " << max << endl;
    cout << "Smallest filtered contour: " << min << endl;

    if(!imwrite("images/out/MaskedScene1.png",white, compression_params) || 
            !imwrite("images/out/CannyOut.png", canny_out, compression_params) || 
            !imwrite("images/out/Contours.png", drawing, compression_params) || 
            !imwrite("images/out/FilteredContours.png", drawing_filtered, compression_params)){
        cout << "Write failed." << endl;
    } else cout << "Write successful." << endl;

    return 0;

}

