#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stddef.h>
#include <iostream>
#include "opencv2/opencv_modules.hpp"
#include "opencv2/opencv.hpp"



using namespace std;
using namespace cv;

bool inBounds(int i, int min, int max){
    if(i >= min && i <= max) return true;
    return false;
}

int main(){

    Mat scene, white, drawing, drawing_filtered, drawing_polygons;
    vector<int> compression_params;
    compression_params.push_back(IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(9);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    scene = imread("images/Scene2-2.tiff", IMREAD_COLOR);
    if(scene.data == NULL){
        cout << "Image failed to read." << endl;
    }

/// Filter only white
    inRange(scene, Scalar(200,200,200), Scalar(255,255,255), white);

// Get contours
    findContours( white, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
    drawing = Mat::zeros(scene.size(), CV_8UC3);
    drawContours(drawing, contours, -1, Scalar(255,0,0), 2);

    cout << "Number of countours: " << contours.size() << endl;

    vector<Point> approxShape;
    vector<vector<Point>> polygons;
    vector<Point> centers;
    Rect cntr;
    drawing_polygons = Mat::zeros(scene.size(), CV_8UC3);
    for(int i = 0; i < contours.size(); i++){
        if(contours[i].size() > 3 &&
                inBounds(arcLength(contours[i], hierarchy[i][2] >= 0), 100, 2000) && // hierarchy[i][2] >= 0 == TRUE when closed
                inBounds(contourArea(contours[i]), 1500, 100000))
        {
                
            approxPolyDP(contours[i], approxShape, 1, true);

            if(approxShape.size() == 4){
                polygons.push_back(approxShape);

                cntr.x = (approxShape[0].x + approxShape[1].x + approxShape[2].x + approxShape[3].x) / 4 - 10;
                cntr.y = (approxShape[0].y + approxShape[1].y + approxShape[2].y + approxShape[3].y) / 4 - 10;
                cntr.width = 20;
                cntr.height = 20;
                Point c; c.x = cntr.x; c.y = cntr.y;
                centers.push_back(c);
                
                rectangle(drawing_polygons, cntr, Scalar(0,255,0));
            }

        }

        
    }
    drawContours(drawing_polygons, polygons, -1, Scalar(0,0,255));
    cout << polygons.size() << ", 4-point polygons detected." << endl;

    int mainBox, mainBoxArea, area;
    int leftBox1, leftBox2, rightBox1, rightBox2, topBox1, topBox2, bottBox1, bottBox2;
    for(int i = 0; i < polygons.size(); i++){
        area = contourArea(polygons[i]);
        if(area >= mainBoxArea) {
            mainBox = i;
            mainBoxArea = area;
        }/*
        if(polygons[i].x < polygons[leftBox1].x){
            if
        }*/
    }
    drawContours(drawing_polygons, polygons, mainBox, Scalar(255,255,255));

    // Left line



    if(!imwrite("images/out/MaskedScene1.png",white, compression_params) || 
            !imwrite("images/out/Contours.png", drawing, compression_params) || 
            !imwrite("images/out/PolygonContours.png", drawing_polygons, compression_params)){
        cout << "Write failed." << endl;
    } else cout << "Write successful." << endl;

    return 0;

}

