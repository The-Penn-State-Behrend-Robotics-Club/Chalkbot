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

    Mat scene, white, drawing, drawing_polygons, drawing_clusters;
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

//////////////////// Filter out canidates //////////////////////////
    vector<Point> approxShape;
    struct canidate_t {vector<Point> polygon; Point center; int width; int height;};
    vector<canidate_t> canidates;
    drawing_polygons = Mat::zeros(scene.size(), CV_8UC3);
    vector<Vec4i>::iterator h = hierarchy.begin();
    for(vector<vector<Point>>::iterator i = contours.begin(); i != contours.end(); i++){
        h++;
        if( (*i).size() >= 4 &&
            inBounds(arcLength((*i), (*h)[2] >= 0), 100, 2000) && // hierarchy[i][2] >= 0 == TRUE when closed
            inBounds(contourArea((*i)), 200, 100000))
        {
            approxPolyDP((*i), approxShape, 1, (*h)[2] >= 0);
            cout << "Found canidate! Size: " << approxShape.size() << endl;

            if(approxShape.size() == 4){

                int x = (approxShape[0].x + approxShape[1].x + approxShape[2].x + approxShape[3].x) / 4 - 10;
                int y = (approxShape[0].y + approxShape[1].y + approxShape[2].y + approxShape[3].y) / 4 - 10;

                cout << "Correct Size!" << endl;
                int top, bottom = INT_MAX, left = INT_MAX, right;
                for(int j = 0; j < 4; j++){
                    if(approxShape[j].x < left)     left = approxShape[j].x;
                    if(approxShape[j].x > right)    right = approxShape[j].x;
                    if(approxShape[j].y < top)      top = approxShape[j].y;
                    if(approxShape[j].y > bottom)   bottom = approxShape[j].y;
                }

                canidates.push_back({
                    approxShape,
                    Point(x,y),
                    right - left,
                    top - bottom
                });
                cout << "Pushed canidate!" << endl;
                drawContours(drawing_polygons, vector<vector<Point> >(1,approxShape), 0, Scalar(0,0,255));
                cout << "Drew canidate! " << x << " " <<  y << endl;

                rectangle(drawing_polygons, Rect(x, y, 20, 20), Scalar(0,255,0));
            }

        }

        
    }
    cout << canidates.size() << ", 4-point polygons detected." << endl;


//////////////////// Filter out clusters //////////////////////////

    drawing_clusters = Mat::zeros(scene.size(), CV_8UC3);
    struct clusterCanidates_t {vector<canidate_t>::iterator outer; vector<vector<canidate_t>::iterator> inners;};
    vector<clusterCanidates_t> clusterCanidates;
    for(vector<canidate_t>::iterator outerCanidate = canidates.begin(); outerCanidate != canidates.end(); outerCanidate++){ // Iterate through outside contours
        vector<vector<canidate_t>::iterator> tempInners; // Temporary record of inner polygons
        int outerArea = -1;
        for(vector<canidate_t>::iterator innerCanidate = canidates.begin(); innerCanidate != canidates.end(); innerCanidate++){ // Iterate through inside contours
            if(innerCanidate != outerCanidate){
                if(pointPolygonTest((*outerCanidate).polygon, (*innerCanidate).center, false) == 1){ // Within contour
                    if(outerArea == -1) outerArea = contourArea((*outerCanidate).polygon); // Only calculate outer area once
                    if(inBounds(outerArea / contourArea((*innerCanidate).polygon), 46, 54)) // If ratio of area is adequate
                        tempInners.push_back(innerCanidate); // Record inner item
                }
            }
            if(tempInners.size() > 5) break;
        }
        if(tempInners.size() == 5) {
            clusterCanidates.push_back({outerCanidate, tempInners}); // Could be changed to >= 5?
            drawContours(drawing_clusters, vector<vector<Point>>(1,(*outerCanidate).polygon), 0, Scalar(255,255,255));
            for(vector<vector<canidate_t>::iterator>::iterator i = tempInners.begin(); i != tempInners.end(); i++)
                drawContours(drawing_clusters, vector<vector<Point>>(1,(**i).polygon), 0, Scalar(255,0,255));
        }
    }
/*
    int mainBox, mainBoxArea, area;
    int leftBox1, leftBox2, rightBox1, rightBox2, topBox1, topBox2, bottBox1, bottBox2;
    for(vector<clusterCanidates_t>::iterator i = clusterCanidates.begin(); i != clusterCanidates.end(); i++){


        
        if(centers[i].x < centers[leftBox1].x){
            if(centers[i].x < centers[leftBox2].x) leftBox2 = i;
            else leftBox1 = i;
        } else{
            if(centers[i].x < centers[leftBox2].x) leftBox2 = i;
        }
        if(centers[i].x > centers[rightBox1].x){
            if(centers[i].x > centers[rightBox2].x) rightBox2 = i;
            else rightBox1 = i;
        } else{
            if(centers[i].x > centers[rightBox2].x) rightBox2 = i;
        }

        if(centers[i].y < centers[bottBox1].y){
            if(centers[i].y < centers[bottBox2].y) bottBox2 = i;
            else bottBox1 = i;
        } else{
            if(centers[i].y < centers[bottBox2].y) bottBox2 = i;
        }
        if(centers[i].y > centers[topBox1].y){
            if(centers[i].y > centers[topBox2].y) topBox2 = i;
            else topBox1 = i;
        } else{
            if(centers[i].y > centers[topBox2].y) topBox2 = i;
        }
    }*/


    // Left line



    if(!imwrite("images/out/MaskedScene1.png",white, compression_params) || 
            !imwrite("images/out/Contours.png", drawing, compression_params) || 
            !imwrite("images/out/PolygonContours.png", drawing_polygons, compression_params) ||
            !imwrite("images/out/Clusters.png", drawing_clusters, compression_params)){
        cout << "Write failed." << endl;
    } else cout << "Write successful." << endl;

    return 0;

}

