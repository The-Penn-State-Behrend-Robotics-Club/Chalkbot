#include <markerTracking.h>

using namespace cv;
using namespace std;

namespace ft{ //fiducial tracker

    void *engineWorker(void *args);
    void *engineController(void *args);

    struct engine_t{
        bool running;
        pthread_t controller;
        pthread_t workers[4];
    } engine;

    bool inBounds(int i, int min, int max){
        if(i >= min && i <= max) return true;
        return false;
    }

    int startFiducialEngine(){

        pthread_create(&(engine.controller), NULL, &engineController, NULL);
        pthread_create(&(engine.workers[0]), NULL, &engineWorker, NULL);
        pthread_create(&(engine.workers[1]), NULL, &engineWorker, NULL);
        pthread_create(&(engine.workers[2]), NULL, &engineWorker, NULL);
        pthread_create(&(engine.workers[3]), NULL, &engineWorker, NULL);
        
        

    }

    void *engineController(void *args){


    }

    void *engineWorker(void *args){


    }

    Point findMarker(Mat *scene, bool preserveInput, bool debug){

        Mat workspace;

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        vector<Point> approxShape, centers;
        vector<vector<Point>> polygons;
        Rect cntr;

        if(preserveInput){
            scene->copyTo(workspace);
        } else workspace = *scene;


    /// Filter only white
        inRange(workspace, Scalar(200,200,200), Scalar(255,255,255), workspace);

    // Get contours
        findContours( workspace, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
        if(debug) drawContours(workspace, contours, -1, Scalar(255,0,0), 2);

        if(debug) cout << "Number of countours: " << contours.size() << endl;

    //////////////////// Filter out canidates //////////////////////////

        struct canidate_t {vector<Point> polygon; Point center; int width; int height;};
        vector<canidate_t> canidates;

        vector<Vec4i>::iterator h = hierarchy.begin();

        for(vector<vector<Point>>::iterator i = contours.begin(); i != contours.end(); i++){
            h++;

            if( (*i).size() >= 4 &&
                inBounds(arcLength((*i), (*h)[2] >= 0), 100, 2000) && // hierarchy[i][2] >= 0 == TRUE when closed
                inBounds(contourArea((*i)), 200, 100000))
            {
                approxPolyDP((*i), approxShape, 1, (*h)[2] >= 0);
                if(debug) cout << "Found canidate! Size: " << approxShape.size() << endl;

                if(approxShape.size() == 4){

                    int x = (approxShape[0].x + approxShape[1].x + approxShape[2].x + approxShape[3].x) / 4 - 10;
                    int y = (approxShape[0].y + approxShape[1].y + approxShape[2].y + approxShape[3].y) / 4 - 10;

                    if(debug) cout << "Correct Size!" << endl;
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
                    
                    if(debug) drawContours(workspace, vector<vector<Point> >(1,approxShape), 0, Scalar(0,0,255));

                    if(debug) rectangle(workspace, Rect(x, y, 20, 20), Scalar(0,255,0));
                }

            }

            
        }
        if(debug) cout << canidates.size() << ", 4-point polygons detected." << endl;


    //////////////////// Filter out clusters //////////////////////////

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
                if(debug){
                    drawContours(workspace, vector<vector<Point>>(1,(*outerCanidate).polygon), 0, Scalar(255,255,255));
                    for(vector<vector<canidate_t>::iterator>::iterator i = tempInners.begin(); i != tempInners.end(); i++)
                        drawContours(workspace, vector<vector<Point>>(1,(**i).polygon), 0, Scalar(255,0,255));
                }
            }
        }

        return clusterCanidates[0].outer->center;

    }
}