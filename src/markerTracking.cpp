#include <markerTracking.h>

using namespace cv;
using namespace std;

namespace ft{ //fiducial tracker

    void *engineController(void *args);
    void *whiteFilter(void *args);
    void *findClusters(void *args);


    bool inBounds(int i, int min, int max){
        if(i >= min && i <= max) return true;
        return false;
    }

    struct engine_t{
        bool running;
        pthread_t controller_pt;
        pthread_t whiteFilter_pt;
        pthread_t findCanidates_pt;
        pthread_t findClusters_pt;

        pthread_barrier_t startBarrier, endBarrier;
        pthread_mutex_t sharedLock;
        pthread_cond_t cycle;

        struct {
            bool run;
            bool exit;

            Mat sceneIn;
            Mat maskedScene;
            vector<vector<Point>> *contours;
            vector<Rect> *clusters;

        } shared;

        struct whiteFilterArgs_t{
            bool debug;
            Scalar upper;
            Scalar lower;
        } whiteFilterArgs;

        struct findCanidatesArgs_t{
            bool debug;

        }findCanidatesArgs;

        struct findClustersArgs_t{
            bool debug;

        }findClustersArgs;
    } engine;    

    bool startFiducialEngine(){

        pthread_mutex_init(&(engine.sharedLock), NULL);
        pthread_barrier_init(&(engine.startBarrier), NULL, 4);
        pthread_barrier_init(&(engine.endBarrier), NULL, 4);

        pthread_mutex_lock(&(engine.sharedLock));
        engine.shared.run = false;
        engine.shared.exit = false;
        pthread_mutex_unlock(&(engine.sharedLock));

        if(!((pthread_create(&(engine.controller_pt)    , NULL, &engineController , NULL) == 0) &&
             (pthread_create(&(engine.whiteFilter_pt)   , NULL, &whiteFilter      , NULL) == 0) &&
             (pthread_create(&(engine.findClusters_pt)  , NULL, &findClusters     , NULL) == 0)))
        {
            pthread_mutex_lock(&(engine.sharedLock));
            engine.shared.exit = true;
            pthread_mutex_unlock(&(engine.sharedLock));

            pthread_kill(engine.controller_pt, SIGINT);
            pthread_kill(engine.whiteFilter_pt, SIGINT);
            pthread_kill(engine.findClusters_pt, SIGINT);
            
            pthread_join(engine.controller_pt, NULL);
            pthread_join(engine.whiteFilter_pt, NULL);
            pthread_join(engine.findClusters_pt, NULL);

            return false;
        }
        
        return true;
        

    }

    void *engineController(void *args){


    }

    void *whiteFilter(void *args){

        Mat sceneOut[2], sceneIn;
        bool activeBuff;

        sceneOut[0] = Mat::zeros(sceneIn.size(), CV_8UC3);
        sceneOut[1] = Mat::zeros(sceneIn.size(), CV_8UC3);

        pthread_mutex_lock(&(engine.sharedLock));
        while(1){
            while(1){
                if(engine.shared.run == true) break;
                if(engine.shared.exit == true) {
                    pthread_mutex_unlock(&(engine.sharedLock));
                    return NULL;
                }
                pthread_cond_wait(&(engine.cycle), &(engine.sharedLock));
            }
            pthread_mutex_unlock(&(engine.sharedLock));

            pthread_barrier_wait(&(engine.startBarrier)); // Wait for all threads to be ready.

            inRange(sceneIn, engine.whiteFilterArgs.lower, engine.whiteFilterArgs.upper, sceneOut[activeBuff]);

            pthread_barrier_wait(&(engine.endBarrier)); // Wait for all threads to finish.
            
            pthread_mutex_lock(&(engine.sharedLock));
            engine.shared.maskedScene = sceneOut[activeBuff];
            activeBuff = !activeBuff; // Switch to other buffer.
        }
    }

    void *findContours(void *args){
        vector<vector<Point>> foundContours[2];
        vector<Vec4i> hierarchy;
        bool activeBuff;

        pthread_mutex_lock(&(engine.sharedLock));
        while(1){
            while(1){
                if(engine.shared.run == true) break;
                if(engine.shared.exit == true) {
                    pthread_mutex_unlock(&(engine.sharedLock));
                    return NULL;
                }
                pthread_cond_wait(&(engine.cycle), &(engine.sharedLock));
            }
            pthread_mutex_unlock(&(engine.sharedLock));

            pthread_barrier_wait(&(engine.startBarrier)); // Wait for all threads to be ready.

            findContours(engine.shared.maskedScene, foundContours[activeBuff], hierarchy, RETR_FLOODFILL, CHAIN_APPROX_SIMPLE);

            pthread_barrier_wait(&(engine.endBarrier)); // Wait for all threads to finish.
            
            pthread_mutex_lock(&(engine.sharedLock));
            engine.shared.contours = &foundContours[activeBuff];
            activeBuff = !activeBuff; // Switch to other buffer.
            foundContours[activeBuff].clear(); // Empty old buffer.

        }
    }

    void *findClusters(void *args){

        pthread_mutex_lock(&(engine.sharedLock));
        while(1){
            while(1){
                if(engine.shared.run == true) break;
                if(engine.shared.exit == true) {
                    pthread_mutex_unlock(&(engine.sharedLock));
                    return NULL;
                }
                pthread_cond_wait(&(engine.cycle), &(engine.sharedLock));
            }
            pthread_mutex_unlock(&(engine.sharedLock));

            pthread_barrier_wait(&(engine.startBarrier)); // Wait for all threads to be ready.

            RotatedRect approxShape;
            vector<RotatedRect> canidates;
            vector<RotatedRect> clusters[2];
            bool activeBuff;
            //vector<Point> orientedShape;

            //struct canidate_t {vector<Point> polygon; Point center; int width; int height;};
            //vector<canidate_t> canidates;

            for(vector<vector<Point>>::iterator i = (*engine.shared.contours).begin(); i != (*engine.shared.contours).end(); i++){
                
                if( (*i).size() >= 4 &&
                    inBounds(arcLength((*i), true), 100, 2000) && // hierarchy[i][2] >= 0 == TRUE when closed
                    inBounds(contourArea((*i)), 200, 100000))
                {
                    approxShape = minAreaRect(*i);
                    //approxPolyDP((*i), approxShape, 1, true);
                    //if(debug) cout << "Found canidate! Size: " << approxShape.size() << endl;

                    //int x = (approxShape[0].x + approxShape[1].x + approxShape[2].x + approxShape[3].x) / 4 - 10;
                    //int y = (approxShape[0].y + approxShape[1].y + approxShape[2].y + approxShape[3].y) / 4 - 10;

                    //if(debug) cout << "Correct Size!" << endl;
                    /*int top, bottom = INT_MAX, left = INT_MAX, right;
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
                    });*/

                    canidates.push_back(approxShape);
                    
                    //if(debug) drawContours(workspace, vector<vector<Point> >(1,approxShape), 0, Scalar(0,0,255));

                    //if(debug) rectangle(workspace, Rect(x, y, 20, 20), Scalar(0,255,0));
                    

                }

                
            }
        //if(debug) cout << canidates.size() << ", 4-point polygons detected." << endl;

            //struct clusterCanidates_t {vector<canidate_t>::iterator outer; vector<vector<canidate_t>::iterator> inners;};
            struct clusterCanidates_t {vector<RotatedRect>::iterator outer; vector<vector<RotatedRect>::iterator> inners;};

            vector<clusterCanidates_t> clusterCanidates;


            for(vector<RotatedRect>::iterator outerCanidate = canidates.begin(); outerCanidate != canidates.end(); outerCanidate++){ // Iterate through outside contours
                vector<vector<RotatedRect>::iterator> tempInners; // Temporary record of inner polygons

                for(vector<RotatedRect>::iterator innerCanidate = canidates.begin(); innerCanidate != canidates.end(); innerCanidate++){ // Iterate through inside contours
                    if(innerCanidate != outerCanidate){
                        vector<Point> outerShape;
                        boxPoints(*outerCanidate, outerShape);

                        if( pointPolygonTest(outerShape, (*innerCanidate).center, false) == 1 && // If within contour (Kinda)
                            inBounds((*outerCanidate).size.area() / (*innerCanidate).size.area(), 46, 54)) // If ratio of area is adequate
                        {
                            tempInners.push_back(innerCanidate); // Record inner item
                        }
                    }
                    if(tempInners.size() > 5) break;
                }
                if(tempInners.size() == 5) {
                    clusterCanidates.push_back({outerCanidate, tempInners}); // Could be changed to >= 5?
                    //drawContours(drawing_clusters, vector<vector<Point>>(1,(*outerCanidate).polygon), 0, Scalar(255,255,255));
                    //for(vector<vector<canidate_t>::iterator>::iterator i = tempInners.begin(); i != tempInners.end(); i++)
                        //drawContours(drawing_clusters, vector<vector<Point>>(1,(**i).polygon), 0, Scalar(255,0,255));
                }
            }

            for(vector<clusterCanidates_t>::iterator i = clusterCanidates.begin(); i < clusterCanidates.end(); i++){
                clusters[activeBuff].push_back(*(*i).outer);
            }

            pthread_barrier_wait(&(engine.endBarrier)); // Wait for all threads to finish.
            
            pthread_mutex_lock(&(engine.sharedLock));

        }
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