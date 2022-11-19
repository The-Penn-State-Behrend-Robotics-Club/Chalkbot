#include <markerTracking.h>

using namespace cv;
using namespace std;



namespace ft{ //fiducial tracker


    bool inBounds(int i, int min, int max){
        if(i >= min && i <= max) return true;
        return false;
    }

    fiducialEngine::fiducialEngine(){
        this->shared.contours = nullptr;
        this->shared.clusters = nullptr;
    }

    fiducialEngine::fiducialEngine(fidSettings settings){
        this->settings = settings; // Just copy the settings
        this->shared.contours = nullptr;
        this->shared.clusters = nullptr;
    }

    fiducialEngine::~fiducialEngine(){
        this->stop();
    }

    bool fiducialEngine::start(){

        pthread_mutex_init(&(this->sharedLock), NULL);
        pthread_barrier_init(&(this->startBarrier), NULL, 3);
        pthread_barrier_init(&(this->endBarrier), NULL, 3);
        pthread_cond_init(&(this->step_sig), NULL);
        pthread_cond_init(&(this->newData_sig), NULL);


        this->shared.run = false;
        this->shared.exit = false;
        this->shared.wf_state = stopped;
        this->shared.cd_state = stopped;
        this->shared.cl_state = stopped;

        this->started = true;

        if(!(//(pthread_create(&(engine.controller_pt)    , NULL, &engineController , NULL) == 0) &&
                (pthread_create(&(this->whiteFilter_pt)   , NULL, &whiteFilter      , this) == 0) &&
                (pthread_create(&(this->findClusters_pt)  , NULL, &findCandidates   , this) == 0) &&
                (pthread_create(&(this->findClusters_pt)  , NULL, &findClusters     , this) == 0)))
        {
            
            this->stop();

            return false;
        }
        
        return true;

    }

    bool fiducialEngine::stop(){

        if(!this->started) return true;

        pthread_mutex_lock(&(this->sharedLock));
        this->shared.exit = true;
        pthread_mutex_unlock(&(this->sharedLock));


        while(1){
            int waiting = 0;
            if(!this->shared.wf_state && !this->shared.cd_state && !this->shared.cl_state){
                break;
            }else {
                if(waiting > 100){ // Escalate to killing
                    pthread_kill(this->whiteFilter_pt, SIGINT);
                    pthread_kill(this->findCanidates_pt, SIGINT);
                    pthread_kill(this->findClusters_pt, SIGINT);
                    this->shared.wf_state = stopped;
                    this->shared.cd_state = stopped;
                    this->shared.cl_state = stopped;
                    break;
                }
                usleep(1000); // 1 ms
                waiting++;
            }
        }
        
        //pthread_join(engine.controller_pt, NULL);
        pthread_join(this->whiteFilter_pt, NULL);
        pthread_join(this->findCanidates_pt, NULL);
        pthread_join(this->findClusters_pt, NULL);

        return true;
    }

    void fiducialEngine::step(){

        pthread_mutex_lock(&(this->sharedLock));
        this->shared.run = true;
        pthread_cond_broadcast(&(this->step_sig));
        pthread_mutex_unlock(&(this->sharedLock));

        pthread_mutex_lock(&(this->sharedLock));
        while(!this->shared.newData){ // This loop is mainly for initializing the pipeline.
            pthread_mutex_unlock(&(this->sharedLock));
    
            pthread_cond_broadcast(&(this->step_sig)); // Keep stepping until new data.
            pthread_cond_wait(&(this->newData_sig), &(this->sharedLock)); // Possible new data?

            pthread_mutex_lock(&(this->sharedLock));
        }
        pthread_mutex_unlock(&(this->sharedLock));

        pthread_mutex_lock(&(this->sharedLock));
        this->shared.run = false;
        this->shared.newData = false;
        pthread_mutex_unlock(&(this->sharedLock));

    }
 
    vector<RotatedRect> *fiducialEngine::getClusters(){
        vector<RotatedRect> *temp;
        pthread_mutex_lock(&(this->sharedLock));
        temp = this->shared.clusters;
        pthread_mutex_unlock(&(this->sharedLock));

        return temp;
    }

    void fiducialEngine::feedImage(Mat image){
        pthread_mutex_lock(&(this->sharedLock));
        this->shared.sceneIn = image;
        pthread_mutex_unlock(&(this->sharedLock));
    }

    void *fiducialEngine::whiteFilter(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        Mat sceneOut[2], sceneIn;
        bool activeBuff;

        sceneOut[0] = Mat::zeros(sceneIn.size(), CV_8UC3);
        sceneOut[1] = Mat::zeros(sceneIn.size(), CV_8UC3);


        pthread_mutex_lock(&(engine->sharedLock));
        while(1){
        // Change state : enabled
        engine->shared.wf_state = enabled;
            while(1){
                pthread_cond_wait(&(engine->step_sig), &(engine->sharedLock));
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.wf_state = stopped;
                    pthread_mutex_unlock(&(engine->sharedLock));
                    return NULL;
                }
            }
            pthread_mutex_unlock(&(engine->sharedLock));

            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            if(!engine->shared.sceneIn.empty()){

                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.wf_state = processing;
                pthread_mutex_unlock(&(engine->sharedLock));

                activeBuff = !activeBuff; // Switch to other buffer.


                inRange(sceneIn, engine->settings.wf_lower, engine->settings.wf_upper, sceneOut[activeBuff]);

                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.wf_state = finished;
                pthread_mutex_unlock(&(engine->sharedLock));

                
            }

            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.
            
            pthread_mutex_lock(&(engine->sharedLock));
            if(!engine->shared.sceneIn.empty()) engine->shared.maskedScene = sceneOut[activeBuff];
        }
    }

    void *fiducialEngine::findCandidates(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        vector<vector<Point>> foundContours[2];
        vector<Vec4i> hierarchy;
        bool activeBuff;



        pthread_mutex_lock(&(engine->sharedLock));
        while(1){
            // Change state : enabled
            engine->shared.cd_state = enabled;
            while(1){
                pthread_cond_wait(&(engine->step_sig), &(engine->sharedLock));
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.cd_state = stopped;
                    pthread_mutex_unlock(&(engine->sharedLock));
                    return NULL;
                }
            }
            pthread_mutex_unlock(&(engine->sharedLock));

            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            if(!engine->shared.maskedScene.empty()){
                // Change state : processing
                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.cd_state = processing;
                pthread_mutex_unlock(&(engine->sharedLock));

                activeBuff = !activeBuff; // Switch to other buffer.
                foundContours[activeBuff].clear(); // Empty old buffer.


                findContours(engine->shared.maskedScene, foundContours[activeBuff], hierarchy, RETR_FLOODFILL, CHAIN_APPROX_SIMPLE);
                
                // Change state : finished
                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.cd_state = finished;
                pthread_mutex_unlock(&(engine->sharedLock));

                
            }

            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.
                
            pthread_mutex_lock(&(engine->sharedLock));
            if(!engine->shared.maskedScene.empty()) engine->shared.contours = &foundContours[activeBuff];
        }
    }

    void *fiducialEngine::findClusters(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        bool activeBuff;

        RotatedRect approxShape;
        vector<RotatedRect> canidates;
        vector<RotatedRect> clusters[2];
        


        pthread_mutex_lock(&(engine->sharedLock));
        while(1){
        engine->shared.cl_state = enabled;
            while(1){
                pthread_cond_wait(&(engine->step_sig), &(engine->sharedLock));
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.cl_state = stopped;
                    pthread_mutex_unlock(&(engine->sharedLock));
                    return NULL;
                }
            }
            pthread_mutex_unlock(&(engine->sharedLock));

            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            if(engine->shared.contours != nullptr){

                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.cl_state = processing;
                pthread_mutex_unlock(&(engine->sharedLock));

                activeBuff = !activeBuff; // Switch to other buffer.
                clusters[activeBuff].clear(); // Empty old buffer.

                for(vector<vector<Point>>::iterator i = (*(engine->shared).contours).begin(); i != (*engine->shared.contours).end(); i++){
                    
                    if( (*i).size() >= 4 &&
                        inBounds(arcLength((*i), true), 100, 2000) && // hierarchy[i][2] >= 0 == TRUE when closed
                        inBounds(contourArea((*i)), 200, 100000))
                    {
                        approxShape = minAreaRect(*i);

                        canidates.push_back(approxShape);
                        
                        //if(engine->settings.cl_debug) drawContours(workspace, vector<vector<Point> >(1,approxShape), 0, Scalar(0,0,255));

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
                                inBounds((*outerCanidate).size.area() / (*innerCanidate).size.area(), engine->settings.cl_ratioLower, engine->settings.cl_ratioUpper)) // If ratio of area is adequate
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

                pthread_mutex_lock(&(engine->sharedLock));
                engine->shared.cl_state = finished;
                pthread_mutex_unlock(&(engine->sharedLock));

                
            } 

            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.

            pthread_mutex_lock(&(engine->sharedLock));
            if(engine->shared.contours != nullptr){
                engine->shared.clusters = &clusters[activeBuff];
                engine->shared.newData = true;
            }

            pthread_cond_broadcast(&(engine->newData_sig));
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