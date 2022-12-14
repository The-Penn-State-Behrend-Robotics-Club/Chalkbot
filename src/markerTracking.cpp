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

        pthread_mutex_init(&(this->control_lock), NULL);
        pthread_mutex_init(&(this->scene_lock), NULL);
        pthread_mutex_init(&(this->masked_lock), NULL);
        pthread_mutex_init(&(this->contour_lock), NULL);
        pthread_barrier_init(&(this->startBarrier), NULL, 4);
        pthread_barrier_init(&(this->endBarrier), NULL, 4);
        pthread_cond_init(&(this->step_sig), NULL);


        this->shared.run = false;
        this->shared.exit = false;
        this->shared.wf_state = stopped;
        this->shared.cd_state = stopped;
        this->shared.cl_state = stopped;

        this->started = true;

        if(!(//(pthread_create(&(engine.controller_pt)    , NULL, &engineController , NULL) == 0) &&
                (pthread_create(&(this->whiteFilter_pt)   , NULL, &whiteFilter      , this) == 0) &&
                (pthread_create(&(this->findCanidates_pt)  , NULL, &findCandidates   , this) == 0) &&
                (pthread_create(&(this->findClusters_pt)  , NULL, &findClusters     , this) == 0)))
        {
            
            this->stop();

            return false;
        }
        
        return true;

    }

    bool fiducialEngine::stop(){

        if(!this->started) return true;

        pthread_mutex_lock(&(this->control_lock));
        this->shared.exit = true;
        pthread_cond_broadcast(&(this->step_sig));
        pthread_mutex_unlock(&(this->control_lock));

        int waiting = 0;
        while(1){
            if(this->shared.wf_state == threadState::stopped && this->shared.cd_state == threadState::stopped && this->shared.cl_state == threadState::stopped){
                break;
            }else {
                if(waiting > 1000){ // Escalate to killing (the whole process)
                    cout << "Couldn't kill fiducial engine." << endl;
                    exit(-1);
                    //pthread_kill(this->whiteFilter_pt, SIGINT);
                    //pthread_kill(this->findCanidates_pt, SIGINT);
                    //pthread_kill(this->findClusters_pt, SIGINT);
                    //this->shared.wf_state = stopped;
                    //this->shared.cd_state = stopped;
                    //this->shared.cl_state = stopped;
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

        pthread_mutex_lock(&(this->control_lock));
        if(!this->shared.sceneIn.empty()){
            do{
                this->shared.run = true;
                pthread_cond_broadcast(&(this->step_sig));
                pthread_mutex_unlock(&(this->control_lock));



                pthread_barrier_wait(&(this->startBarrier)); // Wait for all threads to be ready.

                pthread_mutex_lock(&(this->control_lock));
                this->shared.run = false;
                pthread_mutex_unlock(&(this->control_lock));

                pthread_barrier_wait(&(this->endBarrier)); // Wait for all threads to finish.

                pthread_mutex_lock(&(this->control_lock));
            } while(this->shared.clusters == nullptr);
        }
        pthread_mutex_unlock(&(this->control_lock));

        
    }
 
    vector<RotatedRect> fiducialEngine::getClusters(){
        vector<RotatedRect> temp;
        pthread_mutex_lock(&(this->cluster_lock));
        if(this->shared.clusters != nullptr)
            temp = *(this->shared.clusters);
        pthread_mutex_unlock(&(this->cluster_lock));

        return temp;
    }

    void fiducialEngine::feedImage(Mat image){
        pthread_mutex_lock(&(this->control_lock));
        this->shared.sceneIn = image;
        pthread_mutex_unlock(&(this->control_lock));
    }

    void *fiducialEngine::whiteFilter(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        Mat sceneOut[2];
        bool activeBuff, noData;

        while(1){

        pthread_mutex_lock(&(engine->control_lock));
        // Change state : enabled
        engine->shared.wf_state = enabled;
        //cout << "Wf enabled" << endl;
            while(1){
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.wf_state = stopped;
                    pthread_mutex_unlock(&(engine->control_lock));
                    return NULL;
                }
                pthread_cond_wait(&(engine->step_sig), &(engine->control_lock));
            }
            engine->shared.wf_state = processing;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Wf processing..." << endl;

            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            //cout << "Wf started!" << endl;


            pthread_mutex_lock(&(engine->scene_lock)); // Lock input data until finished
            noData = engine->shared.sceneIn.empty();
            if(!noData){

                //cout << "Wf working..." << endl;


                activeBuff = !activeBuff; // Switch to other buffer.

                if(sceneOut[activeBuff].size() != engine->shared.sceneIn.size()) sceneOut[activeBuff] = Mat::zeros(engine->shared.sceneIn.size(), CV_8U);

                inRange(engine->shared.sceneIn, engine->settings.wf_lower, engine->settings.wf_upper, sceneOut[activeBuff]);
                
            } 
            pthread_mutex_unlock(&(engine->scene_lock)); // Unlock input data

            pthread_mutex_lock(&(engine->control_lock));
            engine->shared.wf_state = finished;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Wf finished!" << endl;
                
            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.

            //cout << "Wf ended!" << endl;
            
            // Update output buff
            if(!noData){
                //cout << "Wf updating buffer." << endl;
                pthread_mutex_lock(&(engine->masked_lock));
                engine->shared.maskedScene = sceneOut[activeBuff]; // Pass output buffer to next input
                pthread_mutex_unlock(&(engine->masked_lock));
            }

        }
    }

    void *fiducialEngine::findCandidates(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        vector<vector<Point>> foundContours[2];
        vector<Vec4i> hierarchy;
        bool activeBuff, noData;

        while(1){

            pthread_mutex_lock(&(engine->control_lock));
            // Change state : enabled
            engine->shared.cd_state = enabled;
            //cout << "Fc enabled" << endl;
            while(1){
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.cd_state = stopped;
                    pthread_mutex_unlock(&(engine->control_lock));
                    return NULL;
                }
                pthread_cond_wait(&(engine->step_sig), &(engine->control_lock));
            }
            engine->shared.cd_state = processing;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Cf processing..." << endl;

            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            //cout << "Cf started!" << endl;

            pthread_mutex_lock(&(engine->masked_lock));
            noData = engine->shared.maskedScene.empty();
            if(!noData){

                //cout << "Cf working..." << endl;

                activeBuff = !activeBuff; // Switch to other buffer.
                foundContours[activeBuff].clear(); // Empty old buffer.

                findContours(engine->shared.maskedScene, foundContours[activeBuff], hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
                
            }
            pthread_mutex_unlock(&(engine->masked_lock)); // Unlock input data

            // Change state : finished
            pthread_mutex_lock(&(engine->control_lock));
            engine->shared.cd_state = finished;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Cf finished!" << endl;

            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.

            //cout << "Cf ended!" << endl;

            // Update output buff
            if(!noData){
                //cout << "Cf updating buffer." << endl;
                pthread_mutex_lock(&(engine->contour_lock));
                engine->shared.contours = &foundContours[activeBuff];
                pthread_mutex_unlock(&(engine->contour_lock));
            }

        }
    }

    void *fiducialEngine::findClusters(void *args){
        fiducialEngine *engine = static_cast<fiducialEngine*>(args);

        bool activeBuff, noData;

        RotatedRect approxShape;
        vector<RotatedRect> canidates;
        vector<RotatedRect> clusters[2];
        


        while(1){
        pthread_mutex_lock(&(engine->control_lock));
        engine->shared.cl_state = enabled;
        //cout << "Fc enabled" << endl;
            while(1){
                if(engine->shared.run == true) break;
                if(engine->shared.exit == true) {
                    engine->shared.cl_state = stopped;
                    pthread_mutex_unlock(&(engine->control_lock));
                    return NULL;
                }
                pthread_cond_wait(&(engine->step_sig), &(engine->control_lock));
            }
            engine->shared.cl_state = processing;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Fc processing..." << endl;
            
            pthread_barrier_wait(&(engine->startBarrier)); // Wait for all threads to be ready.

            //cout << "Fc started!" << endl;
            
            pthread_mutex_lock(&(engine->contour_lock));
            noData = engine->shared.contours == nullptr;
            if(!noData){

                //cout << "Fc working..." << endl;
                
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

            } 
            pthread_mutex_unlock(&(engine->contour_lock));
            
            // Change state : finished
            pthread_mutex_lock(&(engine->control_lock));
            engine->shared.cl_state = finished;
            pthread_mutex_unlock(&(engine->control_lock));

            //cout << "Fc finished!" << endl;
            
            pthread_barrier_wait(&(engine->endBarrier)); // Wait for all threads to finish.

            //cout << "Fc ended!" << endl;
            
            if(!noData){
                //cout << "Fc updating buffer." << endl;
                pthread_mutex_lock(&(engine->cluster_lock));
                engine->shared.clusters = &clusters[activeBuff];
                pthread_mutex_unlock(&(engine->cluster_lock));
            }

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