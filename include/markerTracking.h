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

using namespace std;
using namespace cv;

namespace ft{ //fiducial tracker

    typedef struct fidSettings_t{
        bool wf_debug, cd_debug, cl_debug;

        Scalar wf_upper;
        Scalar wf_lower;

        int cl_ratioUpper;
        int cl_ratioLower;
    } fidSettings;


    class fiducialEngine{
        private:
        bool started;

        //pthread_t controller_pt;
        pthread_t whiteFilter_pt;
        pthread_t findCanidates_pt;
        pthread_t findClusters_pt;

        pthread_barrier_t startBarrier, endBarrier;
        pthread_mutex_t control_lock, scene_lock, masked_lock, contour_lock, cluster_lock;
        pthread_cond_t step_sig;

        enum threadState{stopped = 0, enabled = 1, processing = 2, finished = 3};

        struct {
            // control_lock
            bool run;
            bool exit;
            threadState wf_state, cd_state, cl_state;

            
            Mat sceneIn; // scene_lock
            Mat maskedScene; // masked_lock
            vector<vector<Point>> *(contours); // contour_lock
            vector<RotatedRect> *(clusters); // cluster_lock
        }shared;

        //void *engineController(void *args);
        static void *whiteFilter(void *args);
        static void *findCandidates(void *args);
        static void *findClusters(void *args);

        public:

        fiducialEngine();
        fiducialEngine(fidSettings settings);
        ~fiducialEngine();

        fidSettings settings;

        bool start();
        bool stop();
        void step();
        vector<RotatedRect> getClusters();
        void feedImage(Mat image);
        

    };



cv::Point findMarker(cv::Mat *scene, bool preserveInput, bool debug = false);

}