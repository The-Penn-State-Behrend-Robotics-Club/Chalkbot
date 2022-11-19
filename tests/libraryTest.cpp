#include "markerTracking.h"

using namespace cv;
using namespace std;

bool inBounds(int i, int min, int max){
    if(i >= min && i <= max) return true;
    return false;
}

int main(){

    Mat frame;
    Point marker;

    VideoCapture cap;
    cap.open(0);

    if(!cap.isOpened()){
        cerr << "ERROR! Unable to open camera" << endl;
        return -1;
    }

    while(1){

        cap.read(frame);
        if(frame.empty()){
            cerr << "ERROR! blank frame grabbed" << endl;
            break;
        }
        imshow("Live",frame);
        if(waitKey(5) >= 0) break;

        marker = ft::findMarker(&frame, false,false);
        cout << "Point: " << marker.x << " " << marker.y << endl;
        rectangle(frame, Rect(marker.x - 20, marker.y - 20, 40, 40), Scalar(255,0,0));

        imshow("Point",frame);
    }

    return 0;

}