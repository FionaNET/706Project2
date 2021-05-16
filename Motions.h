#include <RobotBase.h>
#include <LightDetect.h>

class Motions {
    public:
        void rotate_while_scan();
        void obstical_avoid();



    private: 
        LightDetect* lightInfo = LightDetect();
        float obsticalThresh;
        bool backTracker;   //Tracks position of back wheels in regards to obsitcal



}