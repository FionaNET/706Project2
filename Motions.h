#include <RobotBase.h>
#include <LightDetect.h>
#include <PinAllocation.h>

class Motions {
    public:
        Motions();
        void rotate_while_scan();
        void go_target();
        void rotate_angle(folat ref_angle);



    private: 
        LightDetect* lightInfo;






}