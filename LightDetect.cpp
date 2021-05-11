#include <LightDetect.h>


LightDetect::LightDetect(float thr_front, float thr_dir, float thr_far){

	this->thr1 = thr_front;
	this->thr2 = thr_dir;
	this->thr3 = thr_far;
	
	this->filterlenth = 5;
	this->PT_LL = new Phototransistor(PT[0],filterlenth);
	this->PT_LC = new Phototransistor(PT[1],filterlenth);
	this->PT_RC = new Phototransistor(PT[2],filterlenth);
	this->PT_RR = new Phototransistor(PT[3],filterlenth);

}


bool LightDetect::detect_front(){
	if (PT_LC->getAverageReading()+PT_RC->getAverageReading() > this->thr1){
		return true;
	} else {
		return false;
	}
}

float LightDetect::detect_dir(){
	// should use fuzzy logic

	// left - right
	float error = PT_LL->getAverageReading()+PT_LC->getAverageReading() - 
			(PT_RC->getAverageReading() + PT_RR->getAverageReading()); 
	if (abs(error) < thr2) {
		return 0; // ok regine
	else{
		if (abs(error) < thr3){
			if (error > 0) {
				return -1; // light on the left
			} else{
				return 1; // light on the right 
			}
		} else{
			if (error > 0) {
				return -2; // light on the left very much 
			} else{
				return 2; // lith on the right very much
			}
		}
	}
	}

}