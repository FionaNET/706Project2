#include <LightDetect.h>
#include <Arduino.h>
#include <math.h>

LightDetect::LightDetect(){

	this->thr1 = 50; // front detect threshold


	this->PT_LL = new Phototransistor(PTPins[0]);
	this->PT_LC = new Phototransistor(PTPins[1]);
	this->PT_RC = new Phototransistor(PTPins[2]);
	this->PT_RR = new Phototransistor(PTPins[3]);

	this-> very_left_point1 = -100;
	this-> very_left_point2 = -80;
	this-> left_point1 = -90;
	this-> left_point2 = -55;
	this-> left_point3 = -5;
	this-> center_point1 = -10;
	this-> center_point2 = 0;
	this-> center_point3 = 10;
	this-> right_point1 = 5;
	this-> right_point2 = 55;
	this-> right_point3 = 90;
	this-> very_right_point1 = 80;
	this-> very_right_point2 = 100;	

}

LightDetect::~LightDetect(void)
{
	delete this->PT_LL;
	delete this->PT_LC;
	delete this->PT_RC;
	delete this->PT_RR;
}

bool LightDetect::detect_front(){
	//Serial.println("Dettected sum of two center phototransistor:");
	//Serial.println(PT_LC->getRawReading()+PT_RC->getRawReading());

	float dist_LC = PT_LC->getDistance();
	float dist_RC = PT_RC->getDistance(); 	
	float ave = 0;	


	if ((dist_LC+dist_RC)/2 > 500){
		// far
		this->thr1 = LIGHT_THRESH_FAR;
	} else{
		//close
		this->thr1 = LIGHT_THRESH_CLOSE;
	}

	ave =(PT_LC->getRawReading()+PT_RC->getRawReading()+PT_LL->getRawReading()+PT_RR->getRawReading())/4;
	// Serial.println("Raw reading of 4 phototransistor:");
	// Serial.print(PT_LL->getRawReading());
	// Serial.print("		");
	// Serial.print(PT_LC->getRawReading());
	// Serial.print("		");
	// Serial.print(PT_RC->getRawReading());
	// Serial.print("		");
	// Serial.println(PT_RR->getRawReading());

	

	// Serial.print("Average reading of 4 phototransistor:		");
	// Serial.println(ave);
	// if ((PT_LC->getRawReading()+PT_RC->getRawReading())/2 > 40){
	// 	ave = (PT_LC->getRawReading()+PT_RC->getRawReading())/2;
	// 	this->thr1 = 10;
	// }else{
	// 	ave = (PT_LL->getRawReading()+PT_RR->getRawReading())/2;
	// }

	if (ave > this->thr1){
		return true;
	} else {
		return false;
	}
}


float LightDetect::detect_dir(){
	// should use fuzzy logic

	// right-left
	// +ve go right
	// -ve go left
	float error =  (PT_RC->getAverageReading() + PT_RR->getAverageReading()) - (PT_LL->getAverageReading()+PT_LC->getAverageReading()); 


	// initialise the fuzzy value	
	float very_left;
	float left;
	float center;
	float right;
	float very_right;

	// membership functions

	// very_left membership function
	if (error < this->very_left_point1){
		very_left =1;
	} else if (error < this->very_left_point2){
		very_left = (this->very_left_point2 - error)/(this->very_left_point2 - this->very_left_point1);
	} else{
		very_left = 0;
	}

	// left membership function
	if (error < this->left_point1){
		left = 0;
	} else if (error < this->left_point2){
		left = (error - this->left_point1) / (this->left_point2 - this->left_point1);
	} else if (error < this->left_point3) {
		left = (this->left_point3 - error) / (this->left_point3 - this->left_point2);
	} else{
		left = 0;
	}

	// center membership function
	if (error < this->center_point1){
		center = 0;
	} else if (error < this->center_point2){
		center = (error - this->center_point1) / (this->center_point2 - this->center_point1);
	} else if (error < this->center_point3) {
		center = (this->center_point3 - error) / (this->center_point3 - this->center_point2);
	} else{
		center = 0;
	}	


	// right membership function
	if (error < this->right_point1){
		right = 0;
	} else if (error < this->right_point2){
		right = (error - this->right_point1) / (this->right_point2 - this->right_point1);
	} else if (error < this->right_point3) {
		right = (this->right_point3 - error) / (this->right_point3 - this->right_point2);
	} else{
		right = 0;
	}

	// very right membership function
	if (error < this->very_right_point1){
		very_right =0;
	} else if (error < this->very_right_point2){
		very_right = (error - this->very_right_point2)/(this->very_right_point2 - this->very_right_point1);
	} else{
		very_right = 1;
	}

	float weighted_ave = (-2*very_left -1*left + 0*center + 1*right + 2*very_right) / (very_left+left+center+right+very_right);
	return weighted_ave;


//	if (abs(error) < thr2) {
//		return 0; // ok region
//	else{
//		if (abs(error) < thr3){
//			if (error > 0) {
//				return -1; // light on the left
//			} else{
//				return 1; // light on the right 
//			}
//		} else{
//			if (error > 0) {
//				return -2; // light on the left very much 
//			} else{
//				return 2; // lith on the right very much
//			}
//		}
//	}
//	}

}

int LightDetect::getPTAvg(void)
{
	int Avg = 0;
	Avg += this->PT_LL->getAverageReading();
	Avg += this->PT_LC->getAverageReading();
	Avg += this->PT_RC->getAverageReading();
	Avg += this->PT_RR->getAverageReading();
	Avg = Avg/4;
	return Avg;
}