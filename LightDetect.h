/***********************************
* Filename: DtermineDirection.h
* Purpose: Track a detected light using a servo and phototransistors arranged in a line
* Date created: 3/May/2021
***********************************/

#include <Arduino.h>
#include <PinAllocation.h>
#include <Phototransistor.h>
// #include <Motions.h>

class LightDetect {
	public:

		LightDetect(float thr_front, float thr_dir_close, float thr_dir_far);

		//public functions
		bool detect_front();
		int detect_dir();
		int getPTAvg(void);

		float thr1;
		float thr2;
		float thr3;

		//deconstructor
		~LightDetect();


	private:

		// Array of phototransistor pins
		int PTPins[4] = { PHOTOTRANSISTOR1, PHOTOTRANSISTOR2, PHOTOTRANSISTOR3, PHOTOTRANSISTOR4 };
		Phototransistor* PT_LL; // left left phototransistor
		Phototransistor* PT_LC; // left center phototransistor
		Phototransistor* PT_RC; // right center phototransistor
		Phototransistor* PT_RR; // right right phototransistor

		int filterlenth;

};
