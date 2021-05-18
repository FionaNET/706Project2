/***********************************
* Filename: DtermineDirection.h
* Purpose: Track a detected light using a servo and phototransistors arranged in a line
* Date created: 3/May/2021
***********************************/
#ifndef LightDetect_h
#define LightDetect_h

#include <Arduino.h>
#include "PinAllocation.h"
#include "Phototransistor.h"
// #include <Motions.h>

class LightDetect {
	public:

		LightDetect();

		//public functions
		bool detect_front();
		float detect_dir();
		int getPTAvg(void);


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

		float thr1; // threshold for the front
		// shape parameters
		float very_left_point1;
		float very_left_point2;
		float left_point1;
		float left_point2;
		float left_point3;
		float center_point1;
		float center_point2;
		float center_point3;
		float right_point1;
		float right_point2;
		float right_point3;
		float very_right_point1;
		float very_right_point2;

};

#endif
