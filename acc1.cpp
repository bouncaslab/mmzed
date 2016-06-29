/*
 * acc.cpp
 *
 *  Created on: Jun 16, 2016
 *      Author: alp
 */

#include "acc1.h"
#include <cmath>
//#include <chrono>
#define PI 3.14159265
#define R 6372795.477598
#define MAXIMUMDISTANCE


void distancetolinefrompoint(float arr1[10], float dist[3]) {
	float arr[10];
#pragma HLS ARRAY_PARTITION variable=arr dim=1 complete

//#pragma HLS ARRAY_PARTITION variable=arr cyclic factor=8 dim=1
	float distoutA[3];
#pragma HLS ARRAY_PARTITION variable=distoutA dim=1 complete

	float distoutB[3];
#pragma HLS ARRAY_PARTITION variable=distoutB dim=1 complete

	float distoutC[3];
#pragma HLS ARRAY_PARTITION variable=distoutC dim=1 complete

	//auto start = std::chrono::system_clock::now();

	for (int i = 0; i < 10; i++) {
#pragma HLS PIPELINE
		arr[i] = arr1[i];
	}
	float lat1 = arr[6];
	float long1 = arr[7];
	float lat2 = arr[8];
	float long2 = arr[9];

	for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
	float carlat = arr[2*i];
	float carlong = arr[(2*i)+1];
	float l2l1 = long2 - long1;
	float la2la1 = lat2 - lat1;
	 distoutA[i] = fabs(
			((l2l1) * carlat) - ((la2la1) * carlong) + (lat2 * long1)
					- (long2 * lat1));
	distoutB[i] = sqrtf((l2l1 * l2l1) + (la2la1 * la2la1));
	}

	for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE
		distoutC[i] = distoutA[i]/distoutB[i];
	}
	//auto stop = std::chrono::system_clock::now();
	//std::chrono::duration<float, std::micro>
	//time = stop - start;
	//cout << time.count() << "  ms for distance"<<endl;
	for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE
		dist[i] = distoutC[i];
	}
}

void distancetopointfrompoint(float arr1[8], float dist[3]) {
	float arr[8];
#pragma HLS ARRAY_PARTITION variable=arr dim=1 complete
	float distoutA[3];
#pragma HLS ARRAY_PARTITION variable=distoutA dim=1 complete
	//auto start = std::chrono::system_clock::now();

	for (int i = 0; i < 8; i++) {
#pragma HLS PIPELINE
		arr[i] = arr1[i];
	}
	float lat1 = arr[6];
	float long1 = arr[7];
	for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
	float carlat = arr[2*i];
	float carlong = arr[(2*i)+1];
	float lat2lat1 = lat1 - carlat;
	float long1long2 = long1 - carlong;
	 distoutA[i] = sqrtf((lat2lat1 * lat2lat1) + (long1long2 * long1long2));
	}
	for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE
		dist[i] = distoutA[i];
	}
}

void haversine(float arr1[8], float dist1[3]) {
//#pragma HLS PIPELINE
	float arr[8];
#pragma HLS ARRAY_PARTITION variable=arr dim=1 complete

//#pragma HLS ARRAY_PARTITION variable=arr cyclic factor=10 dim=1
	float distout[3];
#pragma HLS ARRAY_PARTITION variable=distout dim=1 complete

//#pragma HLS ARRAY_PARTITION variable=distout cyclic factor=4 dim=1

	for (int i = 0; i < 8; i++) {
#pragma HLS PIPELINE
		arr[i] = arr1[i]* PI / 180;
	}
//#pragma HLS PIPELINE

	//auto start = std::chrono::system_clock::now();
	//call your function here
	float lat2 = arr[6] * PI / 180;
	float lon2 = arr[7] * PI / 180;

	for (int i = 0; i < 3; i++) {
#pragma HLS UNROLL
		float lat1 = arr[2 * i];
		float lon1 = arr[(2 * i) + 1];
		float dlon = (lon2 - lon1) / 2;
		float dlat = (lat2 - lat1) / 2;
		float a = (sinf(dlat) * sinf(dlat))
				+ (cosf(lat1) * cosf(lat2) * sinf(dlon) * sinf(dlon));
		float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
		distout[i] = R * c;
	} //where R is the radius of the Earth
	  //auto stop = std::chrono::system_clock::now();
	  //std::chrono::duration<float, std::milli>
	  //time = stop - start;
	  //cout << fixed << setprecision(6)
	  //<< time.count() << "  ms for havesine"<<endl;
	for (int i = 0; i < 3; i++) {
#pragma HLS PIPELINE
		dist1[i] = distout[i];
	}
}
