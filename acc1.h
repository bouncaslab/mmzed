/*
 * acc.h
 *
 *  Created on: Jun 16, 2016
 *      Author: alp
 */

#ifndef ACC_H_
#define ACC_H_




#pragma SDS data mem_attribute (arr1:PHYSICAL_CONTIGUOUS,dist:PHYSICAL_CONTIGUOUS)
void distancetolinefrompoint(float arr1[10], float dist[3]);

#pragma SDS data mem_attribute (arr1:PHYSICAL_CONTIGUOUS,dist:PHYSICAL_CONTIGUOUS)
void distancetopointfrompoint(float arr1[8], float dist[3]);


#endif /* ACC_H_ */
