/*
 * acc.h
 *
 *  Created on: Jun 16, 2016
 *      Author: alp
 */

#ifndef ACC_H_
#define ACC_H_

//#pragma SDS data mem_attribute(arr:NON_CACHEABLE)
//#pragma SDS zero_copy(arr[0:8],dist[0:3])
//#pragma SDS data access_pattern(arr:SEQUENTIAL, dist:SEQUENTIAL)
//#pragma SDS data mem_attribute(arr1:CACHEABLE)

//void haversine(float arr1[8],float dist1[3]);

//#pragma SDS data mem_attribute(arr1:CACHEABLE)
//#pragma SDS data access_pattern(arr1:SEQUENTIAL, dist:SEQUENTIAL)
#pragma SDS data mem_attribute (arr1:PHYSICAL_CONTIGUOUS,dist:PHYSICAL_CONTIGUOUS)
void distancetolinefrompoint(float arr1[10], float dist[3]);

//#pragma SDS data access_pattern(arr1:SEQUENTIAL, dist:SEQUENTIAL)
#pragma SDS data mem_attribute (arr1:PHYSICAL_CONTIGUOUS,dist:PHYSICAL_CONTIGUOUS)
void distancetopointfrompoint(float arr1[8], float dist[3]);


#endif /* ACC_H_ */
