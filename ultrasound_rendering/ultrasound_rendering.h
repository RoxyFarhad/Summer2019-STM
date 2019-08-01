#pragma once
#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

//vector
#include <vector>


//array length constant
#define ARRAY_LEN 512



//struct for tx interval. holds the angle and the interval as array
struct tx_interval {
	double angle;
	char intensities[ARRAY_LEN + 1];
};
#endif