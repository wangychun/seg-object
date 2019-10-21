/*
 * gaus_blur.h
 *
 *  Created on: Sep 22, 2019
 *      Author: sarah
 */

#ifndef GAUS_BLUR_H_
#define GAUS_BLUR_H_

#include <vector>
#include <array>

#include "gridmap.h"

double gauss(double sigma, double x);

std::vector<double> gaussKernel(int samples, double sigma);

void gaussSmoothen(std::array<Cell, numX>& values, double sigma, int samples);

#endif /* GAUS_BLUR_H_ */
