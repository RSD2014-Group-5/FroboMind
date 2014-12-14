/*
 * cv_functions.h
 *
 *  Created on: Dec 10, 2013
 *      Author: maik
 */

#ifndef CV_FUNCTIONS_H_
#define CV_FUNCTIONS_H_

/*
 * cv_functions.cpp
 *
 *  Created on: Dec 10, 2013
 *      Author: maik
 */

#include <iostream>
#include <sstream>
#include <time.h>
#include <stdio.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <stdio.h>

using namespace cv;
using namespace std;


// Marker B functions

// Transform between coordinate system
vector<Vec2f> transform_point(vector<Vec2f> input,int rows, int cols);

vector<Vec2f> merge_lines(vector<Vec2f> lines);

Vec2f intersection(Vec2f line1,Vec2f line2);

void compute_intersections(vector<Vec2f> horizontal_lines,vector<Vec2f> vertical_lines,vector<Vec2f>&intersection_points_1,vector<Vec2f>& intersection_points_2);

Mat display_lines(Mat image, vector<Vec2f> lines,string name );






#endif /* CV_FUNCTIONS_H_ */
