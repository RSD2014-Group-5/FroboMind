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
//#include "cv_functions.h"




using namespace cv;
using namespace std;



// Define constants

// Camera focal length
float f=320;
// Distance from camera axis to the plane cm
float dist=0.40;
float y_offset=0.350;
// Angle of rotation of x to put z pointing down
float alpha=25 * 3.141592654/180;

float alpha_rot=alpha-3.141592654;


//Matrix<double, 4, 4> transform;

float a[4][4] = {{ 1, 0, 0, 0},
		 { 0, cos(alpha_rot), -sin(alpha_rot), 0},
		 { 0, sin(alpha_rot), cos(alpha_rot), 0},
		 { 0, 0, 0, 1}};

Mat trans = Mat(4, 4, CV_32FC1, a);



// Merge lines
// Tolerance for angle
float m=0.7;
// tolerance for distance
//float d=800.0;


vector<Vec2f> transform_point(vector<Vec2f> lines,int rows, int cols)
{
	//cout<<"Image: size"<<cols<<" "<<rows <<endl;
	vector<Vec2f> points_uv;

    for (int i=0;i<lines.size();i++)
    {
        float rho = lines[i][0];
		float theta = lines[i][1];
		double a = cos(theta), b = sin(theta);
		double x0 = a*rho, y0 = b*rho;
	       
		// Point 1 from line
		Vec2f point;
		point[0]=x0 + 200*(-b) - (float)cols/2;
		point[1]=y0 + 200*(a) - (float)rows/2;
		points_uv.push_back(point);
		//cout<<"Point1: "<<point[0]<<" , "<<point[1]<<endl;

		// Point 2 from line
		point[0]=x0 - 200*(-b)- (float)cols/2;
		point[1]=y0 - 200*(a)- (float)rows/2;
		points_uv.push_back(point);
		//cout<<"Point2: "<<point[0]<<" , "<<point[1]<<endl;
    }
	
	// Now we have the pixels computed	

	// Translate to xyz coordinates
	// For every point

	vector<Vec2f> output;

	for(int i=0;i<points_uv.size();i++)
	{	
		float u=points_uv[i][0];
		float v=points_uv[i][1];
		float point[4];
		double y=v* dist / (f*cos(alpha) + v*sin(alpha) );
		double z=(dist-y*sin(alpha))/cos(alpha);
		double x=u*z/f;
		
		point[0]=x;
		point[1]=y;
		point[2]=z;		
		point[3]=1.0;	
		
		//Put them in homogenous vector
		Mat coordinate = Mat(4, 1, CV_32FC1,point);
		Mat new_coordinate= trans * coordinate;		
		
		Vec2f output_point;
		// Put x coordinate
		output_point[0]=new_coordinate.at<float>(0,0);
		output_point[1]=new_coordinate.at<float>(1,0)+y_offset;

		// Push back to output
		output.push_back(output_point);
	
	}	

	return output;
}


vector<Vec2f> merge_lines(vector<Vec2f> lines)
{
		 // Merge the lines that are close and nearly parallel and compute mean of them.
		 // This function it's for avoiding sending to intersection calculator the same real-line
		 // represented by more than 1 hough-detected line
		 float divider=1.0;
		 float theta;
		 float distance;
		 float pi=3.141592654;

		// all lines goes from -90 to 90 degrees
		 for (uint i=0;i<lines.size();i++)
		 {
			// cout<<"Lines "<<i<<"= "<<lines[i][0]<<","<<lines[i][1]<<endl;
			 // Change sign of negative lines
			 if( (lines[i][1]<0))
			 {
				 lines[i][0]=-lines[i][0];
				 lines[i][1]=lines[i][1]+pi;
			 }
			
		}

		 vector<Vec2f> lines_merged;

		if(lines.size()>0)
		{
		 lines_merged.push_back(lines[0]);
		 lines.erase(lines.begin());
		}	
		 while(lines.size()!=0)
		 {
			// cout<<"Size of lines"<<lines.size()<<endl;;
			 divider=1.0;
			 for (uint i=0;i<lines.size();i++)
				 {

					 theta=lines[i][1];
					 distance=lines[i][0];
					 //cout<<"line_merged :"<<lines_merged.back()[0]<<","<<lines_merged.back()[1]<<" Line :"<<lines[i][0]<<","<<lines[i][1]<<endl;
					

					 // If the lines are parallel
					 // Normal case
					 if( (lines_merged.back()[1]<divider*(theta+m)) and (lines_merged.back()[1]>divider*(theta-m)))
					 {
	//					 cout<<"i: "<<i<<" lines_merg_ing :"<<lines_merged.size()-1<<endl;

						lines_merged.back()[0]+=distance;
						lines_merged.back()[1]+=theta;					
						lines.erase(lines.begin()+i);
						 i--;
						 divider++;

					 }
					// Case line_merged 170 and line 10
					if( (lines_merged.back()[1]<divider*(theta+pi+m)) and (lines_merged.back()[1]>divider*(theta+pi-m)))					{
						lines_merged.back()[0]-=distance;
						lines_merged.back()[1]+=theta+pi;					
						lines.erase(lines.begin()+i);
						 i--;
						 divider++;


					}
					// Case line_merged 10 and line 170
					if( (lines_merged.back()[1]<divider*(theta-pi+m)) and (lines_merged.back()[1]>divider*(theta-pi-m)))					{
						lines_merged.back()[0]-=distance;
						lines_merged.back()[1]+=theta-pi;					
						lines.erase(lines.begin()+i);
						 i--;
						 divider++;


					}	

					

				 }
				 lines_merged.back()[0]/=divider;
				 lines_merged.back()[1]/=divider;

				 if(lines.size()==0)
								 break;
				 //Push next one
				 lines_merged.push_back(lines[0]);
				 lines.erase(lines.begin());

		 }
		/*
		 for (int i=0;i<lines_merged.size();i++)
			 cout<<"Lines_merged "<<i<<"= "<<lines_merged[i][0]<<","<<lines_merged[i][1]*180/3.14<<endl;
		*/

		 return lines_merged;
}

/*
vector<Vec2f> merge_lines_old(vector<Vec2f> lines)
{
		 // Merge the lines that are close and nearly parallel and compute mean of them.
		 // This function it's for avoiding sending to intersection calculator the same real-line
		 // represented by more than 1 hough-detected line
		 float divider=1.0;
		 float theta;
		 float distance;

		 for (uint i=0;i<lines.size();i++)
		 {
			// cout<<"Lines "<<i<<"= "<<lines[i][0]<<","<<lines[i][1]<<endl;
			 // Change sign of negative lines
			 if(lines[i][0]<0)
			 {
				 lines[i][0]=-lines[i][0];
				 lines[i][1]=lines[i][1]-3.141592654;
			 }
		}
		 vector<Vec2f> lines_merged;

		if(lines.size()>0)
		{
		 lines_merged.push_back(lines[0]);
		 lines.erase(lines.begin());
		}	
		 while(lines.size()!=0)
		 {
			// cout<<"Size of lines"<<lines.size()<<endl;;
			 divider=1.0;
			 for (uint i=0;i<lines.size();i++)
				 {

					 theta=lines[i][1];
					 distance=lines[i][0];
					 //cout<<"line_merged :"<<lines_merged.back()[0]<<","<<lines_merged.back()[1]<<" Line :"<<lines[i][0]<<","<<lines[i][1]<<endl;
					 // If the lines are parallel and close make promedium
					 if((lines_merged.back()[0]<divider*(distance+d))and(lines_merged.back()[0]>divider*(distance-d))and(lines_merged.back()[1]<divider*(theta+m))  and(lines_merged.back()[1]>divider*(theta-m)) )
					 {
	//					 cout<<"i: "<<i<<" lines_merg_ing :"<<lines_merged.size()-1<<endl;
						 lines_merged.back()+=lines[i];
						 lines.erase(lines.begin()+i);
						 i--;
						 divider++;
	//
					 }
				 }
				 lines_merged.back()[0]/=divider;
				 lines_merged.back()[1]/=divider;

				 if(lines.size()==0)
								 break;
				 //Push next one
				 lines_merged.push_back(lines[0]);
				 lines.erase(lines.begin());

		 }
		
		 for (int i=0;i<lines_merged.size();i++)
			 cout<<"Lines_merged "<<i<<"= "<<lines_merged[i][0]<<","<<lines_merged[i][1]*180/3.14<<endl;
		

		 return lines_merged;
}

*/

Vec2f intersection(Vec2f line1,Vec2f line2)
{
		// COmpute intersection point between two lines
    	    float ct1=cosf(line1[1]);     //matrix element a
	    float st1=sinf(line1[1]);     //b
	    float ct2=cosf(line2[1]);     //c
	    float st2=sinf(line2[1]);	    //d
	    float r1=line1[0];
	    float r2=line2[0];
	    float dis=ct1*st2-st1*ct2;        //determinative (rearranged matrix for inverse)
	    float x,y;
	    if(dis!=0.0f) {
	            x=((st2*r1-st1*r2)/dis);
	            y=((-ct2*r1+ct1*r2)/dis);

	    } else { //lines are parallel and will NEVER intersect!
	          //  cout<<"Lines are parallel"<<endl;
	            return Vec2f(-1,-1);
	    }
	    Vec2f result;
	    result[0]=x;
	    result[1]=y;
	    return result;

}

void compute_intersections(vector<Vec2f> horizontal_lines,vector<Vec2f> vertical_lines,vector<Vec2f>& intersection_points,vector<Vec2f>& intersection_points_2)
{
	//Compute the intersection points on line_1 and line_2
		    for (int i=0;i<vertical_lines.size();i++)
			   {
				   // Compute intersection
				   Vec2f intersect=intersection(horizontal_lines[0],vertical_lines[i]);
				   intersection_points.push_back(intersect);
				   intersect=intersection(horizontal_lines[1],vertical_lines[i]);
				   intersection_points_2.push_back(intersect);
			   }
}

Mat display_lines(Mat image, vector<Vec2f> lines,string name )
{
	// Display on image the lines contained in lines.
	for( size_t i = 0; i < lines.size(); i++ )
	    {
	        float rho = lines[i][0];
	        float theta = lines[i][1];
	        //cout<<"line: "<<i<<"("<<rho<<","<<theta<<")"<<endl;
	        double a = cos(theta), b = sin(theta);
	        double x0 = a*rho, y0 = b*rho;
	        Point pt1(cvRound(x0 + 1000*(-b)),
	                  cvRound(y0 + 1000*(a)));
	        Point pt2(cvRound(x0 - 1000*(-b)),
	                  cvRound(y0 - 1000*(a)));
	        line( image, pt1, pt2, Scalar(0,0,255), 1, 8 );
	    }
        //imshow(name,image);
	return image;
}


