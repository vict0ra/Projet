#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h> 
#include <string>
#include <iostream>
#include <vector>
#include <algorithm> 

using namespace cv;
using namespace std;
#define PI 3.14159265

/* resizeshow(input image): visualize input image in the window of 300*300
*/

void resizeshow( Mat im, string window_name)
{
  Mat im_output;
  Size size(300,300);
  resize(im, im_output, size);
  imshow( window_name, im_output );
}



string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}
/* directionalgradient(image gradient in the direction x, image gradient in the direction y,
 *  number of direction taken into consideration)
 *  output directional gradient
*/
Mat directionalgradient(const Mat& src_gray, const Mat& grad_x, const Mat& grad_y, int directions_num){

    Mat atan_y_x;
    atan_y_x = Mat::zeros(grad_x.rows, grad_x.cols, CV_8U);//create zero matrix of CV_8U format
   
	int degree_grade(360);

    for (int i=0; i<atan_y_x.size[0]; i++ ){
		 for (int j=0;j<atan_y_x.size[1];j++){
			 
			atan_y_x.at<uchar>(i, j) = (((int)cv::fastAtan2(grad_y.at<short>(i, j), grad_x.at<short>(i, j) )) / (degree_grade/directions_num) + 1
            ) * (src_gray.at<short>(i, j) ? 1 : 0);
			
		 }
	}			
return atan_y_x;
}

//normalise gradient
void norm_gradient(const Mat& grad_x,const Mat& grad_y,Mat & grad_x_norm,Mat & grad_y_norm){
		
	for (unsigned int i=0;i<grad_x.size[0];i++) {
		for (unsigned int j=0;j<grad_x.size[1];j++)
		{   float den(0);
			den = sqrt (pow(grad_x.at<short>(i,j)*1.0,2.0)+pow(grad_y.at<short>(i,j)*1.0,2.0));
			if (den!=0){
			grad_x_norm.at<float>(i,j) = grad_x.at<short>(i,j)/den;
			grad_y_norm.at<float>(i,j) = grad_y.at<short>(i,j)/den ;
		}
		else {grad_x_norm.at<float>(i,j)=0;
			 grad_y_norm.at<float>(i,j)=0;}
		}
		
	}
}

vector< vector<int> > archieve_edge(const Mat& edgesCanny, const Mat & grad_x_norm, const Mat & grad_y_norm,unsigned int coor[2],float pas){
	
	vector< vector<int> > stock_r(0, std::vector<int>(2));
	vector< vector<int> > empty_vec(0, std::vector<int>(2));
	float n(1.0);
	do{
		n +=pas;
		float r[2]={0.0, 0.0};
		r[0] = coor[0]*1.0+n*grad_x_norm.at<float>(coor[0],coor[1]);
		r[1] = coor[1]*1.0+n*grad_y_norm.at<float>(coor[0],coor[1]);
		//cout<<" Test r[0]"<<r[0];
		vector<int> re(2);
		re[0]=int(r[0]);
		re[1]=int(r[1]);
		//cout<<"re "<<re[0]<<" "<<re[1]<<endl;
			if (re[0]>0 && re[0]<edgesCanny.size[0]){
				if (re[1]>0 && re[1]<edgesCanny.size[1]){
					if (stock_r.size()==0 || equal(stock_r.back().begin(), stock_r.back().end(), re.begin())==0){		// if the last element is not the same 
					stock_r.push_back(re);
					}
				}
				else return empty_vec;
			}
			else return empty_vec;
	} while (edgesCanny.at<uchar>(stock_r.back().front(),stock_r.back().back())!=255);
/*
float dp_pi6_x(0),dp_pi6_y(0),dp_l_x(0),dp_l_y(0);
dp_pi6_x = -grad_x_norm.at<float>(coor[0],coor[1])+(float)cos (30.0*PI/180.0 );
dp_pi6_y = -grad_y_norm.at<float>(coor[0],coor[1])+(float)sin (30.0*PI/180.0 );
//cout<<"dp + 30 :"<< dp_pi6_x<<" "<<dp_pi6_y<<endl;
dp_l_x = -grad_x_norm.at<float>(coor[0],coor[1])+(float)cos (30.0*PI/180.0 );
dp_l_y = -grad_y_norm.at<float>(coor[0],coor[1])-(float)sin (30.0*PI/180.0 );
//cout<<"dp - 30: "<< dp_l_x<<" "<<dp_l_y<<endl;
float d_q_x(0);
float d_q_y(0);
d_q_x = grad_x_norm.at<float>(stock_r.back().front(),stock_r.back().back());
d_q_y = grad_y_norm.at<float>(stock_r.back().front(),stock_r.back().back());
//cout<<"dq"<< d_q_x<<" "<<d_q_y<<endl;
float eps =0.2;
if ( abs(d_q_x-dp_pi6_x)<eps && (abs(d_q_y-dp_pi6_y)<eps||abs(d_q_y-dp_l_y)<eps)) return stock_r;
else return empty_vec;
*/

vector <int> dp(2), dq(2);
dp[0] = -grad_x_norm.at<float>(coor[0],coor[1]);
dp[1] = -grad_y_norm.at<float>(coor[0],coor[1]);
dq[0] = grad_x_norm.at<float>(stock_r.back().front(),stock_r.back().back());
dq[1] = grad_y_norm.at<float>(stock_r.back().front(),stock_r.back().back());
//cout<<"Acos : "<< acos(dp[0]*dq[0]+dp[1]*dq[1])<<endl;
if (acos(dp[0]*dq[0]+dp[1]*dq[1])-0.001<PI/2.0) return stock_r;
else return empty_vec;

}
 
 
Mat Ray_images(const Mat& detected_edges,const Mat& src,const Mat& grad_x_norm, const Mat& grad_y_norm){
	
	Mat stock_vis;	
	stock_vis = Mat::zeros(src.size[0],src.size[1],CV_8U);
	unsigned int a[2];
	for(unsigned int i=0;i<src.size[0];i++){
		for(unsigned int j=0;j<src.size[1];j++){
			stock_vis.at<uchar>(i,j)=255;
		}
	}			 	 
	for (unsigned int i=0;i<detected_edges.size[0];i++){
		for (unsigned int j=0;j<detected_edges.size[1];j++){
			if (detected_edges.at<uchar>(i,j)==255){
			 a[0]=i; a[1]=j;
			vector< vector<int> > stock_r;
			stock_r = archieve_edge(detected_edges, grad_x_norm, grad_y_norm, a, 1);		 
			 if (stock_r.size()!=0){
				// n+=1;
				 //cout<<"Size stock "<<stock_r.size()<<endl;	
				 for (unsigned int i=0;i<stock_r.size();i++){
					stock_vis.at<uchar>(stock_r[i][0],stock_r[i][1])=0;
				 }		 	 
				}
			}
	 	}
	}
	resizeshow(stock_vis," Rays ");
	return stock_vis;
} 
Mat SWT(const Mat& detected_edges,const Mat& src,const Mat& grad_x_norm, const Mat& grad_y_norm){
	
	Mat stock_vis;	
	stock_vis = Mat::zeros(src.size[0],src.size[1],CV_32F);
	unsigned int a[2];
	for(unsigned int i=0;i<src.size[0];i++){
		for(unsigned int j=0;j<src.size[1];j++){
			stock_vis.at<float>(i,j)=255;//src.size[0]*src.size[1];
			//cout<<"Values of stock_vis"<<stock_vis.at<float>(i,j)<<endl;
		}
	}			 	 
	for (unsigned int i=0;i<detected_edges.size[0];i++){
		for (unsigned int j=0;j<detected_edges.size[1];j++){
			if (detected_edges.at<uchar>(i,j)==255.0){
			 a[0]=i; a[1]=j;
			vector< vector<int> > stock_r;
			stock_r = archieve_edge(detected_edges, grad_x_norm, grad_y_norm, a, 0.5);		 
			 if (stock_r.size()!=0){
				 float med(0);
				 for (unsigned int i=0;i<stock_r.size();i++){
						 if (stock_vis.at<float>(stock_r[i][0],stock_r[i][1])>stock_r.size()){	
						stock_vis.at<float>(stock_r[i][0],stock_r[i][1])=stock_r.size();
						}
						med += stock_vis.at<float>(stock_r[i][0],stock_r[i][1]); 
				    }
				    med = med/stock_r.size();
				    for (unsigned int i=0;i<stock_r.size();i++) stock_vis.at<float>(stock_r[i][0],stock_r[i][1])=med;		 	 
				}
			}
	 	}
	}

	float mean1;
	int nt(0);
	for (unsigned int i=0;i<detected_edges.size[0];i++){
		for (unsigned int j=0;j<detected_edges.size[1];j++){
			if (stock_vis.at<float>(i,j)!=255) {mean1 +=stock_vis.at<float>(i,j);nt+=1;}
		}
	}
	mean1 = mean1/(nt*1.0);
		for (unsigned int i=0;i<detected_edges.size[0];i++){
		for (unsigned int j=0;j<detected_edges.size[1];j++){
			if (stock_vis.at<float>(i,j)>mean1) {stock_vis.at<float>(i,j)=255.0;}
		}
	}
	Mat swt_image;
	normalize(stock_vis, swt_image, 0, 255, NORM_MINMAX, CV_8UC1);
	//resizeshow(swt_image," SWT ");
	return swt_image;
} 
int main( int argc, char** argv )
{
	Mat src;
		
	/// Load an image
	src = imread( argv[1] );

	if( !src.data )
	{ return -1; }
	
	Mat src_gray(src.size[0],src.size[1],CV_8U,0);	
	
	//cv::Mat *src_gray = new cv::Mat(src.size[0],src.size[1],CV_8U,0);	
	//delete src_gray;
	//src_gray = 0;
	   
   //some parameters
   
	int edgeThresh = 1;
	//int lowThreshold;
	int lowThreshold = 50;
	int const max_lowThreshold = 320;
	int ratio = 3;
	int kernel_size = 3;
	//char* window_name = "Edge Map";
	
	  /// Create a matrix of the same type and size as src (for dst)
	  // Mat dst;
	  // dst.create( src.size(), src.type() );

	  /// Convert the image to grayscale
	  cvtColor( src, src_gray, CV_BGR2GRAY );
	  
	  Mat detected_edges;
	  /// Reduce noise with a kernel 3x3
	  blur( src_gray, detected_edges, Size(3,3) );
	  
	  /// Create a window
	  //namedWindow( window_name, CV_WINDOW_AUTOSIZE );
	  
	  ///Canny edge detection
	  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
	  resizeshow(detected_edges,"Canny detector");

	  //bitwise_not ( detected_edges, detected_edges);	  //inverse Canny edge detector
	  //resizeshow(detected_edges,"Canny detector inverse");
	  
	//  std::cout<<"Test Canny"<<(int)detected_edges.at<uchar>(1,1)<<std::endl;

		
	//string ty =  type2str( detected_edges.type() );
    //printf("Matrix: %s %dx%d \n", ty.c_str(), detected_edges.cols, detected_edges.rows );
    
	  //resizeshow(detected_edges,"Directional gradient");
	  
	 Mat grad_x, grad_y;
	 
	 /// Gradient X
	 Sobel( src_gray, grad_x, CV_16S, 1, 0, 3, 1, 0, BORDER_DEFAULT );
	 /// Gradient Y
	 Sobel( src_gray, grad_y, CV_16S, 0, 1, 3, 1, 0, BORDER_DEFAULT );
	 
	 //normalise gradient
	 Mat grad_x_norm(grad_x.rows, grad_x.cols,CV_32F,0.0);
	 Mat grad_y_norm(grad_x.size[0], grad_x.size[1],CV_32F,0.0);
	 norm_gradient(grad_x,grad_y,grad_x_norm,grad_y_norm);

/*  find maximum
 	 
	double minVal; 
	double maxVal; 
	Point minLoc; 
	Point maxLoc;
	minMaxLoc( grad_x, &minVal, &maxVal, &minLoc, &maxLoc );
	cout<<"Min value : "<<minVal<<endl;
	cout<<"Max value : "<<maxVal<<endl;
*/	
	 /*unsigned int coor[2]={0,0};
	 coor[0] = 5;
	 coor[1] = 10;
	*/
	 //cout<<"stock values "<<stock_r[1][1];
	// stock_r = archieve_edge(detected_edges, grad_x_norm, grad_y_norm, a);
	/*
	  vector< vector<unsigned int> > v;
	  unsigned int n(0);
	  // Test archievement of one ray
	  for (unsigned int i=0;i<detected_edges.size[0];i++){
		for (unsigned int j=0;j<detected_edges.size[1];j++){
			if (detected_edges.at<uchar>(i,j)==255){
				vector<unsigned int> ij(2);
				ij[0]=i;ij[1]=j;
				v.push_back(ij);
			 }
		 }
	 }
	*/ 
	 

	
	
	//cout<<"n = "<<n<<endl;
	 //cout<<"Test vector "<<stock_r[1][0]<<endl;
	 //cout<<"Test vector "<<stock_r.back()[1]<<endl;
	// cout<<"Test vector last "<<stock_r.back().back()<<endl;
	// cout<<"Test vector first "<<stock_r.back().front()<<endl;
	// cout<<"Test vector first "<<stock_r.back().begin()<<endl;
	 
	 //eq = std::equal (test.begin(), test.end(), ra);
	 //cout<<eq<<endl;
	 
	  /// Directional gradient
	  //Mat atan_y_x;
	  //atan_y_x=directionalgradient(src_gray,grad_x, grad_y, 360);
	 /// Visualize directional gradient
	 // resizeshow(atan_y_x,"Directional gradient");

	
	  //imwrite( "Directional_gradient_d.jpg", atan_y_x );
	    Mat stock_vis;
	    stock_vis = Ray_images(detected_edges, src, grad_x_norm, grad_y_norm);
	    Mat swt_image;
	    swt_image = SWT(detected_edges, src, grad_x_norm, grad_y_norm);
	    /*
	    ty =  type2str( swt_image.type() );
		printf("Matrix: %s %dx%d \n", ty.c_str(), swt_image.cols, swt_image.rows );
		
			double minVal; 
			double maxVal; 
			Point minLoc; 
			Point maxLoc;
			minMaxLoc( swt_image, &minVal, &maxVal, &minLoc, &maxLoc );
			cout<<"Min value swt image: "<<minVal<<endl;
			cout<<"Max value swt image: "<<maxVal<<endl;
			for (unsigned int i=0;i<swt_image.size[0];i++){
				for (unsigned int j=0;j<swt_image.size[1];j++){
					if (swt_image.at<uchar>(i,j)!=0 && swt_image.at<uchar>(i,j)!=255)
					cout<<"values : "<<(int)swt_image.at<uchar>(i,j)<<endl;
				}
			}
		*/	
	  imwrite("/home/vika/projects/mycode/Projet/SWT_test.jpg", swt_image );	
	  resizeshow(swt_image ,"SWT");
	  /// Wait until user exit program by pressing a key
	  waitKey(0);
	  return 0;
  
 }
