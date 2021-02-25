#pragma once

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <opencv4/opencv2/xfeatures2d.hpp>

// #include <pcl/common/common_headers.h>
// #include <pcl/io/pcd_io.h>
// #include <pcl/point_types.h>
// #include <pcl/point_cloud.h>
// #include <pcl/visualization/pcl_visualizer.h>
// #include <pcl/visualization/cloud_viewer.h>


#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

#include <ros/ros.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;



#define CIRCLE  0   // Match follows the order of the images 1->2, 2->3 ... n->1
#define ALL     1   // All matches all the images eachother



// Box to store all image informations to reconstruct 3D
struct ImgBox
{
    Mat img;
    string imgName;
    Mat descriptor;
    vector<KeyPoint> keypoints;
    vector<Point2f> points2f;
};


struct MatcherBox
{
    Mat fstImage;
    Mat scdImage;
    Mat matchFsImg;  // First to Second
    Mat matchSfImg;  // Second to First
    vector<DMatch> dmatchFs;
    vector<DMatch> dmatchSf;   
};


// Abstraction to store all camera informations
struct Camera
{
    Mat img;
    Mat translation;
    Mat rotation;
    Mat essential;
    Mat fundamental;
    Mat projection = Mat(3, 4, CV_64F);
    Mat points3D;
};



class Matcher
{

public:

    Matcher(int amountFeatures, int matchType);

    void loadImages(vector<String>& imgs_path);
    void resizeImages(float ratio);
    void enhanceImage();
    void setCameraMatrix(Mat& K);
    void convertKpTo2f();
    void detectFeatures();
    void selectFeatures();
    void match();
    void selectGoodMatches();
    void showMatches();
    void triangulate();
    void reconstruct();
    void printImages();
    void showPCL();

    // Match Implementations
    void iMatchCircle();
    void iMatchAll();

    String& getCurrentInfoMessage();

    Matcher& setBFMatcher();    // default
    Matcher& setDescriptorMatcher();


    vector<ImgBox>& getImageBoxes();

private:
    Ptr<SIFT> detector;
    Ptr<DescriptorMatcher> matcher;
    BFMatcher* bfMatcher; 
    vector<ImgBox> imgBoxes;
    vector<Camera> cameras;
    vector <vector< vector<DMatch> > > matrix_knn_matches;
    vector< vector<DMatch> > matrix_good_matches;
    vector<Mat> matrix_img_matches;
    Mat K;

    String current_message = ""; 

    int amountFeatures;
    int matcherType;
};