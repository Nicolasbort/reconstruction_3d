#include "matcher.hpp"



Matcher::Matcher(int amountFeatures, int matchType)
{
    printf("Matcher initialized\n");
    this->amountFeatures = amountFeatures;
    this->detector = SIFT::create(amountFeatures);

    this->matcher = DescriptorMatcher::create(DescriptorMatcher::FLANNBASED);   // default

    this->matcherType = matchType;
}


void Matcher::setCameraMatrix(Mat& K)
{
    printf("Setting cam param\n");
    this->K = K;
}


void Matcher::loadImages(vector<String>& imgs_path)
{
    for (int i=0; i<imgs_path.size(); i++)
    {
        this->current_message = "Loading " + imgs_path[i];
        ROS_WARN(this->current_message.c_str());

        ImgBox tempImgBox;

        tempImgBox.img = imread(imgs_path[i]);
        tempImgBox.imgName = imgs_path[i];

        imgBoxes.push_back(tempImgBox);
    }

    matrix_knn_matches.resize(imgBoxes.size());
    matrix_good_matches.resize(imgBoxes.size());
    matrix_img_matches.resize(imgBoxes.size());
    cameras.resize(imgBoxes.size());
}


void Matcher::enhanceImage()
{
    for (auto& imgBox : imgBoxes)
        detailEnhance(imgBox.img, imgBox.img, 15.0f, 0.2f);
}


void Matcher::resizeImages(float ratio)
{
    for (auto& imgBox : imgBoxes)
        resize(imgBox.img, imgBox.img, Size(), ratio, ratio);
}


void Matcher::printImages()
{
    if (imgBoxes.empty())
        printf("Images empty\n");
    else
        for (auto& imgBox : imgBoxes)
            printf("%s\n", imgBox.imgName.c_str());
}


void Matcher::detectFeatures()
{
    printf("Detecting keypoints and descriptors\n");

    // Computes the keypointss and descriptors
    for (auto& imgBox : imgBoxes){
        printf("Current %s\n", imgBox.imgName.c_str());
        this->detector->detectAndCompute( imgBox.img, noArray(), imgBox.keypoints, imgBox.descriptor );
    }
}


void Matcher::match()
{
    switch (matcherType)
    {
        case CIRCLE:
            this->iMatchCircle();
            break;

        case ALL:
            this->iMatchAll();
            break;

        default:
            printf("Match type error\n");
            break;
    }

    // printf("Starting matches\n");

    // // Matches the image with the next in the vector
    // for (int i=0; i<imgBoxes.size(); i++)
    // {
    //     printf("Matching image[%d] and image[%d]\n", i, i+1);
    //     if (i+1 == imgBoxes.size())
    //         // this->bfMatcher->knnMatch(imgBoxes[i].descriptor, imgBoxes[0].descriptor, matrix_knn_matches[i], 3);
    //         this->matcher->knnMatch( imgBoxes[i].descriptor, imgBoxes[0].descriptor, matrix_knn_matches[i], 3 );
    //     else
    //         // this->bfMatcher->knnMatch(imgBoxes[i].descriptor, imgBoxes[i+1].descriptor, matrix_knn_matches[i], 3);
    //         this->matcher->knnMatch( imgBoxes[i].descriptor, imgBoxes[i+1].descriptor, matrix_knn_matches[i], 3 );
    // }

    // // Filter matches using the Lowe's ratio test
    // const float ratio_thresh = 0.67f;




    // printf("Getting the best matches\n");
    // // Go through all the matches and stores the best matches **MATCHES ARE THE SAME SIZE AS imgBoxes**
    // for (int i=0; i<imgBoxes.size(); i++)
    // {
    //     // Get the good matches
    //     for (size_t j = 0; j < matrix_knn_matches[i].size(); j++)
    //     {
    //         if (matrix_knn_matches[i][j][0].distance < ratio_thresh * matrix_knn_matches[i][j][1].distance)
    //             matrix_good_matches[i].push_back(matrix_knn_matches[i][j][0]);
    //     }

    // }

}


void Matcher::iMatchCircle()
{
    printf("Match circle selected\n");

    this->matrix_knn_matches.resize(imgBoxes.size());

    // Matches the image with the next in the vector
    for (int i=0; i<imgBoxes.size(); i++)
    {
        if (i+1 == imgBoxes.size())
            this->matcher->knnMatch( imgBoxes[i].descriptor, imgBoxes[0].descriptor, matrix_knn_matches[i], 5 );
        else
            this->matcher->knnMatch( imgBoxes[i].descriptor, imgBoxes[i+1].descriptor, matrix_knn_matches[i], 5 );
    }
}


void Matcher::iMatchAll()
{
    printf("Match all selected\n");

    this->matrix_knn_matches.resize(imgBoxes.size() * imgBoxes.size() -1 );
    int k=0;

    // Matches the image with all the others
    for (int i=0; i<imgBoxes.size(); i++)
    {
        for (int j=0; j<imgBoxes.size(); j++)
        {
            if (i != j){
                this->matcher->knnMatch( imgBoxes[i].descriptor, imgBoxes[j].descriptor, matrix_knn_matches[k], 5 );
                k++;
            }
        }
    }

    printf("matrix knn size: %lu\n", matrix_knn_matches.size());
}


void Matcher::selectGoodMatches()
{
    printf("Getting the best matches\n");

    const float ratio_thresh = 0.7f;

    // Go through all the matches and stores the best matches **MATCHES ARE THE SAME SIZE AS imgBoxes**
    for (int i=0; i<imgBoxes.size(); i++)
    {
        // Get the good matches
        for (size_t j = 0; j < matrix_knn_matches[i].size(); j++)
        {
            if (matrix_knn_matches[i][j][0].distance < ratio_thresh * matrix_knn_matches[i][j][1].distance)
                matrix_good_matches[i].push_back(matrix_knn_matches[i][j][0]);
        }

    } 
}


void Matcher::showMatches()
{
    printf("Drawing the best matches\n");

    switch (matcherType)
    {
        case CIRCLE:
            for (int i=0; i<imgBoxes.size(); i++)
            {
                if (i+1 == imgBoxes.size())
                    drawMatches( imgBoxes[i].img, imgBoxes[i].keypoints, imgBoxes[0].img, imgBoxes[0].keypoints, matrix_good_matches[i], matrix_img_matches[i], Scalar::all(-1),
                        Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
                else
                    drawMatches( imgBoxes[i].img, imgBoxes[i].keypoints, imgBoxes[i+1].img, imgBoxes[i+1].keypoints, matrix_good_matches[i], matrix_img_matches[i], Scalar::all(-1),
                        Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                imshow("MATCH_" + to_string(i), matrix_img_matches[i]);
            }
            break;

        case ALL:
            // Draw the matches and imshow
            for (int i=0; i<imgBoxes.size(); i++)
            {
                for (int j=0; i<imgBoxes.size(); j++)
                {
                    if (i == j)
                        continue;
                    // TERMINAR
                    drawMatches( imgBoxes[i].img, imgBoxes[i].keypoints, imgBoxes[j].img, imgBoxes[j].keypoints, matrix_good_matches[i], matrix_img_matches[i], Scalar::all(-1),
                            Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

                    imshow("MATCH_" + to_string(i), matrix_img_matches[i]);
                    

                }
            }
            break;

        default:
            break;
    }


    waitKey(0);
}


void Matcher::triangulate()
{
    printf("Triangulating points\n");

    K.convertTo(K, CV_64FC1);

    Mat Rt_origin = Mat::eye(3, 4, CV_64FC1);


    // cout << "K:\n" << K * Rt_origin <<"\n";
    // cout << "Proj origin:\n" << Rt_origin <<"\n";
    // cout << "Proj cam:\n" << cameras[0].projection  << "\n";


    for (int i=0; i<cameras.size(); i++)
    {
        if (i+1 == cameras.size()){
            printf("Triangulating %s and %s\n", imgBoxes[i].imgName.c_str(), imgBoxes[0].imgName.c_str());
            triangulatePoints(K * Rt_origin, cameras[i].projection, imgBoxes[i].points2f, imgBoxes[0].points2f, cameras[i].points3D);
        }else{
            printf("Triangulating %s and %s\n", imgBoxes[i].imgName.c_str(), imgBoxes[i+1].imgName.c_str());
            triangulatePoints(K * Rt_origin, cameras[i].projection, imgBoxes[i].points2f, imgBoxes[i+1].points2f, cameras[i].points3D);
        }
    }
}


void Matcher::selectFeatures()
{
//     vector<Point2f> selected_points1, selected_points2;

//     for(int i = 0; i < matches_2nn_12.size(); i++)
//     { // i is queryIdx
//         if( matches_2nn_12[i][0].distance/matches_2nn_12[i][1].distance < ratio && matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].distance 
//             / matches_2nn_21[matches_2nn_12[i][0].trainIdx][1].distance < ratio ) 
//         {
//         if(matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].trainIdx == matches_2nn_12[i][0].queryIdx) 
//         {
//             selected_points1.push_back( kpts_vec1[matches_2nn_12[i][0].queryIdx].pt );
//             selected_points2.push_back( kpts_vec2[matches_2nn_21[matches_2nn_12[i][0].trainIdx][0].queryIdx].pt );
//         }
//         }
//     }
}


void Matcher::reconstruct()
{
    this->detectFeatures();

    this->match();
    this->selectGoodMatches();

    this->showMatches();

    this->convertKpTo2f();

    for (int i=0; i<imgBoxes.size(); i++)
    {
        if (i+1 != imgBoxes.size())
        {
            cameras[i].essential = findEssentialMat(imgBoxes[i].points2f, imgBoxes[i+1].points2f, K, RANSAC);
            recoverPose(cameras[i].essential, imgBoxes[i].points2f, imgBoxes[i+1].points2f, K, cameras[i].rotation, cameras[i].translation);
        }
        else
        {
            cameras[i].essential = findEssentialMat(imgBoxes[i].points2f, imgBoxes[0].points2f, K, RANSAC);
            recoverPose(cameras[i].essential, imgBoxes[i].points2f, imgBoxes[0].points2f, K, cameras[i].rotation, cameras[i].translation);
        }

        // Criando a matrix de projecao 3x4
        cameras[i].rotation.copyTo(cameras[i].projection(Rect(0, 0, 3, 3)));
        cameras[i].translation.copyTo(cameras[i].projection.colRange(3, 4)); 
    }
    this->current_message = "Recovering camera pose finished";
}


void Matcher::convertKpTo2f()
{
    for (auto& imgbox : imgBoxes)
    {
        imgbox.points2f.resize(imgbox.keypoints.size());

        for (int i=0; i<imgbox.keypoints.size(); i++)
        {
            float x = imgbox.keypoints[i].pt.x;
            float y = imgbox.keypoints[i].pt.y;

            imgbox.points2f[i] = Point2f(x, y);
        }

        imgbox.points2f.resize(amountFeatures);

        this->current_message = "KeyPoints converted. Kp size: " + to_string(imgbox.keypoints.size()) + ". Pt2f size: " + to_string(imgbox.points2f.size());
    }
    
}


void Matcher::showPCL()
{
    // prepare a viewer
	// pcl::visualization::PCLVisualizer viewer("Viewer");
    // viewer.setBackgroundColor (255, 255, 255);

    // // Gets the sum of all 3D points
    // int points3D_size = 0;
    // for (auto& camera : cameras)
    //     points3D_size += camera.points3D.cols;


    // create point cloud
	// pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	// cloud->points.resize (points3D_size);


    // int countPoints3D = 0;
    // for (int idx=0; idx<cameras.size(); idx++)
    // {
    //     for(int i = 0; i<cameras[idx].points3D.cols; i++) 
    //     {
    //         pcl::PointXYZRGB &point = cloud->points[countPoints3D];
    //         Mat p3d;
    //         Mat _p3h = cameras[idx].points3D.col(i);
    //         convertPointsFromHomogeneous(_p3h.t(), p3d);
    //         point.x = p3d.at<double>(0);
    //         point.y = p3d.at<double>(1);
    //         point.z = p3d.at<double>(2);
    //         point.r = 0;
    //         point.g = 0;
    //         point.b = 255;

    //         countPoints3D++;
    //     } 
    // }



    // viewer.addPointCloud(cloud, "Triangulated Point Cloud");
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
    //                                             3,
    //                                             "Triangulated Point Cloud");
    // viewer.addCoordinateSystem (1.0);
    // viewer.addCoordinateSystem(3.0);


    // // add the second camera pose 
    // Eigen::Matrix4f eig_mat;
    // Eigen::Affine3f cam_pose;


    // R.convertTo(R, CV_32F);
    // t.convertTo(t, CV_32F);

    // //this shows how a camera moves
    // Mat Rinv = R.t(); 
    // Mat T = -Rinv * t;

    // eig_mat(0,0) = Rinv.at<float>(0,0);eig_mat(0,1) = Rinv.at<float>(0,1);eig_mat(0,2) = Rinv.at<float>(0,2);
    // eig_mat(1,0) = Rinv.at<float>(1,0);eig_mat(1,1) = Rinv.at<float>(1,1);eig_mat(1,2) = Rinv.at<float>(1,2);
    // eig_mat(2,0) = Rinv.at<float>(2,0);eig_mat(2,1) = Rinv.at<float>(2,1);eig_mat(2,2) = Rinv.at<float>(2,2);
    // eig_mat(3,0) = 0.f; eig_mat(3,1) = 0.f; eig_mat(3,2) = 0.f;
    // eig_mat(0, 3) = T.at<float>(0);
    // eig_mat(1, 3) = T.at<float>(1);
    // eig_mat(2, 3) = T.at<float>(2);
    // eig_mat(3, 3) = 1.f;

    // cam_pose = eig_mat;

    // //cam_pose should be Affine3f, Affine3d cannot be used
    // viewer.addCoordinateSystem(1.0, cam_pose, "2nd cam");

    // viewer.initCameraParameters ();
    // while (!viewer.wasStopped ()) {
    //     viewer.spin();
    // }


    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;
    // //... populate cloud
    // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    // viewer.showCloud (cloud);
    // while (!viewer.wasStopped ())
    // {
    // }

}


String& Matcher::getCurrentInfoMessage(){
    return this->current_message;
}

vector<ImgBox>& Matcher::getImageBoxes()
{
    return this->imgBoxes;
}
