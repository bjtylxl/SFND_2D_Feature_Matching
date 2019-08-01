# SFND 2D Feature Tracking

<img src="images/keypoints.png" width="820" height="248" />

The idea of the camera course is to build a collision detection system - that's the overall goal for the Final Project. As a preparation for this, you will now build the feature tracking part and test various detector / descriptor combinations to see which ones perform best. This mid-term project consists of four parts:

* First, you will focus on loading images, setting up data structures and putting everything into a ring buffer to optimize memory load. 
* Then, you will integrate several keypoint detectors such as HARRIS, FAST, BRISK and SIFT and compare them with regard to number of keypoints and speed. 
* In the next part, you will then focus on descriptor extraction and matching using brute force and also the FLANN approach we discussed in the previous lesson. 
* In the last part, once the code framework is complete, you will test the various algorithms in different combinations and compare them with regard to some performance measures. 

See the classroom instruction and code comments for more details on each of these parts. Once you are finished with this project, the keypoint matching part will be set up and you can proceed to the next lesson, where the focus is on integrating Lidar points and on object detection using deep-learning. 

## Dependencies for Running Locally
* cmake >= 2.8
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1 (Linux, Mac), 3.81 (Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* OpenCV >= 4.1
  * This must be compiled from source using the `-D OPENCV_ENABLE_NONFREE=ON` cmake flag for testing the SIFT and SURF detectors.
  * The OpenCV 4.1.0 source code can be found [here](https://github.com/opencv/opencv/tree/4.1.0)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools](https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory in the top level directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./2D_feature_tracking`.


Rubic Points:
MP.1 Data Buffer Optimization
Implement a vector for dataBuffer objects whose size does not exceed a limit (e.g. 2 elements). This can be achieved by pushing in new elements on one end and removing elements on the other end.

    int dataBufferSize = 5;       // number of images which are held in memory (ring buffer) at the same time
    vector<DataFrame> dataBuffer; 
	DataFrame frame;
    frame.cameraImg = imgGray;
    
    if (dataBuffer.size() < dataBufferSize)
		{
			dataBuffer.push_back(frame);
			cout << "#1 : LOAD IMAGE INTO BUFFER done" << endl;
		}
		else
		{
			dataBuffer.erase(dataBuffer.begin());
			dataBuffer.push_back(frame);
			cout << "Replace image in buffer !" << endl;
		}


MP.2 Keypoint Detection

Implement detectors HARRIS, FAST, BRISK, ORB, AKAZE, and SIFT and make them selectable by setting a string accordingly.

	//// TASK MP.2 -> add the following keypoint detectors in file matching2D.cpp and enable string-based selection based on detectorType
        //// -> HARRIS, FAST, BRISK, ORB, AKAZE, SIFT

        if (detectorType.compare("SHITOMASI") == 0)
        {
            detKeypointsShiTomasi(keypoints, imgGray, false);
        }
        else if (detectorType.compare("HARRIS") == 0)
		{
			detKeypointsHarris(keypoints, imgGray, false);
		}
		else
		{
			detKeypointsModern(keypoints, imgGray, detectorType, false);
		}



	//FAST, BRISK, ORB, AKAZE, SIFT
	void detKeypointsModern(std::vector<cv::KeyPoint> &keypoints, cv::Mat &img, std::string detectorType, bool bVis)
	{
	string windowName;
	if (detectorType.compare("FAST") == 0)
	{
		int threshold = 30;
		bool bNMS = true;
		cv::FastFeatureDetector::DetectorType type = cv::FastFeatureDetector::TYPE_9_16; // TYPE_9_16, TYPE_7_12, TYPE_5_8
		cv::Ptr<cv::FeatureDetector> detector = cv::FastFeatureDetector::create(threshold, bNMS, type);
		double t = (double)cv::getTickCount();
		detector->detect(img, keypoints);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "FAST detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		windowName = "FAST  Detector Results";
	}
	else if (detectorType.compare("BRISK") == 0)
	{
		cv::Ptr<cv::FeatureDetector> detector = cv::BRISK::create();
		double t = (double)cv::getTickCount();
		detector->detect(img, keypoints);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "BRISK detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		windowName = "BRISK  Detector Results";
	}
	else if (detectorType.compare("ORB") == 0)
	{
		cv::Ptr<cv::FeatureDetector> detector = cv::ORB::create();
		double t = (double)cv::getTickCount();
		detector->detect(img, keypoints);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "ORB detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		windowName = "ORB  Detector Results";
	}
	else if (detectorType.compare("AKAZE") == 0)
	{
		cv::Ptr<cv::FeatureDetector> detector = cv::AKAZE::create();
		double t = (double)cv::getTickCount();
		detector->detect(img, keypoints);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "AKAZE detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		windowName = "AKAZE  Detector Results";
	}
	else if (detectorType.compare("SIFT") == 0)
	{
		cv::Ptr<cv::FeatureDetector> detector = cv::xfeatures2d::SIFT::create();
		double t = (double)cv::getTickCount();
		detector->detect(img, keypoints);
		t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
		cout << "SIFT detection with n= " << keypoints.size() << " keypoints in " << 1000 * t / 1.0 << " ms" << endl;
		windowName = "SIFT  Detector Results";
	}
	// visualize results
	if (bVis)
	{
		cv::Mat visImage = img.clone();
		cv::drawKeypoints(img, keypoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
		cv::namedWindow(windowName, 6);
		imshow(windowName, visImage);
		cv::waitKey(0);
	}
}


	


MP.3 Keypoint Removal
Remove all keypoints outside of a pre-defined rectangle and only use the keypoints within the rectangle for further processing.

	//// TASK MP.3 -> only keep keypoints on the preceding vehicle

        // only keep keypoints on the preceding vehicle
        bool bFocusOnVehicle = true;
        cv::Rect vehicleRect(535, 180, 180, 150);
		cv::vector<cv::KeyPoint>::iterator it;
		if (bFocusOnVehicle)
		{
			for (it = keypoints.end(); it != keypoint.begin(); --it)
			{
				if (!vehicleRect.contains(it->pt))
				{ // keypoint not on preceding vehicle

					keypoints.erase(it);
					// We need to come back size we are erasing points from the vector.
					--it;
				}

			}


		}

MP.4 Keypoint Descriptors
Implement descriptors BRIEF, ORB, FREAK, AKAZE and SIFT and make them selectable by setting a string accordingly.

	 //// TASK MP.4 -> add the following descriptors in file matching2D.cpp and enable string-based selection based on descriptorType
        //// -> BRIEF, ORB, FREAK, AKAZE, SIFT

        cv::Mat descriptors;
        string descriptorType = "BRISK"; // BRISK, ORB, FREAK, AKAZE, SIFT
        descKeypoints((dataBuffer.end() - 1)->keypoints, (dataBuffer.end() - 1)->cameraImg, descriptors, descriptorType);
        //// EOF STUDENT ASSIGNMENT





	void descKeypoints(vector<cv::KeyPoint> &keypoints, cv::Mat &img, cv::Mat &descriptors, string descriptorType)
	{
    // select appropriate descriptor
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descriptorType.compare("BRISK") == 0)
    {

        int threshold = 30;        // FAST/AGAST detection threshold score.
        int octaves = 3;           // detection octaves (use 0 to do single scale)
        float patternScale = 1.0f; // apply this scale to the pattern used for sampling the neighbourhood of a keypoint.

        extractor = cv::BRISK::create(threshold, octaves, patternScale);
    }
	else if (descriptorType.compare("BRIEF") == 0)
	{

		extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
	}
    else if (descriptorType.compare("ORB") == 0)
    {

		extractor = cv::ORB::create();
    }
	else if (descriptorType.compare("FREAK") == 0)
	{
		extractor = cv::xfeatures2d::FREAK::create();
	}
	else if (descriptorType.compare("AKAZE") == 0)
	{
		extractor = cv::AKAZE::create();
	}
	else if (descriptorType.compare("SIFT") == 0)
	{
		extractor = cv::xfeatures2d::SIFT::create();
	}

    // perform feature description
    double t = (double)cv::getTickCount();
    extractor->compute(img, keypoints, descriptors);
    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    cout << descriptorType << " descriptor extraction in " << 1000 * t / 1.0 << " ms" << endl;
}





	

MP.5 Descriptor Matching
Implement FLANN matching as well as k-nearest neighbor selection. Both methods must be selectable using the respective strings in the main function.

	if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
		if (descSource.type() != CV_32F || descRef.type() != CV_32F)
		{ // OpenCV bug workaround, float 32 type is required
			descSource.convertTo(descSource, CV_32F);
			descRef.convertTo(descRef, CV_32F);
		}
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

MP.6 Descriptor Distance Ratio
Use the K-Nearest-Neighbor matching to implement the descriptor distance ratio test, which looks at the ratio of best vs. second-best match to decide whether to keep an associated pair of keypoints.

	// perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descSource, descRef, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)

		vector<vector<cv::DMatch>> knn_matches;
		matcher->knnMatch(descSource, descRef, knn_matches, 2);
		double minDesDistRio = 0.8;
		for (auto it = knn_matches.begin(); it != knn_matches.end(); ++it)
		{
			if ((*it)[0].distance < minDesDistRio * (*it)[1].distance)
			{
				matches.push_back((*it)[0]);
			}

			cout << "keypoint removed: " << knn_matches.size() - matches.size() << endl;
		}
    }

MP.7 Performance Evaluation 1
Count the number of keypoints on the preceding vehicle for all 10 images and take note of the distribution of their neighborhood size. Do this for all the detectors you have implemented.

![在这里插入图片描述](https://img-blog.csdnimg.cn/20190731163643454.png)
 

MP.8 Performance Evaluation 2
Count the number of matched keypoints for all 10 images using all possible combinations of detectors and descriptors. In the matching step, the BF approach is used with the descriptor distance ratio set to 0.8.

![在这里插入图片描述](https://img-blog.csdnimg.cn/2019073116372898.png)

MP.9 Performance Evaluation 3
Log the time it takes for keypoint detection and descriptor extraction. The results must be entered into a spreadsheet and based on this data, the TOP3 detector / descriptor combinations must be recommended as the best choice for our purpose of detecting keypoints on vehicles.


![在这里插入图片描述](https://img-blog.csdnimg.cn/20190731163808594.png)


![在这里插入图片描述](https://img-blog.csdnimg.cn/20190731163842800.png)


 the TOP3 detector / descriptor combinations of detecting keypoints on vehicles is "FAST+BRIEF", "FAST+ORB" and "SHITOMASI+ORB".