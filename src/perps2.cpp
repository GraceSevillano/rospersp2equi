#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <per/prPerspective.h>
#include <per/prEquirectangular.h>
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpHomogeneousMatrix.h>
#include <Eigen/Dense>
#include <omp.h>
#include <queue>
#include <thread>
#include <mutex>
#include <opencv2/opencv.hpp>  
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>
#include <iomanip>
#include <opencv2/calib3d.hpp>

image_transport::Publisher pub_equirectangular_color_image;
image_transport::Publisher pub_equirectangular_depth_image;
vpImage<vpRGBa> EquiColorImage;
vpImage<float> EquiDepthImage;
int persWidth, persHeight;
int equiWidth, equiHeight;
unsigned int nbPixels;
Eigen::Matrix3d rotationMatrix;
Eigen::Vector3d translationVector;
prPerspective* perspectiveCamera;
prEquirectangular* equirectangularCamera;
std::queue<std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr>> frameQueue;
std::mutex queueMutex;
bool processing = true;
//bool generateSinglePtsFile = true;

//perspective camera values
double fx = 629.70495;
double fy = 628.84787;
double cx = 630.90746;
double cy = 366.6015;

double k1 = 0.088275;
double k2 = -0.035798;
double p1 = -0.001272;
double p2 = -0.002114;
double k3 = 0.00000;

// cámara wide-angle depth values
//double fx = 257.358; 
//double fy = 257.026; 
//double cx = 257.527; 
//double cy = 262.628; 

//double k1 = -0.351437; 
//double k2 = 0.183288; 
//double p1 = -0.00229322; 
//double p2 = 0.00252389; 
//double k3 = -0.0666614;


cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);
cv::Mat distCoeffs = (cv::Mat_<double>(5, 1) << k1, k2, p1, p2, k3);

std::string basePath;

void init()
{
    ROS_INFO("Initialization started");

    EquiColorImage.resize(equiHeight, equiWidth);
    EquiDepthImage.resize(equiHeight, equiWidth);
    perspectiveCamera = new prPerspective(fx, fy, cx, cy, k1, k2, p1, p2, k3);
    equirectangularCamera = new prEquirectangular(equiWidth, equiHeight);

    nbPixels = equiWidth * equiHeight;

    mkdir((basePath).c_str(), 0777);
    mkdir((basePath + "rgb").c_str(), 0777);
    mkdir((basePath + "depth").c_str(), 0777);

    ROS_INFO("Initialization complete");
}

template <typename T>
void convertToEquirectangular(const vpImage<T>& inputColor, const vpImage<float>& inputDepth, vpImage<T>& outputColor, vpImage<float>& outputDepth, const prPerspective& perspectiveCamera, const prEquirectangular& equirectangularCamera, int frameCounter)
{
    //std::string ptsFilename = basePath + "point_cloud_fra" + std::to_string(frameCounter) + ".txt";
    //std::ofstream ptsFile(ptsFilename, std::ios_base::out);

    //if (!ptsFile.is_open()) {
    //    ROS_ERROR("Failed to open file: %s", ptsFilename.c_str());
    //    return;
    //}

    //ptsFile << std::fixed << std::setprecision(8);
    std::vector<std::string> points;  //temporal  evitar confusion con el anterior

    cv::Mat map1, map2;
    cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(persWidth, persHeight), CV_32FC1, map1, map2);

    #pragma omp parallel for
    for (int y = 0; y < outputColor.getHeight(); y++) {
        for (int x = 0; x < outputColor.getWidth(); x++) {
            if constexpr (std::is_same_v<T, vpRGBa>) {
                outputColor[y][x].R = 0;
                outputColor[y][x].G = 0;
                outputColor[y][x].B = 0;
                outputColor[y][x].A = 0;
            } else {
                outputColor[y][x] = 0;
            }
            outputDepth[y][x] = 0.0f;
        }
    }

    #pragma omp parallel for
    for (int y = 0; y < persHeight; y++) {
        for (int x = 0; x < persWidth; x++) {

            float correctedX = map1.at<float>(y, x);
            float correctedY = map2.at<float>(y, x);

            unsigned int correctedX_int = std::round(correctedX);
            unsigned int correctedY_int = std::round(correctedY);

            if (correctedX_int >= 0 && correctedX_int < persWidth && correctedY_int >= 0 && correctedY_int < persHeight) {
                            
                double z = inputDepth[correctedY_int][correctedX_int];

            //if (z <= 0) continue;

                Eigen::Vector3d point;
                point << (x - cx) / fx * z, 
                        (y- cy) / fy * z, 
                        z;

                point = rotationMatrix * point + translationVector;

                double distance = point.norm();  // sqrt(X^2 + Y^2 + Z^2)

                //assert(distance > 0 && "Distance should always be positive"); //una verificadita 

                double theta = atan2(point[0], point[2]) ;
                double phi = asin(point[1] / point.norm());

                int u = static_cast<int>((theta + M_PI) / (2.0 * M_PI) * equiWidth);
                int v = static_cast<int>((phi + (M_PI / 2.0)) / M_PI * equiHeight);
                
                u = std::min(std::max(u, 0), equiWidth - 1);
                v = std::min(std::max(v, 0), equiHeight - 1);
            
                //if (u >= 0 && u < equiWidth && v >= 0 && v < equiHeight) {
                outputColor[v][u] = inputColor[correctedY_int][correctedX_int];
                outputDepth[v][u] = distance; //depth;

                  //  double max_distance = 2.88; 

                 //   int r = static_cast<int>(inputColor[y][x].R);
                 //   int g = static_cast<int>(inputColor[y][x].G);
                 //   int b = static_cast<int>(inputColor[y][x].B);

                 //   int intensity = static_cast<int>(std::min(255.0, (distance / max_distance) * 255.0));

                    //#pragma omp critical
                    //{
                    //    std::ostringstream ss;
                    //    ss << point[0] << " " << point[1] << " " << point[2] << " "
                    //    << intensity << " " << r << " " << g << " " << b << std::endl;
                    //    points.push_back(ss.str());
                    //}
                // }
            }
        }
    }

    //ptsFile << points.size() << std::endl;

    //for (const auto& point : points) {
    //    ptsFile << point;
    //}

    //ptsFile.close(); 
    
}

void processFrames()
{   
    //save the timestamps in the TUM format
    int frameCounter = 0;
    std::ofstream rgbFile(basePath + "rgb.txt");
    std::ofstream depthFile(basePath + "depth.txt");

    while (ros::ok() && processing)
    {   
        
        
        std::pair<sensor_msgs::ImageConstPtr, sensor_msgs::ImageConstPtr> frames;

        {
            std::unique_lock<std::mutex> lock(queueMutex);
            if (frameQueue.empty()) continue;

            frames = frameQueue.front();
            frameQueue.pop();
        }

        auto start_time = std::chrono::steady_clock::now();

        try
        {
            const sensor_msgs::ImageConstPtr& ColorImage_msg = frames.first;
            const sensor_msgs::ImageConstPtr& DepthImage_msg = frames.second;

            if (ColorImage_msg->data.empty() || DepthImage_msg->data.empty())
            {
                ROS_WARN("Empty image message");
                continue;
            }

            cv_bridge::CvImagePtr cvColorPtr = cv_bridge::toCvCopy(ColorImage_msg, sensor_msgs::image_encodings::BGR8);
            cv_bridge::CvImagePtr cvDepthPtr = cv_bridge::toCvCopy(DepthImage_msg, sensor_msgs::image_encodings::TYPE_32FC1);

            //cv::Mat undistortedColorImage, undistortedDepthImage;
            //cv::remap(cvColorPtr->image, undistortedColorImage, map1, map2, cv::INTER_LINEAR);
            //cv::remap(cvDepthPtr->image, undistortedDepthImage, map1, map2, cv::INTER_LINEAR);

            vpImage<vpRGBa> inputColorImage(persHeight, persWidth);
            vpImage<float> inputDepthImage(persHeight, persWidth);

            #pragma omp parallel for
            for (int i = 0; i < persHeight; i++) {
                for (int j = 0; j < persWidth; j++) {
                    inputColorImage[i][j].R = cvColorPtr->image.at<cv::Vec3b>(i, j)[2];
                    inputColorImage[i][j].G = cvColorPtr->image.at<cv::Vec3b>(i, j)[1];
                    inputColorImage[i][j].B = cvColorPtr->image.at<cv::Vec3b>(i, j)[0];
                    inputDepthImage[i][j] = cvDepthPtr->image.at<float>(i, j);
                }
            }

            EquiColorImage.resize(equiHeight, equiWidth);
            EquiDepthImage.resize(equiHeight, equiWidth);

            convertToEquirectangular(inputColorImage, inputDepthImage, EquiColorImage, EquiDepthImage, *perspectiveCamera, *equirectangularCamera, frameCounter);

            cv::Mat outColorImage(equiHeight, equiWidth, CV_8UC3);
            cv::Mat outDepthImage(equiHeight, equiWidth, CV_16UC1);

            #pragma omp parallel for
            for (int i = 0; i < equiHeight; i++) {
                for (int j = 0; j < equiWidth; j++) {
                    outColorImage.at<cv::Vec3b>(i, j)[0] = EquiColorImage[i][j].B;
                    outColorImage.at<cv::Vec3b>(i, j)[1] = EquiColorImage[i][j].G;
                    outColorImage.at<cv::Vec3b>(i, j)[2] = EquiColorImage[i][j].R;
                    outDepthImage.at<uint16_t>(i, j) = static_cast<uint16_t>(EquiDepthImage[i][j]*5000.0f);
                }
            }

            std::string colorFilename = basePath + "rgb/frame" + std::to_string(frameCounter) + ".png";
            std::string depthFilename = basePath + "depth/frame" + std::to_string(frameCounter) + ".png";
            cv::imwrite(colorFilename, outColorImage);
            cv::imwrite(depthFilename, outDepthImage);

            double colorTimestamp = ColorImage_msg->header.stamp.toSec();
            double depthTimestamp = DepthImage_msg->header.stamp.toSec();
            rgbFile << std::fixed << colorTimestamp << " rgb/frame" << frameCounter << ".png" << std::endl;
            depthFile << std::fixed << depthTimestamp << " depth/frame" << frameCounter << ".png" << std::endl;

            frameCounter++;

            cvColorPtr.reset();
            cvDepthPtr.reset();

            /*
            sensor_msgs::ImagePtr EquiColorImage_msg = cv_bridge::CvImage(ColorImage_msg->header, "bgr8", outColorImage).toImageMsg();
            pub_equirectangular_color_image.publish(EquiColorImage_msg);

            sensor_msgs::ImagePtr EquiDepthImage_msg = cv_bridge::CvImage(DepthImage_msg->header, "mono16", outDepthImage).toImageMsg();
            pub_equirectangular_depth_image.publish(EquiDepthImage_msg);
            */

        }
        catch (const cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
        }

        auto end_time = std::chrono::steady_clock::now();
        auto processing_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        ROS_INFO("Processing time for frame %d: %ld ms", frameCounter, processing_time);
    }

    rgbFile.close();
    depthFile.close();
    
}

void callback(const sensor_msgs::ImageConstPtr& ColorImage_msg, const sensor_msgs::ImageConstPtr& DepthImage_msg)
{
    std::unique_lock<std::mutex> lock(queueMutex);
    frameQueue.emplace(ColorImage_msg, DepthImage_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "perps2");
    ros::NodeHandle nH;

    ros::NodeHandle nHp("~");
    std::string inputColorImagesTopic, inputDepthImagesTopic, outputColorImagesTopic, outputDepthImagesTopic;
    nHp.param("inputColorImagesTopic", inputColorImagesTopic, std::string(""));
    nHp.param("outputColorImagesTopic", outputColorImagesTopic, std::string(""));
    nHp.param("inputDepthImagesTopic", inputDepthImagesTopic, std::string(""));
    nHp.param("outputDepthImagesTopic", outputDepthImagesTopic, std::string(""));
    nHp.param("savePath", basePath, std::string("/root/Desktop/Kevin_Azure_outputNEW/Data/Map/Dense/Dataset_equi/"));

    nHp.param("persWidth", persWidth, 1280);
    nHp.param("persHeight", persHeight, 720);
    nHp.param("equiWidth", equiWidth, 1280); 
    nHp.param("equiHeight", equiHeight, 720); 
    //nHp.param("generateSinglePtsFile", generateSinglePtsFile, false);

    nbPixels = equiWidth * equiHeight;

    // Set the rotation matrix and the translation vector

    // narrow camera kinect
    rotationMatrix << 0.01348508, -0.99950615, -0.02838054,  
                      0.99985291,  0.01377857, -0.01017193,  
                      0.01055796, -0.02823921,  0.99954551; 
        
    translationVector << 0.08626425, 0.14222383,0.02128052;
    
    //wide camera kinect
    //rotationMatrix << 0.01317515, -0.99063778, -0.13588022, 
    //                 0.99990909,  0.01344365, -0.00105867,  
    //                 0.0028755,  -0.13585375,  0.99072529;  
    //translationVector <<0.07562525, 0.1101918,0.04680101;

    init();

    std::thread processingThread(processFrames);

    image_transport::ImageTransport it(nH);
    message_filters::Subscriber<sensor_msgs::Image> sub_color_image(nH, inputColorImagesTopic, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth_image(nH, inputDepthImagesTopic, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10000), sub_color_image, sub_depth_image); 
    sync.registerCallback(boost::bind(&callback, _1, _2));

    pub_equirectangular_color_image = it.advertise(outputColorImagesTopic, 1);
    pub_equirectangular_depth_image = it.advertise(outputDepthImagesTopic, 1);

    ros::spin();

    processing = false;
    processingThread.join();

    delete perspectiveCamera;
    delete equirectangularCamera;

    return 0;
}
