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
#include <opencv2/opencv.hpp>  //solo para guardar
#include <fstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <chrono>

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

//perspective camera values
double fx = 629.70495;
double fy = 628.84787;
double cx = 630.90746;
double cy = 366.6015;


std::string basePath;

void init()
{
    ROS_INFO("Initialization started");

    EquiColorImage.resize(equiHeight, equiWidth);
    EquiDepthImage.resize(equiHeight, equiWidth);
    perspectiveCamera = new prPerspective(fx, fy, cx, cy, 0.088275, -0.035798, -0.001272, -0.002114, 0.000000);
    equirectangularCamera = new prEquirectangular(equiWidth, equiHeight);

    nbPixels = equiWidth * equiHeight;

    mkdir((basePath).c_str(), 0777);
    mkdir((basePath + "rgb").c_str(), 0777);
    mkdir((basePath + "depth").c_str(), 0777);

    ROS_INFO("Initialization complete");
}

template <typename T>
void convertToEquirectangular(const vpImage<T>& inputColor, const vpImage<float>& inputDepth, vpImage<T>& outputColor, vpImage<float>& outputDepth, const prPerspective& perspectiveCamera, const prEquirectangular& equirectangularCamera)
{
    #pragma omp parallel for
    for (int y = 0; y < outputColor.getHeight(); ++y) {
        for (int x = 0; x < outputColor.getWidth(); ++x) {
            if constexpr (std::is_same_v<T, vpRGBa>) {
                outputColor[y][x].R = 0;
                outputColor[y][x].G = 0;
                outputColor[y][x].B = 0;
                outputColor[y][x].A = 0;
            } else {
                outputColor[y][x] = 0;
            }
            outputDepth[y][x] = 0;
        }
    }

    #pragma omp parallel for
    for (int y = 0; y < persHeight; ++y) {
        for (int x = 0; x < persWidth; ++x) {
            double z = inputDepth[y][x]; //cambie la definicion depth a z
            Eigen::Vector3d point;
            point << (x - cx) / fx * z, 
                     (y - cy) / fy * z, 
                     z;

            point = rotationMatrix * point + translationVector;
            double distance = point.norm();  // sqrt(X^2 + Y^2 + Z^2)

            assert(distance > 0 && "Distance should always be positive"); //una verificadita 


            double theta = atan2(point[2], point[0]) - (M_PI / 2.0);
            double phi = asin(point[1] / point.norm());

            int u = static_cast<int>((theta + M_PI) / (2.0 * M_PI) * equiWidth);
            int v = static_cast<int>((phi + (M_PI / 2.0)) / M_PI * equiHeight);
            
            u = equiWidth - u;

            if (u >= 0 && u < equiWidth && v >= 0 && v < equiHeight) {
                outputColor[v][u] = inputColor[y][x];
                outputDepth[v][u] = distance; //depth;
            }
        }
    }
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

            convertToEquirectangular(inputColorImage, inputDepthImage, EquiColorImage, EquiDepthImage, *perspectiveCamera, *equirectangularCamera);

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

    nbPixels = equiWidth * equiHeight;

    // Set the rotation matrix and the translation vector
    //Dense
    rotationMatrix << 0.02452235260785531 , -0.999126137733474    , -0.03384703668859073,
                      0.9991684496848064  ,  0.025598472201519583 , -0.031735222505802504,
                      0.03257392606134954 , -0.033040667914919206 , 0.9989230474671853;

    translationVector << 0.015842130898891596, 0.13519952218224063, -0.061955005323263534;

    //Eight
    //rotationMatrix << -0.004790475933558106 , -0.999705611061588    , -0.02378539116821474,
    //                   0.9921722258096854   , -0.0017835601374794282, -0.12486431763381703,
    //                   0.12478513735511554  , -0.02419736202440241  ,  0.9918886832710138 ;

    //translationVector << 0.011336040274572651 , 0.0547212385805503  , -0.008194343855935282;

    //SlowCircle
    //rotationMatrix << 0.01231197886305713, -0.9992541627671837, -0.03659971550196347,
    //                 0.9988520629647276, 0.01398508084898507, -0.04581454808090266,
    //                  0.046292235131550986, -0.03599363416198878, 0.9982792706025492;
                        
    //translationVector << 0.011479783301971105, 0.01603237083853206, 0.021855775198222304; 

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