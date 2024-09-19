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
    //Dense

     //M_final=  M_Dense x pose_39_inv   #####PROBAR COMO 4NEW
    //rotationMatrix <<0.05945462, -0.99728885, -0.04337438,
    //                 0.99751625,  0.06099991, -0.03521861,
    //                 0.03776897, -0.04117278,  0.99843737;

    //translationVector << 0.01023823, 0.26494908, 0.00629911;                
    //M_final=  M_Dense x pose_39   #####PROBAR COMO 3NEW
    //rotationMatrix << -0.01048821, -0.99964555, -0.02444489, 
    //                   0.99956218, -0.00980464, -0.02791576 ,      
    //                  0.02766618, -0.02472692,  0.99931191 ;

    //translationVector << 0.02545723, 0.00538866 , -0.12964304;

    //equiMpc2 x M_Slow (1 scale): 21NEW   ############################   este tendre que comparar con elmejorcito como SLOW   
    //rotationMatrix << 0.00134933, -0.99951907, -0.03097845, 
    //0.99970228,  0.00210245, -0.02429167,  
    //                  0.02434514, -0.03093647,  0.9992249;  

    //translationVector <<  0.08366683, 0.02488427, 0.10770249;

    //equiMpc2 x M_Eight (1 scale): 17NEW    #################################  este tendre que comparar con elmejorcito como EIGHT
    //rotationMatrix << -0.01527555, -0.99973337, -0.01731153,  
    //                   0.99455784, -0.01340746, -0.10331559,  
    //                   0.103056,   -0.01879553,  0.99449803;   
                    
    //translationVector << 0.08293568, 0.06290013, 0.0768137;

    //equiMpc2 x M_dense (1 scale): 13NEW      ###############################              el mejorcito de DENSE 
    rotationMatrix << 0.01348508, -0.99950615, -0.02838054,  
                      0.99985291,  0.01377857, -0.01017193,  
                      0.01055796, -0.02823921,  0.99954551; 
        
    translationVector << 0.08626425, 0.14222383,0.02128052;

    //equiMpc_inv x M_dense (1 scale): 10NEW
    //rotationMatrix <<   0.02144861, -0.99737523, -0.06915998,  
    //                    0.99453831,  0.02835274, -0.10044621,  
    //                    0.10214336, -0.06662781,  0.99253547; 
        
    //translationVector <<0.01687081,0.27760019,-0.14151878;

    //M_dense (1 scale)x equiMpc: 11NEW
    //rotationMatrix <<  0.02761024, -0.99443511, -0.10166974,
    //                   0.99961466,  0.02718427,  0.00557318,
    //                  -0.00277837, -0.1017844,   0.994803;
        
    //translationVector << 0.14444763, 0.13283358, 0.04129122;

    //M_dense (1 scale) x equiMpc_inv: 12NEW
    //rotationMatrix << 0.02384529, -0.99913024,  0.0342044,  
    //                  0.99739967,  0.02144844, -0.06880648,  
    //                  0.06801304,  0.03575609,  0.99704309; 
        
    //translationVector << -0.11948847, 0.14068991,-0.15608379;



    //pose_59_inv x M_dense (1 scale): 6NEW
    //rotationMatrix << 0.05950845, -0.99747862, -0.03866737, 
    //                  0.9979548,   0.06035446, -0.02108997, 
    //                  0.0233705,  -0.03733322,  0.99902898;  
 
    //translationVector <<  0.15254342,  0.14063389, 0.00110662;


    //M_final=  pose_39_inv x M_Dense   #####PROBAR COMO 2NEW
    //rotationMatrix << 0.05950845, -0.99747862, -0.03866737 , 
    //                  0.9979548,   0.06035446, -0.02108997,      
    //                  0.0233705,  -0.03733322,  0.99902898 ;

    //translationVector <<  0.070021,0.13645837 , -0.03914529;

    //M_final=  pose_39 x M_Dense  *(1 scale) #####PROBAR COMO 5NEW

    //rotationMatrix <<  -0.01044714, -0.99953472, -0.02865633,
    //                    0.99906578, -0.00923238, -0.042203,   
    //                    0.04191884, -0.02907049,  0.99869858;

    //translationVector <<-0.12083889,0.12555045,-0.12455485;
    //M_final=  pose_39 x M_Dense   #####PROBAR COMO 1NEW
    //rotationMatrix << -0.01044714, -0.99953472, -0.02865633 , 
    //                  0.99906578,  -0.00923238, -0.042203,      
    //                 0.04191884,  -0.02907049,  0.99869858 ;

    //translationVector <<  -0.03835126,0.13224934 , -0.08457336;


    //M_final= M_Dense × (pose_0_inv x pose_39)   6 
    //rotationMatrix << 0.08497511 , -0.99395069 , -0.06957526 , 
    //                  0.9950998  ,  0.08820167 , -0.04469207 ,  
    //                  0.05055843 , -0.06543656 ,  0.99657504 ;

    //translationVector <<  -0.17154682,0.17719159 , -0.18694856;

    //M_final = (pose_39 × M_Dense) × pose_0: 5 TAMPOCO NO ROTUNDO
    // rotationMatrix <<  0.99875124, 0.04993453, -0.00155034 ,
    //                    -0.00581678, 0.08540838, -0.99632877 ,
    //                    -0.04961879, 0.99509421, 0.0855922 ;
    
    //translationVector << -0.11109138, -0.20357774,0.03743816;


    //M_final = (pose_39_inv × M_Dense) × pose_0  4 definitivamente NOOOO ROTUNDO:
    //rotationMatrix <<  0.99444552, 0.0474447, 0.0939499,
    //                   0.0943809, -0.00692217, -0.99551237,
    //                   -0.04658147, 0.99884994, -0.01136158;

    //translationVector << -0.06061674 , -0.21323866, 0.05426068;


    //M_Dense × (pose_39 × pose_0_inv) 3:
    //rotationMatrix << 0.06038693, -0.99369799, -0.09443082,
    //                 0.99797785, 0.06198433, -0.01407307,
    //                  0.01983764, -0.09339002, 0.99543222;

    //translationVector <<0.20403821, 0.00946779, -0.0262464;  
    
    
    //(pose_0×pose_39_inv)×M_Dense    2:
    //rotationMatrix << -0.01136158, -0.99884994, -0.04658147,
    //                 0.99551237, -0.00692217, -0.0943809,
    //                  0.0939499, -0.0474447, 0.99444552;

    //translationVector <<0.05426068, 0.21323866, -0.06061674;  

    //M_final=(pose_0_inv × pose_39)×M_Dense
    //rotationMatrix << 0.0855922, -0.99509421, -0.04961879,
    //                 0.99632877, 0.08540838, 0.00581678,
    //                  -0.00155034, -0.04993453, 0.99875124;

    //translationVector <<0.03743816, 0.20357774, -0.11109138;
 //dense nuevo2 generando nuevamente thetas
    //rotationMatrix << 0.02240503249178913, -0.9992064562075822, -0.03293132684238116,
    //                  0.9992014233367026, 0.023470647862718328, -0.03233650313838989,
    //                  0.033083758865769516, -0.032180522673268715, 0.9989343694915819;

    //translationVector <<  0.018516914914333656, 0.12329024026719461, -0.05535687049279284;

    //DENSE NUEVO
    //rotationMatrix << 0.021884959269957968, -0.9992082781098939, -0.03322447338658289,
    //                  0.999171848394038, 0.023000225207552055, -0.03356506544848443,
    //                  0.034302658081533675, -0.032462382389235725, 0.9988841356879927;

    //translationVector << 0.01528973245948181, 0.12654936298383707, -0.05767239951346522;

    //DENSE
    //rotationMatrix << 0.02452235260785531 , -0.999126137733474    , -0.03384703668859073,
    //                  0.9991684496848064  ,  0.025598472201519583 , -0.031735222505802504,
    //                  0.03257392606134954 , -0.033040667914919206 , 0.9989230474671853;

    //translationVector << 0.015842130898891596, 0.13519952218224063, -0.061955005323263534;

    
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
