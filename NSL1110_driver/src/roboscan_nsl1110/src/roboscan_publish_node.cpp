#include <cstdio>
#include "cartesian_transform.hpp"
#include "interface.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <cv_bridge/cv_bridge.h>
//#include <pcl/conversions.h>
//#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/srv/set_camera_info.hpp>
#include <cstdio>
#include <sys/stat.h>
#include <cstdlib>
#include <unistd.h>
#include "roboscan_publish_node.hpp"


using namespace nanosys;
using namespace std::chrono_literals;
using namespace cv;
using namespace std;

#define WIN_NAME "NSL-1110AA IMAGE"
#define IMAGE_DATA_SIZE		(320*240*4*2)
#define NUM_COLORS     		30000


enum
{
	NO_FILTER,
	TEMPORAL_FILTER = MASK_TEMPORAL_FILTER,
	AVERAGE_FILTER = MASK_AVERAGE_FILTER,
	MEDIAN_FILTER = MASK_MEDIAN_FILTER,
	EDGE_FILTER = MASK_EDGE_FILTER,
	AVERAGE_TEMPORAL_FILTER = (MASK_TEMPORAL_FILTER | MASK_AVERAGE_FILTER),
	MEDIAN_TEMPORAL_FILTER = (MASK_TEMPORAL_FILTER | MASK_MEDIAN_FILTER),
	TEMPORAL_EDGE_FILTER = (MASK_TEMPORAL_FILTER | MASK_EDGE_FILTER),
	MEDIAN_TEMPORAL_EDGE_FILTER = (MASK_MEDIAN_FILTER | MASK_TEMPORAL_FILTER | MASK_EDGE_FILTER),
	AVERAGE_TEMPORAL_EDGE_FILTER = (MASK_AVERAGE_FILTER | MASK_TEMPORAL_FILTER | MASK_EDGE_FILTER)
};

std::atomic<int> x_start = -1, y_start = -1;


static void callback_mouse_click(int event, int x, int y, int flags, void* user_data)
{
	std::ignore = flags;
	std::ignore = user_data;
	
	if (event == cv::EVENT_LBUTTONDOWN)
	{
		x_start = x;
		y_start = y;
	}
	else if (event == cv::EVENT_LBUTTONUP)
	{
	}
	else if (event == cv::EVENT_MOUSEMOVE)
	{
	}
}


roboscanPublisher::roboscanPublisher() : Node("roboscan_publish_node")
{ 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    imgDistancePub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDistance", qos_profile); 
    imgAmplPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanAmpl", qos_profile); 
    imgGrayPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanGray", qos_profile); 
    imgDCSPub = this->create_publisher<sensor_msgs::msg::Image>("roboscanDCS", qos_profile); 
    pointcloudPub = this->create_publisher<sensor_msgs::msg::PointCloud2>("roboscanPointCloud", qos_profile); 

    printf("Init...\n");
    roboscanPublisher::initialise();
    roboscanPublisher::setParameters();

    callback_handle_ = this->add_on_set_parameters_callback(std::bind(&roboscanPublisher::parametersCallback, this, std::placeholders::_1));

	int numSteps = NUM_COLORS;
	unsigned char red, green, blue;

	for(int i=0;  i< numSteps; i++)
	{
	  createColorMapPixel(numSteps, i, red, green, blue);
	  colorVector.push_back(Vec3b(blue, green, red));
	}

	mouseXpos = -1;
	mouseYpos = -1;
	bInitCmd = false;
	bSetReconfigure = false;
	runThread = true;
    publisherThread.reset(new boost::thread(boost::bind(&roboscanPublisher::thread_callback, this)));

    printf("\nRun rqt to view the image!\n");

} 

roboscanPublisher::~roboscanPublisher()
{
	runThread = false;
	publisherThread->join();
	cv::destroyAllWindows();

    printf("\nEnd roboscanPublisher()!\n");
}

void roboscanPublisher::thread_callback()
{
	static Packet databuf(IMAGE_DATA_SIZE);
	int currentFrame_id = 0;
	
	while(runThread){

		if( bSetReconfigure ){
			bSetReconfigure = false;
			setReconfigure();
		}

		if( bInitCmd && startStreaming(databuf) ){
			int width = lidarParam.roi_rightX-lidarParam.roi_leftX+1;
			int height = (lidarParam.roi_bottomY-lidarParam.roi_topY+1)*2;
			Frame frame(interface.getDataType(), currentFrame_id++, width, height, lidarParam.hdr_mode);
			frame.sortData(databuf, maxDistance);
			publishFrame(&frame);
			waitKey(10);
		}
		else{
			if( lidarParam.imageType != Frame::NONTYPE){
//				printf("streaming network error::\n");
			}
			usleep(10000);
		}

	}

	printf("End thread_callback\n");
}


rcl_interfaces::msg::SetParametersResult roboscanPublisher::parametersCallback( const std::vector<rclcpp::Parameter> &parameters)
{
	rcl_interfaces::msg::SetParametersResult result;
	result.successful = true;
	result.reason = "success";
	// Here update class attributes, do some actions, etc.
	for (const auto &param: parameters)
	{
		if (param.get_name() == "B. lensType")
		{
			lidarParam.lensType = param.as_int();
		}
		else if (param.get_name() == "C. imageType")
		{
			lidarParam.imageType = param.as_int();
		}
		else if (param.get_name() == "D. hdr_mode")
		{
			lidarParam.hdr_mode = param.as_int();
		}
		else if (param.get_name() == "E. int0")
		{
			lidarParam.int0 = param.as_int();
		}
		else if (param.get_name() == "F. int1")
		{
			lidarParam.int1 = param.as_int();
		}
		else if (param.get_name() == "G. int2")
		{
			lidarParam.int2 = param.as_int();
		}
		else if (param.get_name() == "H. intGr")
		{
			lidarParam.intGr = param.as_int();
		}
		else if (param.get_name() == "I. minAmplitude")
		{
			lidarParam.minAmplitude = param.as_int();
		}
		else if (param.get_name() == "J. modIndex")
		{
			lidarParam.frequencyModulation = param.as_int();
		}
		else if (param.get_name() == "K. channel")
		{
			lidarParam.channel = param.as_int();
		}
		else if (param.get_name() == "L. roi_leftX")
		{
			lidarParam.roi_leftX = param.as_int();
		}
		else if (param.get_name() == "M. roi_topY")
		{
			lidarParam.roi_topY = param.as_int();
		}
		else if (param.get_name() == "N. roi_rightX")
		{
			lidarParam.roi_rightX = param.as_int();
		}
		else if (param.get_name() == "O. roi_bottomY")
		{
//			lidarParam.roi_bottomY = param.as_int();
		}
		else if (param.get_name() == "P. transformAngle")
		{
			lidarParam.transformAngle = param.as_double();
		}
		else if (param.get_name() == "Q. cutPixels")
		{
			lidarParam.cutPixels = param.as_int();
		}
		else if (param.get_name() == "Q. cutPixels")
		{
			lidarParam.cutPixels = param.as_int();
		}
		else if (param.get_name() == "R. medianFilter")
		{
			lidarParam.medianFilter = param.as_bool();
		}
		else if (param.get_name() == "S. averageFilter")
		{
			lidarParam.averageFilter = param.as_bool();
		}
		else if (param.get_name() == "T. temporalFilterFactor")
		{
			lidarParam.temporalFilterFactor = param.as_double();
		}
		else if (param.get_name() == "T. temporalFilterFactorThreshold")
		{
			lidarParam.temporalFilterThreshold = param.as_int();
		}
		else if (param.get_name() == "U. edgeFilterThreshold")
		{
			lidarParam.edgeFilterThreshold = param.as_int();
		}
		/*
		else if (param.get_name() == "W. temporalEdgeThresholdLow")
		{
			lidarParam.temporalEdgeThresholdLow = param.as_int();
		}
		else if (param.get_name() == "X. temporalEdgeThresholdHigh")
		{
			lidarParam.temporalEdgeThresholdHigh = param.as_int();
		}
		*/
		else if (param.get_name() == "V. interferenceDetectionLimit")
		{
			lidarParam.interferenceDetectionLimit = param.as_int();
		}
		else if (param.get_name() == "V. useLastValue")
		{
			lidarParam.useLastValue = param.as_bool();
		}
		else if (param.get_name() == "A. cvShow")
		{
			lidarParam.cvShow = param.as_bool();
		}
		else if( param.get_name() == "Y. ipAddr")
		{
			lidarParam.ipAddr = param.as_string();
		}
	}
	bSetReconfigure = true;
	return result;
}


void roboscanPublisher::setReconfigure()
{
	//int0 = integrationTime0;

	interface.setIpAddr(lidarParam.ipAddr);

	if( !bInitCmd ){
		bInitCmd = interface.initCommand();
		if( bInitCmd == false ){
			printf("setReconfigure error\n");
			return;
		}
	}
	
	if( !interface.stopStream() ){
		printf("setReconfigure error\n");
		return;
	}

	printf("setReconfigure\n");
	interface.setLoadConfig(lidarParam.imageType == Frame::GRAYSCALE ? 0 : 1);

	interface.setMinAmplitude(lidarParam.minAmplitude);
	interface.setIntegrationTime2d(lidarParam.intGr);//, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	interface.setIntegrationTime3d(lidarParam.int0);//, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	interface.setIntegrationTime3dHdr(lidarParam.int1);
		
	interface.setHDRMode((uint8_t)lidarParam.hdr_mode);

	interface.setAdcOverflow(lidarParam.bAdcOverflow);
	interface.setSaturation(lidarParam.bSaturation);

	if(lidarParam.frequencyModulation == 0) lidarParam.modIndex = 1;
	else if(lidarParam.frequencyModulation == 1)  lidarParam.modIndex = 0;
	else if(lidarParam.frequencyModulation == 2)  lidarParam.modIndex = 2;
	else    lidarParam.modIndex = 3;

	maxDistance = lidarParam.frequencyModulation == 0 ? 6500.0f : lidarParam.frequencyModulation == 1 ? 12500.0f : lidarParam.frequencyModulation == 2 ? 25000.0f : 50000.0f;

	interface.setModulation(lidarParam.frequencyModulation);
	interface.setRoi(lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);

	if(lidarParam.old_lensCenterOffsetX != lidarParam.lensCenterOffsetX || lidarParam.old_lensCenterOffsetY != lidarParam.lensCenterOffsetY || lidarParam.old_lensType != lidarParam.lensType){
	  cartesianTransform.initLensTransform(sensorPixelSizeMM, LensWidth, LensHeight, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType);
	  lidarParam.old_lensCenterOffsetX = lidarParam.lensCenterOffsetX;
	  lidarParam.old_lensCenterOffsetY = lidarParam.lensCenterOffsetY;
	  lidarParam.old_lensType = lidarParam.lensType;

	}

	interface.startStream();
	//startStream = true;
	printf("setReconfigure OK!\n\n");
}


void roboscanPublisher::initialise()
{
	printf("Init roboscan_nsl1110 node\n");

	
	cv::namedWindow(WIN_NAME, cv::WINDOW_AUTOSIZE);
	cv::setMouseCallback(WIN_NAME, callback_mouse_click, NULL);

	lidarParam.ipAddr = "192.168.0.210";
	lidarParam.lensType = 2;
	lidarParam.lensCenterOffsetX = 0;
	lidarParam.lensCenterOffsetY = 0;
	lidarParam.startStream = 0;
	lidarParam.imageType = 3; 
	lidarParam.hdr_mode = 0; //0 - hdr off, 1 - hdr spatial
	lidarParam.int0 = 1500;
	lidarParam.int1 = 100;
	lidarParam.int2 = 50;
	lidarParam.intGr = 20000; //integration times
	lidarParam.frequencyModulation = 1;
	lidarParam.modIndex = 0;
	lidarParam.channel = 0;
	lidarParam.minAmplitude = 100;
	lidarParam.medianFilter = false;
	lidarParam.averageFilter = false;
	lidarParam.temporalFilterFactor = 0;
	lidarParam.temporalFilterThreshold = 0;
	lidarParam.edgeFilterThreshold = 0;
	lidarParam.temporalEdgeThresholdLow = 0;
	lidarParam.temporalEdgeThresholdHigh = 0;
	lidarParam.interferenceDetectionLimit = 0;
	lidarParam.useLastValue = false;
	lidarParam.cartesian = true;
	lidarParam.publishPointCloud = true;

	lidarParam.roi_leftX = 4;
	lidarParam.roi_topY = 6;
	lidarParam.roi_rightX = 323;
	lidarParam.roi_bottomY = 125;	// Don't modify 
	lidarParam.transformAngle = 0;
	lidarParam.cutPixels = 0;

	lidarParam.cvShow = false;
	//roi_height

	maxDistance = lidarParam.frequencyModulation == 0 ? 6500.0f : lidarParam.frequencyModulation == 1 ? 12500.0f : lidarParam.frequencyModulation == 2 ? 25000.0f : 50000.0f;

	rclcpp::Parameter pCvShow("A. cvShow", lidarParam.cvShow);
	rclcpp::Parameter pLensType("B. lensType", lidarParam.lensType);
	rclcpp::Parameter pImageType("C. imageType", lidarParam.imageType);
	rclcpp::Parameter pHdr_mode("D. hdr_mode", lidarParam.hdr_mode);
	rclcpp::Parameter pInt0("E. int0", lidarParam.int0);
	rclcpp::Parameter pInt1("F. int1", lidarParam.int1);
//	rclcpp::Parameter pInt2("G. int2", lidarParam.int2);
	rclcpp::Parameter pIntGr("H. intGr", lidarParam.intGr);
	rclcpp::Parameter pMinAmplitude("I. minAmplitude", lidarParam.minAmplitude);
	rclcpp::Parameter pModIndex("J. modIndex", lidarParam.frequencyModulation);
//	rclcpp::Parameter pChannel("K. channel", lidarParam.channel);
	rclcpp::Parameter pRoi_leftX("L. roi_leftX", lidarParam.roi_leftX);
	rclcpp::Parameter pRoi_topY("M. roi_topY", lidarParam.roi_topY);
	rclcpp::Parameter pRoi_rightX("N. roi_rightX", lidarParam.roi_rightX);
	rclcpp::Parameter pRoi_bottomY("O. roi_bottomY", lidarParam.roi_bottomY);
	rclcpp::Parameter pTransformAngle("P. transformAngle", lidarParam.transformAngle);
	rclcpp::Parameter pCutpixels("Q. cutPixels", lidarParam.cutPixels);
//	rclcpp::Parameter pMedianFilter("R. medianFilter", lidarParam.medianFilter);
//	rclcpp::Parameter pAverageFilter("S. averageFilter", lidarParam.averageFilter);
//	rclcpp::Parameter pTemporalFilterFactor("T. temporalFilterFactor", lidarParam.temporalFilterFactor);
//	rclcpp::Parameter pTemporalFilterThreshold("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold);
//	rclcpp::Parameter pEdgeFilterThreshold("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold);
	//rclcpp::Parameter pTemporalEdgeThresholdLow("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow);
	//rclcpp::Parameter pTemporalEdgeThresholdHigh("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh);
//	rclcpp::Parameter pInterferenceDetectionLimit("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit);
//	rclcpp::Parameter pUseLastValue("V. useLastValue", lidarParam.useLastValue);

	rclcpp::Parameter pIpAddr("Y. ipAddr", lidarParam.ipAddr);


	this->declare_parameter<bool>("A. cvShow", lidarParam.cvShow);
	this->declare_parameter<int>("B. lensType", lidarParam.lensType);
	this->declare_parameter<int>("C. imageType", lidarParam.imageType);
	this->declare_parameter<int>("D. hdr_mode", lidarParam.hdr_mode);
	this->declare_parameter<int>("E. int0", lidarParam.int0);
	this->declare_parameter<int>("F. int1", lidarParam.int1);
//	this->declare_parameter<int>("G. int2", lidarParam.int2);
	this->declare_parameter<int>("H. intGr",lidarParam.intGr);
	this->declare_parameter<int>("I. minAmplitude", lidarParam.minAmplitude);
	this->declare_parameter<int>("J. modIndex", lidarParam.frequencyModulation);
//	this->declare_parameter<int>("K. channel", lidarParam.channel);
	this->declare_parameter<int>("L. roi_leftX", lidarParam.roi_leftX);
	this->declare_parameter<int>("M. roi_topY", lidarParam.roi_topY);
	this->declare_parameter<int>("N. roi_rightX", lidarParam.roi_rightX);
	this->declare_parameter<int>("O. roi_bottomY", lidarParam.roi_bottomY);
	this->declare_parameter<double>("P. transformAngle", lidarParam.transformAngle);
	this->declare_parameter<int>("Q. cutPixels", lidarParam.transformAngle);
//	this->declare_parameter<bool>("R. medianFilter", lidarParam.medianFilter);
//	this->declare_parameter<bool>("S. averageFilter", lidarParam.averageFilter);
//	this->declare_parameter<double>("T. temporalFilterFactor", lidarParam.temporalFilterFactor);
//	this->declare_parameter<int>("T. temporalFilterFactorThreshold", lidarParam.temporalFilterThreshold);
//	this->declare_parameter<int>("U. edgeFilterThreshold", lidarParam.edgeFilterThreshold);
	//this->declare_parameter<int>("W temporalEdgeThresholdLow", lidarParam.temporalEdgeThresholdLow);
	//this->declare_parameter<int>("X temporalEdgeThresholdHigh", lidarParam.temporalEdgeThresholdHigh);
//	this->declare_parameter<int>("V. interferenceDetectionLimit", lidarParam.interferenceDetectionLimit);
//	this->declare_parameter<bool>("V. useLastValue", lidarParam.useLastValue);
	this->declare_parameter<string>("Y. ipAddr", lidarParam.ipAddr);


	this->set_parameter(pCvShow);
	this->set_parameter(pLensType);
	this->set_parameter(pImageType);
	this->set_parameter(pHdr_mode);
	this->set_parameter(pInt0);
	this->set_parameter(pInt1);
//	this->set_parameter(pInt2);
	this->set_parameter(pIntGr);
	this->set_parameter(pMinAmplitude);
	this->set_parameter(pModIndex);
//	this->set_parameter(pChannel);
	this->set_parameter(pRoi_leftX);
	this->set_parameter(pRoi_topY);
	this->set_parameter(pRoi_rightX);
	this->set_parameter(pRoi_bottomY);
	this->set_parameter(pTransformAngle);
	this->set_parameter(pCutpixels);
//	this->set_parameter(pMedianFilter);
//	this->set_parameter(pAverageFilter);
//	this->set_parameter(pTemporalFilterFactor);
//	this->set_parameter(pTemporalFilterThreshold);
//	this->set_parameter(pEdgeFilterThreshold);
	//this->set_parameter(pTemporalEdgeThresholdLow);
	//this->set_parameter(pTemporalEdgeThresholdHigh);
//	this->set_parameter(pInterferenceDetectionLimit);
//	this->set_parameter(pUseLastValue);
	this->set_parameter(pIpAddr);


	cartesianTransform.initLensTransform(sensorPixelSizeMM, LensWidth, LensHeight, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType); //0.02 mm - sensor pixel size


}


void roboscanPublisher::setParameters()
{
	interface.setIpAddr(lidarParam.ipAddr);
	bInitCmd = interface.initCommand();
	printf("setParameters init = %d\n", bInitCmd);

	if( bInitCmd == false ) return;
	
	interface.setMinAmplitude(lidarParam.minAmplitude);
	interface.setIntegrationTime2d(lidarParam.intGr);//, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	interface.setIntegrationTime3d(lidarParam.int0);//, lidarParam.int1, lidarParam.int2, lidarParam.intGr, lidarParam.grayscaleIlluminationMode);
	interface.setIntegrationTime3dHdr(lidarParam.int1);
	    
	interface.setHDRMode((uint8_t)lidarParam.hdr_mode);

	interface.setAdcOverflow(lidarParam.bAdcOverflow);
	interface.setSaturation(lidarParam.bSaturation);

	if(lidarParam.frequencyModulation == 0) lidarParam.modIndex = 1;
	else if(lidarParam.frequencyModulation == 1)  lidarParam.modIndex = 0;
	else    lidarParam.modIndex = lidarParam.frequencyModulation;

	maxDistance = lidarParam.frequencyModulation == 0 ? 6500.0f : lidarParam.frequencyModulation == 1 ? 12500.0f : lidarParam.frequencyModulation == 2 ? 25000.0f : 50000.0f;

	interface.setModulation(lidarParam.frequencyModulation);
	interface.setRoi(lidarParam.roi_leftX, lidarParam.roi_topY, lidarParam.roi_rightX, lidarParam.roi_bottomY);


	if(lidarParam.old_lensCenterOffsetX != lidarParam.lensCenterOffsetX || lidarParam.old_lensCenterOffsetY != lidarParam.lensCenterOffsetY || lidarParam.old_lensType != lidarParam.lensType){
	  cartesianTransform.initLensTransform(sensorPixelSizeMM, LensWidth, LensHeight, lidarParam.lensCenterOffsetX, lidarParam.lensCenterOffsetY, lidarParam.lensType);
	  lidarParam.old_lensCenterOffsetX = lidarParam.lensCenterOffsetX;
	  lidarParam.old_lensCenterOffsetY = lidarParam.lensCenterOffsetY;
	  lidarParam.old_lensType = lidarParam.lensType;
	}
	lidarParam.cvShow = false;
	interface.startStream();

	printf("setParameters OK\n");

}

double roboscanPublisher::interpolate( double x, double x0, double y0, double x1, double y1){

    if( x1 == x0 ){
        return y0;
    } else {
        return ((x-x0)*(y1-y0)/(x1-x0) + y0);
    }

}

void roboscanPublisher::createColorMapPixel(int numSteps, int indx, unsigned char &red, unsigned char &green, unsigned char &blue)
{
    double k = 1;
    double BIT0 = -0.125 * k - 0.25;
    double BIT1 = BIT0 + 0.25 * k;
    double BIT2 = BIT1 + 0.25 * k;
    double BIT3 = BIT2 + 0.25 * k;

    double G0 = BIT1;
    double G1 = G0 + 0.25 * k;
    double G2 = G1 + 0.25 * k;
    double G3 = G2 + 0.25 * k + 0.125;

    double R0 = BIT2;
    double R1 = R0 + 0.25 * k;
    double R2 = R1 + 0.25 * k;
    double R3 = R2 + 0.25 * k + 0.25;

    double i = (double)indx/(double)numSteps - 0.25 * k;

    if( i>= R0 && i < R1 ){
        red = interpolate(i, R0, 0, R1, 255);
    } else if((i >= R1) && (i < R2)){
        red = 255;
    } else if((i >= R2) && (i < R3)) {
        red = interpolate(i, R2, 255, R3, 0);
    } else {
        red = 0;
    }

    if( i>= G0 && i < G1 ){
        green = interpolate(i, G0, 0, G1, 255);
    } else if((i>=G1)&&(i<G2)){
        green = 255;
    } else if((i >= G2)&&(i < G3)){
        green = interpolate(i, G2, 255, G3, 0);
    } else {
        green = 0;
    }


    if( i>= BIT0 && i < BIT1 ){
        blue = interpolate(i, BIT0, 0, BIT1, 255);
    } else if((i >= BIT1)&&(i < BIT2)){
        blue = 255;
    } else if((i >= BIT2)&&(i < BIT3)) {
        blue = interpolate(i, BIT2, 255, BIT3, 0);
    } else{
        blue = 0;
    }

}

bool roboscanPublisher::startStreaming(Packet &databuf)
{
	bool bRet = false;

	if( !interface.isStream() ) return bRet;
	
	switch(lidarParam.imageType) {
	case Frame::GRAYSCALE:
	  bRet = interface.streamGrayscale(databuf);
//	  printf("streaming grayscale\n");
	  break;
	case Frame::DISTANCE:
	  bRet = interface.streamDistance(databuf);
//	  printf("streaming distance\n");
	  break;
	case Frame::AMPLITUDE:
	  bRet = interface.streamDistanceAmplitude(databuf);
//	  printf("streaming distance-amplitude\n");
	  break;
	case Frame::DCS:
	  bRet = interface.streamDCS(databuf);
//	  printf("streaming DCS\n");
	  break;
	default:
	  break;
	}

	return bRet;
}


int roboscanPublisher::Convert_To_RGB24( float fValue, RGB888Pixel *nRGBData, float fMinValue, float fMaxValue)
{
	if(fValue == Frame::ADC_OVERFLOW)
	{
	  nRGBData->r = 169;//R
	  nRGBData->g = 14;//G
	  nRGBData->b = 255;//B
	}
	else if(fValue == Frame::SATURATION)
	{
	  nRGBData->r = 255;//R
	  nRGBData->g = 0;//G
	  nRGBData->b = 128;//B
	}
	else if( fValue == Frame::LOW_AMPLITUDE )
	{
	  nRGBData->r = 0;//R
	  nRGBData->g = 0;//G
	  nRGBData->b = 0;//B
	}
	else if(fValue == 0) //Invalide Pixel
	{
	  nRGBData->r = 0;//R
	  nRGBData->g = 0;//G
	  nRGBData->b = 0;//B
	}
	else if(fValue < fMinValue)
	{
	  nRGBData->r = 255;//R
	  nRGBData->g = 0;//G
	  nRGBData->b = 0;//B
	}
	else if(fValue > fMaxValue)
	{
	  nRGBData->r = 255;//R
	  nRGBData->g = 0;//G
	  nRGBData->b = 255;//B
	}
	else
	{
	  float fColorWeight;
	  fColorWeight = (fValue-fMinValue) / (fMaxValue-fMinValue);

	  if( (fColorWeight <= 1.0f) && (fColorWeight > 0.8f) )
	  {
	    nRGBData->r = (unsigned char)(255 * ((fColorWeight - 0.8f) / 0.2f));//값에 따라 증가
	    nRGBData->g = 0;
	    nRGBData->b = 255;
	  } 
	  else if( (fColorWeight <= 0.8f) && (fColorWeight > 0.6f) )
	  {
	    nRGBData->r = 0;
	    nRGBData->g = (unsigned char)(255 * (1.0f - (fColorWeight - 0.6f) / 0.2f));//값에 따라 감소
	    nRGBData->b = 255;
	  }
	  else if( (fColorWeight <= 0.6f) && (fColorWeight > 0.4f) )
	  {
	    nRGBData->r = 0;
	    nRGBData->g = 255;
	    nRGBData->b = (unsigned char)(255 * ((fColorWeight - 0.4f) / 0.2f));//값에 따라 증가
	  }
	  else if( (fColorWeight <= 0.4f) && (fColorWeight > 0.2f) )
	  {
	    nRGBData->r = (unsigned char)(255 * (1.0f - (fColorWeight - 0.2f) / 0.2f));//값에 따라 감소
	    nRGBData->g = 255;
	    nRGBData->b = 0;
	  }
	  else if( (fColorWeight <= 0.2f) && (fColorWeight >= 0.0f) )
	  {
	    nRGBData->r = 255;
	    nRGBData->g = (unsigned char)(255 * ((fColorWeight - 0.0f) / 0.2f));//값에 따라 증가
	    nRGBData->b = 0;
	  }
	  else
	  {
	    nRGBData->r = 0;
	    nRGBData->g = 0;
	    nRGBData->b = 0;
	  }
	}

	return true;
}

void roboscanPublisher::setGrayscaleColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{   
	if (value == Frame::SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == Frame::ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if (value == Frame::LOW_AMPLITUDE)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 255;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value <= 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else
	{
		int color = value * (255/end_range);

		//printf("color index = %d\n", color);

		imageLidar.at<Vec3b>(y, x)[0] = color;
		imageLidar.at<Vec3b>(y, x)[1] = color;
		imageLidar.at<Vec3b>(y, x)[2] = color; 
	}
}

void roboscanPublisher::setAmplitudeColor(cv::Mat &imageLidar, int x, int y, int value, double end_range )
{
	if( value == Frame::LOW_AMPLITUDE )
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value == Frame::SATURATION)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 128;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 255; 
	}
	else if (value == Frame::ADC_OVERFLOW)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 255;
		imageLidar.at<Vec3b>(y, x)[1] = 14;
		imageLidar.at<Vec3b>(y, x)[2] = 169; 
	}
	else if(value == Frame::BAD_PIXEL)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if(value == 0)
	{
		imageLidar.at<Vec3b>(y, x) = colorVector.at(0);
	}
	else if (value < 0)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else if (value > end_range)
	{
		imageLidar.at<Vec3b>(y, x)[0] = 0;
		imageLidar.at<Vec3b>(y, x)[1] = 0;
		imageLidar.at<Vec3b>(y, x)[2] = 0; 
	}
	else{
		int index = value * (NUM_COLORS / end_range);
		if( index < 0 ){
			printf("error index = %d\n", index);
			index = 0;
		}
		else if( index >= (int)colorVector.size() ){
			index = colorVector.size()-1;
		}
		
		imageLidar.at<Vec3b>(y, x) = colorVector.at(index);
	}

}

cv::Mat roboscanPublisher::addDistanceInfo(cv::Mat distMat, std::vector<uint16_t> dist2BData, int width)
{
//	printf("xpos = %d ypos = %d show = %d\n", mouseXpos, mouseYpos, lidarParam.cvShow);
	if( mouseXpos > 0 || (mouseYpos > 0 && mouseYpos < 240)){
		// mouseXpos, mouseYpos
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));

		cv::line(distMat, cv::Point(mouseXpos-10, mouseYpos), cv::Point(mouseXpos+10, mouseYpos), cv::Scalar(255, 255, 0), 2);
		cv::line(distMat, cv::Point(mouseXpos, mouseYpos-10), cv::Point(mouseXpos, mouseYpos+10), cv::Scalar(255, 255, 0), 2);

		if( mouseXpos >=320 ) mouseXpos -= 320;

		
		std::string dist_caption = cv::format("X:%d, Y:%d, %d mm", mouseXpos, mouseYpos, dist2BData[mouseYpos*width + mouseXpos]);

		putText(infoImage, dist_caption.c_str(), cv::Point(10, 30), cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 0, 0));
		cv::vconcat(distMat, infoImage, distMat);
	}
	else{
		cv::Mat infoImage(50, distMat.cols, CV_8UC3, Scalar(255, 255, 255));
		cv::vconcat(distMat, infoImage, distMat);
	}

	return distMat;
}
void roboscanPublisher::publishFrame(Frame *frame)
{
	int x, y, k, l;
	static rclcpp::Clock s_rclcpp_clock;
	auto data_stamp = s_rclcpp_clock.now();
//	std::chrono::system_clock::time_point update_start = std::chrono::system_clock::now();

	cv::Mat imageLidar(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));
	cv::Mat amplitudeLidar(frame->height, frame->width, CV_8UC3, Scalar(255, 255, 255));

	if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE ){
		sensor_msgs::msg::Image imgDistance;

		imgDistance.header.stamp = data_stamp;
		imgDistance.header.frame_id = "roboscan_frame";
		imgDistance.height = static_cast<uint32_t>(frame->height);
		imgDistance.width = static_cast<uint32_t>(frame->width);
		imgDistance.encoding = sensor_msgs::image_encodings::MONO16;
		imgDistance.step = imgDistance.width * frame->px_size;
		imgDistance.is_bigendian = 0;
		imgDistance.data = frame->distData;
		imgDistancePub->publish(imgDistance);
	}

	if(frame->dataType == Frame::AMPLITUDE ){
		sensor_msgs::msg::Image imgAmpl;

		imgAmpl.header.stamp = data_stamp;
		imgAmpl.header.frame_id = "roboscan_frame";
		imgAmpl.height = static_cast<uint32_t>(frame->height);
		imgAmpl.width = static_cast<uint32_t>(frame->width);
		imgAmpl.encoding = sensor_msgs::image_encodings::MONO16;
		imgAmpl.step = imgAmpl.width * frame->px_size;
		imgAmpl.is_bigendian = 0;
		imgAmpl.data = frame->amplData;
		imgAmplPub->publish(imgAmpl);
	}

	if(frame->dataType == Frame::DCS){
		sensor_msgs::msg::Image imgDCS;

		imgDCS.header.stamp = data_stamp;
		imgDCS.header.frame_id = "roboscan_frame";
		imgDCS.height = static_cast<uint32_t>(frame->height) * 4;
		imgDCS.width = static_cast<uint32_t>(frame->width);
		imgDCS.encoding = sensor_msgs::image_encodings::MONO16;
		imgDCS.step = imgDCS.width * frame->px_size;
		imgDCS.is_bigendian = 0;
		imgDCS.data = frame->dcsData;
		imgDCSPub->publish(imgDCS);
	}

	getMouseEvent(mouseXpos, mouseYpos);

	if(frame->dataType == Frame::GRAYSCALE){
		uint16_t gray; 
		for(l=0, y=0; y< frame->height; y++){
			for(x=0; x< frame->width; x++, l+=2){
				gray = (frame->amplData[l+1] << 8)	+ frame->amplData[l];
				if( gray < 8000 ) 		gray -= 2048;
				setGrayscaleColor(imageLidar, x, y, gray, 2047);
			}
		}
	}
	else 
	{
		const size_t nPixel = frame->width * frame->height;
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
		cloud->header.frame_id = "roboscan_frame";
		cloud->header.stamp = pcl_conversions::toPCL(data_stamp);
		cloud->width = static_cast<uint32_t>(frame->width);
		cloud->height = static_cast<uint32_t>(frame->height);
		cloud->is_dense = false;
		cloud->points.resize(nPixel);

		uint16_t distance = 0;
		uint16_t amplitude = 0;
		double px, py, pz;

		RGB888Pixel* pTex1 = new RGB888Pixel[1];

		for(k=0, l=0, y=0; y< frame->height; y++)
		{
			for(x=0; x< frame->width; x++, k++, l+=2)
			{

				pcl::PointXYZI &p = cloud->points[k];
				if(frame->dataType == Frame::DISTANCE || frame->dataType == Frame::AMPLITUDE )
					distance = (frame->distData[l+1] << 8) + frame->distData[l];

				if(frame->dataType == Frame::DCS)
					distance = (frame->dcsData[l+1] << 8) + frame->dcsData[l];

				if(frame->dataType == Frame::AMPLITUDE)
					amplitude = (frame->amplData[l+1] << 8)  + frame->amplData[l];

				if( !(y > -x + lidarParam.cutPixels
						&& y > x - (319-lidarParam.cutPixels)
						&& y < x + (239-lidarParam.cutPixels)
						&& y < -x + lidarParam.cutPixels + (239-lidarParam.cutPixels) + (319-lidarParam.cutPixels)))
				{
					distance = Frame::LOW_AMPLITUDE;
				}
				
				Convert_To_RGB24((double)distance, pTex1, 0.0f, maxDistance);
				imageLidar.at<Vec3b>(y, x)[0] = pTex1->b;
				imageLidar.at<Vec3b>(y, x)[1] = pTex1->g;
				imageLidar.at<Vec3b>(y, x)[2] = pTex1->r;

				if(frame->dataType == Frame::AMPLITUDE){
					setAmplitudeColor(amplitudeLidar, x, y, amplitude, 4095);
				}
				
#if 0
				if(x == 160 && y == 120)
				{
					RCLCPP_INFO(this->get_logger(), "distance : %d [%02x:%02x]", distance, frame->distData[l+1], frame->distData[l]);
				}
#endif
				if (distance > 0 && distance < Frame::PIXEL_VALID_DATA)
				{
					if(lidarParam.cartesian)
					{  
						cartesianTransform.transformPixel(x, y, distance, px, py, pz, lidarParam.transformAngle);
						p.x = static_cast<float>(pz / 1000.0); //mm -> m
						p.y = static_cast<float>(px / 1000.0);
						p.z = static_cast<float>(-py / 1000.0);

						if(frame->dataType == Frame::AMPLITUDE) p.intensity = static_cast<float>(amplitude);
						else p.intensity = static_cast<float>(pz / 1000.0);
					}
					else
					{
						p.x = distance / 1000.0;
						p.y = -(160-x) / 100.0;
						p.z = (120-y) / 100.0;
						if(frame->dataType == Frame::AMPLITUDE) p.intensity =  static_cast<float>(amplitude);
						else p.intensity = static_cast<float>(distance / 1000.0);
					} 
				}
				else
				{
					p.x = std::numeric_limits<float>::quiet_NaN();
					p.y = std::numeric_limits<float>::quiet_NaN();
					p.z = std::numeric_limits<float>::quiet_NaN();
					p.intensity = std::numeric_limits<float>::quiet_NaN();
				}		  

			}
		}


		sensor_msgs::msg::PointCloud2 msg;
		pcl::toROSMsg(*cloud, msg);
		msg.header.stamp = data_stamp;
		msg.header.frame_id = "roboscan_frame";
		pointcloudPub->publish(msg);  

		delete[] pTex1; 	   
	}

	
	if(lidarParam.cvShow == true)
	{
		if(frame->dataType == Frame::AMPLITUDE){
			cv::hconcat(imageLidar, amplitudeLidar, imageLidar);

			imageLidar = addDistanceInfo(imageLidar, frame->dist2BData, frame->width);
		}
		else{
			
			if( frame->dataType == Frame::DISTANCE ){
				imageLidar = addDistanceInfo(imageLidar, frame->dist2BData, frame->width);
			}
		}

		imshow(WIN_NAME, imageLidar);
	}

//	std::chrono::system_clock::time_point update_end = std::chrono::system_clock::now();
//	std::chrono::milliseconds timeCnt = std::chrono::duration_cast<std::chrono::milliseconds>(update_end - update_start);
//	printf("update-color time = %ld ms\n", timeCnt.count());
}

void roboscanPublisher::getMouseEvent( int &mouse_xpos, int &mouse_ypos )
{
	mouse_xpos = x_start;
	mouse_ypos = y_start;
}


int main(int argc, char ** argv)
{
	(void) argc;
	(void) argv;
	rclcpp::init(argc, argv);

	auto node = std::make_shared<roboscanPublisher>();


	rclcpp::spin(node);
	rclcpp::shutdown();

	printf("end main()\n");
	return 0;
}
