#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include "frame.hpp"
#include "interface.hpp"


std::chrono::system_clock::time_point start;
std::chrono::system_clock::time_point end;

#define ENABLE_DRNU_TEMPERATURE


const char *initialcmd[]=
{
	"stopVideo\n",
	"version\n",
	"getIcVersion\n",
	"getModulationFrequencies\n",
	"getCalibrationTypeForFreqIdx 0\n",
	"getCalibrationTypeForFreqIdx 1\n",
	"getCalibrationTypeForFreqIdx 2\n",
	"getCalibrationTypeForFreqIdx 3\n",
	"getCalibrationTypeForFreqIdx 4\n",
	"getCalibrationTypeForFreqIdx 5\n",
	"getCalibrationTypeForFreqIdx 6\n",
	"correctAsymDCS 1\n",
	"correctAsymDCS 0\n",
	"nloopFilter 6\n",
	"nloopFilter 10\n",
	"isDRNUAvailable\n",
	"isGrayscaleCorrectionAvailable\n",
	"isDRNUAvailable\n",
	"correctDRNU 0\n",
	"correctTemperature 0\n",
	"correctAmbientLight 0",
	"setKalmanK 0.1000\n",
	"setKalmanK 0.1000\n",
	"setKalmanQ 2.0000\n",
	"setKalmanKdiff 0.10\n",
	"setKalmanThreshold 600\n",
	"setKalmanNumCheck 2\n",
#ifdef MODULATION_24MHZ
	"setModulationFrequency 0\n",
	"setKalmanThreshold 1200\n",
	"setKalmanThreshold2 1200\n",
#else			
	"setModulationFrequency 1\n",
	"setKalmanThreshold 600\n",
	"setKalmanThreshold2 600\n",
#endif	
	"setABS 0\n",
	"selectMode -1\n",
	"selectMode 0\n",
#ifdef ENABLE_DRNU_TEMPERATURE	
	"setABS 1\n",
	"correctDRNU 2\n",
	"correctTemperature 2\n",
#endif	
	"isDRNUAvailable\n",
	"enableDefaultOffset 0\n",
	"enablePiDelay 1\n",
	"enableDualMGX 0\n",	// enableDualMGX 1 <-> enableHDR 0

	"enableHDR 0\n",
	"setHysteresis 10\n",
//	"setMinAmplitude 50\n",
#ifdef ENABLE_DRNU_TEMPERATURE	
	"enableDefaultOffset 1\n",
#endif	
	"getOffset\n",
	"setIntegrationTime3DHDR 100\n",

	"setKalmanK 0.6000\n",

	"enableVerticalBinning 1\n",
	"setRowReduction 1\n",
	"enableVerticalBinning 0\n",
	"setRowReduction 0\n",
	"enableHorizontalBinning 1\n",
	"enableHorizontalBinning 0\n",
	"setRowReduction 1\n",
	"setRowReduction 0\n",
	"selectPolynomial 1 0\n",
	"selectPolynomial 4 0\n",
	"selectPolynomial 4 1\n",
	"selectPolynomial 4 0\n",
	"getAmbientLightFactorOrg 34\n",
	"enableSaturation 1\n",
	"enableAdcOverflow 1\n",
	"setROI 4 323 6 125\n",
	"loadConfig 1\n",
};


namespace nanosys {

Interface::Interface() : tcpConnection(ioService),
    dataType(0),
    currentFrame_id(0),
    data(Packet(320*240*4*2)),
    currentFrameIdx(0),
    frameRxCnt(0)
{
	rxFrame = NULL;
    serverThread.reset(new boost::thread(boost::bind(&boost::asio::io_service::run, &ioService)));
	
	usedTemporalFactor = 0;
	usedTemporalThreshold = 0;
	usedEdgeThreshold = 0;
	filterSelector = 0;
}

Interface::~Interface() {
    stopStream();
    serverThread->interrupt();
    ioService.stop();
}

void Interface::initCommand() 
{
	int size = (sizeof(initialcmd)/sizeof(char*));

	for(int i = 0; i < size ; i++){
		tcpConnection.sendCommand((uint8_t *)initialcmd[i], strlen(initialcmd[i]));
	}

}


uint8_t Interface::isStream() {
	return isStreaming;
}

void Interface::startStream() {

	isStreaming = 1;

}

uint8_t Interface::getDataType(){
	return dataType;
}


void Interface::stopStream() {

	isStreaming = 0;
	
    setDataType(Frame::NONTYPE);
	
	char payloadStr[100];
	sprintf(payloadStr, "stopVideo\n");
    tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

bool Interface::streamDCS(Packet &databuf)
{
    setDataType(Frame::DCS);

	char payloadStr[100];
	sprintf(payloadStr, "getDCSSorted\n");
	return tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr), databuf);

}

bool Interface::streamDistanceAmplitude(Packet &databuf) {
	
    setDataType(Frame::AMPLITUDE);

	char payloadStr[100];
	sprintf(payloadStr, "getDistanceAndAmplitudeSorted\n");
    return tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr), databuf);
}

bool Interface::streamDistance(Packet &databuf) {

    setDataType(Frame::DISTANCE);

	char payloadStr[100];
	sprintf(payloadStr, "getDistanceSorted\n");
    return tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr), databuf);
}

bool Interface::streamGrayscale(Packet &databuf) {
    setDataType(Frame::GRAYSCALE);

	char payloadStr[100];
	sprintf(payloadStr, "getBWSorted\n");
    return tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr), databuf);
}

void Interface::setModulation(uint16_t modulation)
{
	char payloadStr[100];
	sprintf(payloadStr, "setModulationFrequency %d\n", modulation);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setMinAmplitude(uint16_t minAmplitude){
	char payloadStr[100];
	sprintf(payloadStr, "setMinAmplitude %d\n", minAmplitude);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setLoadConfig(int config)
{
	char payloadStr[100];
	sprintf(payloadStr, "loadConfig %d\n", config);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}
void Interface::setRoi(const uint16_t x0, const uint16_t y0, const uint16_t x1, const uint16_t y1)
{
	char payloadStr[100];
	sprintf(payloadStr, "setROI %d %d %d %d\n", x0, x1, y0, y1);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setIntegrationTime2d(uint16_t int2d)
{
	char payloadStr[100];
	sprintf(payloadStr, "setIntegrationTime2D %d\n", int2d);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setIntegrationTime3d(uint16_t int3d)
{
	char payloadStr[100];
	sprintf(payloadStr, "setIntegrationTime3D %d\n", int3d);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setIntegrationTime3dHdr(uint16_t int3d)
{
	char payloadStr[100];
	sprintf(payloadStr, "setIntegrationTime3DHDR %d\n", int3d);
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setHDRMode(uint8_t mode)
{
	char payloadStr[100];
	sprintf(payloadStr, "enableHDR %d\n", mode == 1 ? 1 : 0);	// 0:off, 1 : spatialHdr
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setAdcOverflow(uint8_t bAdcOverflow)
{
	char payloadStr[100];
	sprintf(payloadStr, "enableAdcOverflow %d\n", bAdcOverflow == 0 ? 0 : 1);	// 0:off, 1 : on
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}

void Interface::setSaturation(uint8_t bSaturation)
{
	char payloadStr[100];
	sprintf(payloadStr, "enableSaturation %d\n", bSaturation == 0 ? 0 : 1);	// 0:off, 1 : on
	tcpConnection.sendCommand((uint8_t *)payloadStr, strlen(payloadStr));
}


void Interface::insertValue8(std::vector<uint8_t> &output, const uint8_t value){
    output.push_back(static_cast<int8_t>(value));
}

void Interface::insertValue8(std::vector<uint8_t> &output, const int8_t value){
    output.push_back(value);
}

void Interface::insertValue(std::vector<uint8_t> &output, const uint16_t value)
{
    output.push_back(static_cast<int8_t>(value >> 8));
    output.push_back(static_cast<int8_t>(value & 0xFF));
}

void Interface::insertValue(std::vector<uint8_t> &output, const int16_t value)
{
    output.push_back(value >> 8);
    output.push_back(static_cast<int8_t>(value & 0xFF));
}

uint8_t Interface::boolToUint8(const bool value)
{
    if (value)  return 1;
    else        return 0;
}

int8_t Interface::boolToInt8(const bool value)
{
    if(value)   return 1;
    else        return 0;
}


void Interface::setDataType(uint8_t d) {
    dataType = d;
}

} //end namespace nanosys
