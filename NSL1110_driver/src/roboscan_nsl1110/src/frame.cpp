#include "frame.hpp"
#include <stdint.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#define MAX_PHASE  					30000

namespace nanosys {

Frame::Frame(uint16_t dataType_, uint64_t frame_id_, uint16_t width_, uint16_t height_, uint16_t hdrMode_) :
frame_id(frame_id_),
dataType(dataType_),
width(width_),
height(height_),
hdrMode(hdrMode_),
px_size(sizeof(uint16_t)),
distData(std::vector<uint8_t>(width * height * px_size)), //16 bit
amplData(std::vector<uint8_t>(width * height * px_size)), //16 bit
dcsData(std::vector<uint8_t> (width * height * px_size * 4)) //16 bit 4 dcs
{    
	
}

void Frame::getComparedHDRAmplitudeImage(uint16_t *pMemAmplitude, uint16_t *pMemDistance)
{
    int highAmpThreshold = 2000;

	for(int x = 0 ; x < width ; x++){
		for( int y = 0; y < height; y+=2) {

		    int amp1 = pMemAmplitude[y*width+x]; //amp[i][j]
		    int amp2 = pMemAmplitude[(y+1)*width+x]; //amp[i][j+1];
		    int ampCase = 1;
			
		    if(amp1 < highAmpThreshold && amp2 < highAmpThreshold){ // both amplitudes in range
		        if(amp2 > amp1) ampCase = 2;
		        else            ampCase = 1;
		    }else if(amp1 >= highAmpThreshold && amp2 <= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 2;
		    }else if(amp1 <= highAmpThreshold && amp2 >= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 1;
		    }else{ //both amplitudes out of range
		        if(amp2 < amp1) ampCase = 1;
		        else            ampCase = 2;
		    }

//		    if(/*ampCase == 1 &&*/ y >= 120){
			if(ampCase == 1 ){
				pMemDistance[(y+1)*width+x] = pMemDistance[y*width+x]; // dist[i][j+1] = dist[i][j];
				pMemAmplitude[(y+1)*width+x] = pMemAmplitude[y*width+x]; // amp[i][j+1] = amp[i][j];
		    }else{
				pMemDistance[y*width+x] = pMemDistance[(y+1)*width+x]; // dist[i][j] = dist[i][j+1];
				pMemAmplitude[y*width+x] = pMemAmplitude[(y+1)*width+x]; // amp[i][j] = amp[i][j+1];
		    }
		}
	}
}

void Frame::getComparedHDRDistanceImage(uint16_t *pMemDistance)
{
    int highAmpThreshold = 12500;

	for(int x = 0 ; x < width ; x++){
		for( int y = 0; y < height; y+=2) {

		    int amp1 = pMemDistance[y*width+x]; //amp[i][j]
		    int amp2 = pMemDistance[(y+1)*width+x]; //amp[i][j+1];
		    int ampCase = 1;
			
		    if(amp1 < highAmpThreshold && amp2 < highAmpThreshold){ // both amplitudes in range
		        if(amp2 > amp1) ampCase = 2;
		        else            ampCase = 1;
		    }else if(amp1 >= highAmpThreshold && amp2 <= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 2;
		    }else if(amp1 <= highAmpThreshold && amp2 >= highAmpThreshold){ //one amplitude in range or both amplitudes LOW_AMPLITUDE
		        ampCase = 1;
		    }else{ //both amplitudes out of range
		        if(amp2 < amp1) ampCase = 1;
		        else            ampCase = 2;
		    }

//		    if(/*ampCase == 1 &&*/ y >= 120){
			if(ampCase == 1 ){
				pMemDistance[(y+1)*width+x] = pMemDistance[y*width+x]; // dist[i][j+1] = dist[i][j];
		    }else{
				pMemDistance[y*width+x] = pMemDistance[(y+1)*width+x]; // dist[i][j] = dist[i][j+1];
		    }
		}
	}
}


void Frame::sortData(const Packet &data, int maxDistance)
{    
    int i;
	int distanceData;
	
    if(dataType == Frame::AMPLITUDE){ //distance - amplitude

        int distanceSize = width * height * px_size;		
		uint16_t *pMemDistance = (uint16_t *)&data[0];
		uint16_t *pMemAmplitude = (uint16_t *)&data[distanceSize];

		if( hdrMode == 1 ){
			getComparedHDRAmplitudeImage(pMemAmplitude, pMemDistance);
		}

        for(i = 0; i < distanceSize; i+=2){

            distanceData = (data[i+1] << 8) + data[i];

			if( distanceData < Frame::PIXEL_VALID_DATA ){
				distanceData = (maxDistance * distanceData / MAX_PHASE);
			}

            distData[i]   = distanceData & 0xFF;
            distData[i+1] = (distanceData>>8) & 0xFF;

            amplData[i]   = data[i+distanceSize];
            amplData[i+1] = data[i+distanceSize+1];				
        }

    }
	else if(dataType == Frame::DISTANCE){ //distance

        int sz = width * height * px_size;

		if( hdrMode == 1 ){
			getComparedHDRDistanceImage((uint16_t *)&data[0]);
		}

		for(i = 0; i < sz; i+=2){

            distanceData = (data[i+1] << 8) + data[i];
			if( distanceData < Frame::PIXEL_VALID_DATA ){
				distanceData = (maxDistance * distanceData / MAX_PHASE);
			}

            distData[i]   = distanceData & 0xFF;
            distData[i+1] = (distanceData>>8) & 0xFF;
        }

    }
	else if(dataType == Frame::GRAYSCALE){ //grayscale
        int sz = width * height * px_size;
        for(i = 0; i < sz; i+=2){
            //if(amplData[i+1] > 61) { continue; }
            amplData[i]    = data[i];
            amplData[i+1]  = data[i+1];
        }
	}else{ //DCS

        int sz = width * height * px_size * 4;
        for(i = 0; i < sz; i+=2){
            //if(dcsData[i+1] > 61) { continue; }
            dcsData[i]    = data[i];
            dcsData[i+1]  = data[i+1];
        }
    }

}


} //end namespace nanosys

