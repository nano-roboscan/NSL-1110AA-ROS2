#include "frame.hpp"
#include <stdint.h>
#include <iostream>
#include "rclcpp/rclcpp.hpp"

#define MAX_PHASE  					30000

namespace nanosys {

Frame::Frame(uint16_t dataType_, uint64_t frame_id_, uint16_t width_, uint16_t height_) :
frame_id(frame_id_),
dataType(dataType_),
width(width_),
height(height_),
px_size(sizeof(uint16_t)),
distData(std::vector<uint8_t>(width * height * px_size)), //16 bit
amplData(std::vector<uint8_t>(width * height * px_size)), //16 bit
dcsData(std::vector<uint8_t> (width * height * px_size * 4)) //16 bit 4 dcs
{    
	
}

void Frame::sortData(const Packet &data, int maxDistance)
{    
    int i;
	int distanceData;
	
    if(dataType == Frame::AMPLITUDE){ //distance - amplitude

        int sz = width * height * px_size;

        for(i = 0; i < sz; i+=2){

            distanceData = (data[i+1] << 8) + data[i];

			if( distanceData < Frame::PIXEL_VALID_DATA ){
				distanceData = (maxDistance * distanceData / MAX_PHASE);
			}

            distData[i]   = distanceData & 0xFF;
            distData[i+1] = (distanceData>>8) & 0xFF;

            amplData[i]   = data[i+sz];
            amplData[i+1] = data[i+sz+1];				
        }

    }
	else if(dataType == Frame::DISTANCE){ //distance

        int sz = width * height * px_size;
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

