#ifndef __ROBOSCAN_FRAME_H__
#define __ROBOSCAN_FRAME_H__

#include <cstdint>
#include <vector>

namespace nanosys {

    typedef std::vector<uint8_t> Packet;

    struct Frame
    {
        enum DataType { NONTYPE, GRAYSCALE, DISTANCE, AMPLITUDE, DCS};
        enum PixelType { PIXEL_VALID_DATA = 65300, LOW_AMPLITUDE = 65300, ADC_OVERFLOW = 65500, SATURATION = 65400, BAD_PIXEL = 65450};

        uint64_t frame_id;        
        uint16_t dataType;
        uint16_t width;
        uint16_t height;
        uint16_t hdrMode;
        uint32_t px_size;        
        uint8_t stride;        
        std::vector<uint8_t> distData;
        std::vector<uint8_t> amplData;
        std::vector<uint8_t> dcsData;
        std::vector<uint16_t> dist2BData;
		

        Frame(uint16_t, uint64_t, uint16_t, uint16_t, uint16_t);
		void getComparedHDRAmplitudeImage(uint16_t *pMemAmplitude, uint16_t *pMemDistance);
		void getComparedHDRDistanceImage(uint16_t *pMemDistance);
        void sortData(const Packet &data, int maxDistance);
    };

} //end namespace nanosys

#endif // __ROBOSCAN_FRAME_H__
