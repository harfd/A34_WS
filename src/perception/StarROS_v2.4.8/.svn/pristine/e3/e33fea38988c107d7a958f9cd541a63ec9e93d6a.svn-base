/**
 * @author   lucb
 * @date     2020/1/18
 */

#ifndef __STAR_SDK_PACKETS_H
#define __STAR_SDK_PACKETS_H

#include <vector>
#include <sdk-decoder/Shot.h>

namespace ss {

struct Packet {
    using Points = std::vector<LasShot_S>; //todo 使用object_pool和list避免内存拷贝
    using Synchrons = std::vector<PosPPS_S>;
    // using GPSPackets = std::vector<scd::GPS>;
    // using IMUPackets = std::vector<scd::IMU>;

    Points points;
    Synchrons fast_synchrons;
    Synchrons slow_synchrons;
};

}
#endif //__STAR_SDK_PACKETS_H
