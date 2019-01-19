#ifndef MAPPOINT_H
#define MAPPOINT_H

#include "myslam/common_include.h"

namespace myslam
{

class MapPoint
{
public:
    MapPoint();

    MapPoint( long id, Vector3d position, Vector3d norm );
    
    // Factory function
    static MapPoint::Ptr createMapPoint();

public:
    typedef std::shared_ptr<MapPoint> Ptr;
    unsigned long                     id_;
    Vector3d                          pos_;
    Vector3d                          norm_;  // Normal of viewing direction
    Mat                               descriptor_;
    int                               observed_times_;
    int                               correct_times_;   // Being an inliner in pose estimation
};

}

#endif // MAPPOINT_H