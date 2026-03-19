#pragma once
#include "Track/track_base.h"

namespace ns_control{

    class Track_trackdrive : public Track {

    public:
    bool getTraj();
    bool getTrajPath(Trajectory &TrajPath);
    bool calTraj(Trajectory &relativePath);

    };
}
