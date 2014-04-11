#ifndef CNN_REGISTRATION_H
#define CNN_REGISTRATION_H

#include <pcl/registration/registration.h>

namespace pcl
{
    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class CoherentNearestNeighborIteration : Registration<PointSource, PointTarget, Scalar>
    {
        public:
            CoherentNearestNeighborIteration();
            ~CoherentNearestNeighborIteration();

            
    }
}

#endif

