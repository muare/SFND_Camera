#include <iostream>
#include <numeric>
#include <opencv2/core.hpp>


#include "dataStructures.h"
#include "structIO.hpp"

using namespace std;

void computeTTCLidar(std::vector<LidarPoint> &lidarPointsPrev,
                     std::vector<LidarPoint> &lidarPointsCurr, double &TTC)
{
    // auxiliary variables
    double dT = 0.1;        // time between two measurements in seconds
    double laneWidth = 4.0; // assumed width of the ego lane

    // find closest distance to Lidar points within ego lane
    double minXPrev = 1e9, minXCurr = 1e9;
    double minYPrev = 1e9, minYCurr = 1e9;
    double maxYPrev = -1e9, maxYCurr = -1e9;
    for (auto it = lidarPointsPrev.begin(); it != lidarPointsPrev.end(); ++it)
    {
        if (abs(it->y) < laneWidth/2.)
        {
            minXPrev = minXPrev > it->x ? it->x : minXPrev;
        }
        // minYPrev = minYPrev > it->y ? it->y : minXPrev;
        // maxYPrev = maxYPrev < it->y ? it->y : minXPrev;
    }

    for (auto it = lidarPointsCurr.begin(); it != lidarPointsCurr.end(); ++it)
    {
        if (abs(it->y) < laneWidth/2.)
        {
            minXCurr = minXCurr > it->x ? it->x : minXCurr;            
        }
        // minYCurr = minYCurr > it->y ? it->y : minXCurr;
        // maxYCurr = maxYCurr < it->y ? it->y : minXCurr;
    }
    // cout << "minYPrev = " << minYPrev << "minYCurr = " << minYCurr <<endl;
    // cout << "maxYPrev = " << maxYPrev << "maxYCurr = " << maxYCurr <<endl;
    
    // compute TTC from both measurements
    TTC = minXCurr * dT / (minXPrev - minXCurr);
}

int main()
{

    std::vector<LidarPoint> currLidarPts, prevLidarPts;
    readLidarPts("../dat/C22A5_currLidarPts.dat", currLidarPts);
    readLidarPts("../dat/C22A5_prevLidarPts.dat", prevLidarPts);


    double ttc;
    computeTTCLidar(prevLidarPts, currLidarPts, ttc);
    cout << "ttc = " << ttc << "s" << endl;
}