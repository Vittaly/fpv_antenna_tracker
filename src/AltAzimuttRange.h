/*
Based on python solution by https://github.com/sq3tle/altazrange
calculated distance is in meters and THIS IS NOT distance around the curvature of Earth
(it's like straight line between points)
latitude : -90 to 90 [deg]
longitude : -180 to 180 [deg]
height : is meters above sea level (can be lower than zero) [m] */

#include <arduino.h>

// typedef  FPD_TYPE double;
#define FPD_TYPE double

struct GpsData
{
    FPD_TYPE lat; // latitude
    FPD_TYPE lon; // longitude
    FPD_TYPE alt; // altitude
};

struct AzimuthInfo
{
    FPD_TYPE az;   // azimuth
    FPD_TYPE ele;  // elevation
    FPD_TYPE dist; // distance
};

struct Point
{
    FPD_TYPE x{0};
    FPD_TYPE y{0};
    FPD_TYPE z{0};
};

struct PointLocation : Point
{
    FPD_TYPE radius;
    FPD_TYPE nx;
    FPD_TYPE ny;
    FPD_TYPE nz;
};

class AltAzimuthRange
{
     FPD_TYPE calcDistance(Point ap, Point bp);
     FPD_TYPE calcGeocentricLatitude(FPD_TYPE lat);
     FPD_TYPE calcEarthRadiusInMeters(FPD_TYPE latitudeRadians);
     PointLocation calcLocationToPoint(GpsData c);
     Point normalizeVectorDiff(Point a, Point b);
     Point rotateGlobe(GpsData b, GpsData a);
     GpsData observer;

public:
     AzimuthInfo calculate(const GpsData& a, const GpsData& b);
    AzimuthInfo calculate(const GpsData& b);

    void setObserverLocation(const GpsData& a);

    explicit AltAzimuthRange(const GpsData& pObserver);
    AltAzimuthRange();
};