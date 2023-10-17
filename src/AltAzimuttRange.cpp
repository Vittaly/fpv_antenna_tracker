#include "AltAzimuttRange.h"
#include <math.h>


FPD_TYPE AltAzimuthRange::calcDistance(Point ap, Point bp)
{
    return sqrt(pow(ap.x - bp.x, 2) + pow(ap.y - bp.y, 2) + pow(ap.z - bp.z, 2));
}

FPD_TYPE AltAzimuthRange::calcGeocentricLatitude(FPD_TYPE latRad)
{

    return atan((1.0 - 0.00669437999014) * tan(latRad));
}

FPD_TYPE AltAzimuthRange::calcEarthRadiusInMeters(FPD_TYPE latitudeRadians)
{
    FPD_TYPE a = 6378137.0;
    FPD_TYPE b = 6356752.3;
    FPD_TYPE cosLat = cos(latitudeRadians);
    FPD_TYPE sinLat = sin(latitudeRadians);
    FPD_TYPE t1 = a * a * cosLat;
    FPD_TYPE t2 = b * b * sinLat;
    FPD_TYPE t3 = a * cosLat;
    FPD_TYPE t4 = b * sinLat;

    return sqrt((t1 * t1 + t2 * t2) / (t3 * t3 + t4 * t4));
}

PointLocation AltAzimuthRange::calcLocationToPoint(GpsData c)
{
    FPD_TYPE latRad = radians(c.lat);
    FPD_TYPE lonRad = radians(c.lon);

    FPD_TYPE radius = calcEarthRadiusInMeters(latRad);
    FPD_TYPE clat = calcGeocentricLatitude(latRad);

    FPD_TYPE cosLon = cos(lonRad);
    FPD_TYPE sinLon = sin(lonRad);
    FPD_TYPE cosLat = cos(clat);
    FPD_TYPE sinLat = sin(clat);
    FPD_TYPE cosGlat = cos(latRad);
    FPD_TYPE sinGlat = sin(latRad);

    PointLocation res;
    res.radius = radius;

    res.x = radius * cosLon * cosLat;
    res.y = radius * sinLon * cosLat;
    res.z = radius * sinLat;

    res.nx = cosGlat * cosLon;
    res.ny = cosGlat * sinLon;
    res.nz = sinGlat;

    res.x += c.alt * res.nx;
    res.y += c.alt * res.ny;
    res.z += c.alt * res.nz;

    return res;
}

Point AltAzimuthRange::normalizeVectorDiff(Point a, Point b)
{
    FPD_TYPE dx = b.x - a.x;
    FPD_TYPE dy = b.y - a.y;
    FPD_TYPE dz = b.z - a.z;
    FPD_TYPE dist;


    dist = sqrt(dx * dx + dy * dy + dz * dz);

    if (dist == 0)
        return {0, 0, 0}; // 0,0,0

    return {dx / dist, dy / dist, dz / dist};
}

Point AltAzimuthRange::rotateGlobe(GpsData b, GpsData a)
{
    Point res;
    GpsData br = {b.lat, b.lon - a.lon, b.alt};
    PointLocation brp = calcLocationToPoint(br);

    FPD_TYPE alat = calcGeocentricLatitude(radians(-a.lat));

    FPD_TYPE alat_cos = cos(alat);
    FPD_TYPE alat_sin = sin(alat);

    res.x = brp.x * alat_cos - brp.z * alat_sin;
    res.y = brp.y;
    res.z = brp.x * alat_sin + brp.z * alat_cos;

    return res;
}

/// @brief Calculate azimuth, elevation and distance from observer to target
/// @param a gps position of observer
/// @param b gps position of target
/// @return
AzimuthInfo AltAzimuthRange::calculate(const GpsData& a, const GpsData& b)
{
    PointLocation ap, bp;
    AzimuthInfo res = {0, 0, 0};
    Point br;
    Point br2;

    ap = calcLocationToPoint(a);
    bp = calcLocationToPoint(b);
    br = rotateGlobe(b, a);

    res.dist = calcDistance(ap, bp);

    if ((br.z * br.z + br.y * br.y) < 1.0e-6)
        return res; // return zeroes
    FPD_TYPE theta = degrees(atan2(br.z, br.y));
    SerialUSB.println("theta:");
    SerialUSB.println(theta);
    res.az = 90 - theta;
    if (res.az < 0)
        res.az += 360;
    if (res.az > 360)
        res.az -= 360;

    br2 = normalizeVectorDiff(ap, bp);

    SerialUSB.println("-----");
    SerialUSB.println("ap:");
    SerialUSB.println(ap.x);
    SerialUSB.println(ap.y);
    SerialUSB.println(ap.z);

    SerialUSB.println("-----");
    SerialUSB.println("bp:");
    SerialUSB.println(bp.x);
    SerialUSB.println(bp.y);
    SerialUSB.println(bp.z);

    SerialUSB.println("-----");
    SerialUSB.println("br:");
    SerialUSB.println(br2.x);
    SerialUSB.println(br2.y);
    SerialUSB.println(br2.z);

    if (br2.x != 0 || br2.y != 0 || br2.z != 0)
    {

        res.ele = 90.0 - degrees(acos(br2.x * ap.nx + br2.y * ap.ny + br2.z * ap.nz));
    }

    return res;
}

/// @brief Calculate azimuth, elevation and distance from default observer to target on b position
/// @param b  gps data of target position
/// @return
AzimuthInfo AltAzimuthRange::calculate(const GpsData& b)
{
    SerialUSB.println("calculate.observer:");
    SerialUSB.println(observer.lat);
    SerialUSB.println(observer.lon);
    SerialUSB.println(observer.alt);

    return calculate(observer, b);
}

void AltAzimuthRange::setObserverLocation(const GpsData& a)
{
    observer = a;
}

AltAzimuthRange::AltAzimuthRange(const GpsData& pObserver): observer(pObserver)
{
   // observer = pObserver;
}
AltAzimuthRange::AltAzimuthRange(): observer ((GpsData){90, 0, 0}) // default observer position on North Pole
{
    
}