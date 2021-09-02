#pragma once

#include "location_point.h"

namespace navigine {
namespace indoor_routing {

constexpr double EPSILON  = 1e-8;

// Get distance from segment to point
double SegmentPointDist(const LocationPoint& segmentPoint1,
                               const LocationPoint& segmentPoint2,
                               const LocationPoint& p);

// Get distance from A to B
double GetDist(const LocationPoint& A, const LocationPoint& B);
double GetDist(double ax, double ay, double bx, double by);

// Get distance from C to line (AB)
double GetDist(const LocationPoint& A,
               const LocationPoint& B,
               const LocationPoint& C);

// Calculate the oriented area of triangle (ABC).
double GetArea(const LocationPoint& A,
               const LocationPoint& B,
               const LocationPoint& C);

// Calculate the oriented area of triangle (ABC).
double GetArea(double ax, double ay,
                      double bx, double by,
                      double cx, double cy);

// Calculate determinant of the 2x2 matrix:
//  a11 a12
//  a21 a22
double Determinant(double a11, double a12,
                   double a21, double a22);

// Calculate determinant of the 3x3 matrix:
//  a11 a12 a13
//  a21 a22 a23
//  a31 a32 a33
double Determinant(double a11, double a12, double a13,
                   double a21, double a22, double a23,
                   double a31, double a32, double a33);

// Get distance from O to segment [AB].
// Px - is the closest point on [AB].
double GetProjection(double ax, double ay,
                     double bx, double by,
                     double ox, double oy,
                     double* px, double* py);

// Get intersection of vector PQ and segment [AB].
double GetIntersection(double px, double py,
                       double qx, double qy,
                       double ax, double ay,
                       double bx, double by,
                       double* x, double* y);

// Check if segments [AB] and [CD] has intersection
bool CheckIntersection(double ax, double ay,
                       double bx, double by,
                       double cx, double cy,
                       double dx, double dy);

bool PointOnLine        (double x, double y, double x1, double y1, double x2, double y2);
bool XRayIntersectsLine (double x, double y, double x1, double y1, double x2, double y2);

} } // namespace navigine::indoor_routing
