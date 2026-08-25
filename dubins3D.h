#ifndef DUBINS_H
#define DUBINS_H
#include <sys/types.h>
#include <dirent.h>
#include <fstream>
#include <string>
#include <sstream>
#include <iostream>
#include <vector>
#include <cmath>
#include "ompl/base/State.h"
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/DubinsStateSpace.h>
#include <ompl/base/SpaceInformation.h>
#include <map>
#include <cstring>
#include <unsupported/Eigen/Polynomials>
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/util/Console.h"
using namespace std;
namespace ob = ompl::base;
struct DubinsStruct {
    double t, p, q, length;
    std::string path_type;
};

struct DubinsState3D {
    double x, y, z, theta, pitch;
    DubinsState3D(double x_,double y_, double z_, double theta_, double pitch_) : x(x_), y(y_), z(z_), theta (theta_), pitch (pitch_) {}
};


class DubinsManeuver2D {
public:
    
    DubinsManeuver2D(const std::vector<double>& qi, const std::vector<double>& qf, double rhomin, double minLength = std::numeric_limits<double>::infinity(), bool disable_CCC = false) 
        : qi(qi), qf(qf), rhomin(rhomin) {
        
         double dx = qf[0] - qi[0];
         double dy = qf[1] - qi[1];
         double D = std::sqrt(dx * dx + dy * dy);
         double d = D / rhomin;
        
             // Normalize the angles
         double rotationAngle = mod2piGL(std::atan2(dy, dx));
         double a = mod2piGL(qi[2] - rotationAngle);
         double b = mod2piGL(qf[2] - rotationAngle);
         
         double sa = std::sin(a), ca = std::cos(a);
         double sb = std::sin(b), cb = std::cos(b);
         //cerr << " d: " <<d << ", a: " << a << ", b: " << b << " " << sa << " " << sb << " " << ca << " " << cb << endl; 
         DubinsStruct pathLSL = _LSL_GL(a, b, d, sa, ca, sb, cb);
         DubinsStruct pathRSR = _RSR_GL(a, b, d, sa, ca, sb, cb);
         DubinsStruct pathLSR = _LSR_GL(a, b, d, sa, ca, sb, cb);
         DubinsStruct pathRSL = _RSL_GL(a, b, d, sa, ca, sb, cb);
         DubinsStruct pathRLR = _RLR_GL(a, b, d, sa, ca, sb, cb);
         DubinsStruct pathLRL = _LRL_GL(a, b, d, sa, ca, sb, cb);
         
         std::vector<DubinsStruct> paths = {pathLSL, pathRSR, pathLSR, pathRSL};

         if (!disable_CCC) 
         { 

             paths.push_back(pathRLR);
             paths.push_back(pathLRL);
         }
         
       //cerr << pathLRL.length <<"  " <<pathLSL.length << " " << pathLSR.length << "  "<< pathRLR.length << " " << pathRSL.length <<" "  << pathRSR.length   << endl;   
             // Special case when distance is very small
        if (std::abs(d) < 1e-5 && std::abs(a) < 1e-5 && std::abs(b) < 1e-5) {
             double dist2D = std::max(std::abs(qi[0] - qf[0]), std::abs(qi[1] - qf[1]));
             if (dist2D < 1e-5) {
              DubinsStruct pathC = _C_GL();
              paths = {pathC};
             }
        }
            // Sort paths by length and choose the shortest        
        sort(paths.begin(), paths.end(), [](const DubinsStruct& a, const DubinsStruct& b) { return a.length < b.length; });
        maneuver = paths[0];

        
    }
    DubinsStruct maneuver;
    std::vector<double> getCoordinatesAtGL(double offset) const;
    
    static double mod2piGL(double angle);
    
private:
    std::vector<double> qi;
    std::vector<double> qf;
    double rhomin;

    std::vector<double> getPositionInSegmentGL(double offset, const std::vector<double>& qi, char segmentType) const;
    DubinsStruct _LSL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _RSR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _LSR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _RSL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _RLR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _LRL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const;
    DubinsStruct _C_GL() const;
    
};




class DubinsManeuver3D {
public:
    DubinsManeuver3D(const DubinsState3D& start, const DubinsState3D& end, double rhomin, double pitchMin, double pitchMax);
    std::vector<DubinsManeuver2D> tryToConstructGL(double horizontalRadius);
    void planPathGL();
    double length;
    std::vector<DubinsManeuver2D> path;

private:
    DubinsState3D qi;
    DubinsState3D qf;
    double rhomin;
    double pitchMin;
    double pitchMax;
};

inline DubinsManeuver3D maneuverI_(DubinsState3D(0, 0, 0, 0, 0), DubinsState3D(0, 0, 0, 0, 0), 40, 0, 0);


inline double DubinsManeuver2D::mod2piGL(double angle) 
{
    double result = std::fmod(angle, M_PI + M_PI);
    if (result < 0) result += M_PI + M_PI;
    return result;
}
inline DubinsStruct DubinsManeuver2D::_C_GL() const {
    return {0.0, 2 * M_PI, 0.0, 2 * M_PI * rhomin, "RRR"};
}

inline DubinsStruct DubinsManeuver2D::_LSL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux = std::atan2(cb - ca, d + sa - sb);
    double t = mod2piGL(-a + aux);
    double p = std::sqrt(2 + d * d - 2 * std::cos(a - b) + 2 * d * (sa - sb));
    double q = mod2piGL(b - aux);
    //cerr << t << " " << p << " " << q << " " << rhomin << " " << -a << " " << aux << endl;
    return {t, p, q, (t + p + q) * rhomin, "LSL"};
}

inline DubinsStruct DubinsManeuver2D::_RSR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux = std::atan2(ca - cb, d - sa + sb);
    double t = mod2piGL(a - aux);
    double p = std::sqrt(2 + d * d - 2 * std::cos(a - b) + 2 * d * (sb - sa));
    double q = mod2piGL(-b + aux);
    return {t, p, q, (t + p + q) * rhomin, "RSR"};
}

inline DubinsStruct DubinsManeuver2D::_LSR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux1 = -2 + d * d + 2 * std::cos(a - b) + 2 * d * (sa + sb);
    if (aux1 > 0) {
        double p = std::sqrt(aux1);
        double aux2 = std::atan2(-ca - cb, d + sa + sb) - std::atan2(-2.0, p);
        double t = mod2piGL(-a + aux2);
        double q = mod2piGL(-b + aux2);
        return {t, p, q, (t + p + q) * rhomin, "LSR"};
    } else {
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), "LSR"};
    }
}

inline DubinsStruct DubinsManeuver2D::_RSL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux1 = d * d - 2 + 2 * std::cos(a - b) - 2 * d * (sa + sb);
    if (aux1 > 0) {
        double p = std::sqrt(aux1);
        double aux2 = std::atan2(ca + cb, d - sa - sb) - std::atan2(2.0, p);
        double t = mod2piGL(a - aux2);
        double q = mod2piGL(b - aux2);
        return {t, p, q, (t + p + q) * rhomin, "RSL"};
    } else {
        return {std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), "RSL"};
    }
}

inline DubinsStruct DubinsManeuver2D::_RLR_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux = (6 - d * d + 2 * cos(a - b) + 2 * d * (sa - sb)) / 8;
    double t, p, q;
    if (std::fabs(aux) <= 1) {
        p = mod2piGL(-acos(aux));
        t = mod2piGL(a - atan2(ca - cb, d - sa + sb) + p / 2);
        q = mod2piGL(a - b - t + p);
    } else {
        t = p = q = std::numeric_limits<double>::infinity();
    }
    double length = (t + p + q) * rhomin;
    return {t, p, q, length, "RLR"};
}

inline DubinsStruct DubinsManeuver2D::_LRL_GL(double a, double b, double d, double sa, double ca, double sb, double cb) const {
    double aux = (6 - d * d + 2 * cos(a - b) + 2 * d * (-sa + sb)) / 8;
    double t, p, q;
    if (std::fabs(aux) <= 1) {
        p = mod2piGL(-acos(aux));
        t = mod2piGL(-a + atan2(-ca + cb, d + sa - sb) + p / 2);
        q = mod2piGL(b - a - t + p);
    } else {
        t = p = q = std::numeric_limits<double>::infinity();
    }
    double length = (t + p + q) * rhomin;
    return {t, p, q, length, "LRL"};
}



inline std::vector<double> DubinsManeuver2D::getCoordinatesAtGL(double offset) const {
    double normalizedOffset = offset / rhomin;
    std::vector<double> qi = {0.0, 0.0, this->qi[2]};

    double l1 = maneuver.t;
    double l2 = maneuver.p;
    std::vector<double> q1 = getPositionInSegmentGL(l1, qi, maneuver.path_type[0]);
    std::vector<double> q2 = getPositionInSegmentGL(l2, q1, maneuver.path_type[1]);

    std::vector<double> q;
    if (normalizedOffset < l1) {
        q = getPositionInSegmentGL(normalizedOffset, qi, maneuver.path_type[0]);
    } else if (normalizedOffset < l1 + l2) {
        q = getPositionInSegmentGL(normalizedOffset - l1, q1, maneuver.path_type[1]);
    } else {
        q = getPositionInSegmentGL(normalizedOffset - l1 - l2, q2, maneuver.path_type[2]);
    }

    q[0] = q[0] * rhomin + this->qi[0];
    q[1] = q[1] * rhomin + this->qi[1];
    q[2] = mod2piGL(q[2]);
    return q;
}

inline std::vector<double> DubinsManeuver2D::getPositionInSegmentGL(double offset, const std::vector<double>& qi, char segmentType) const {
    std::vector<double> q(3, 0.0);
    if (segmentType == 'L') {
        q[0] = qi[0] + std::sin(qi[2] + offset) - std::sin(qi[2]);
        q[1] = qi[1] - std::cos(qi[2] + offset) + std::cos(qi[2]);
        q[2] = qi[2] + offset;
    } else if (segmentType == 'R') {
        q[0] = qi[0] - std::sin(qi[2] - offset) + std::sin(qi[2]);
        q[1] = qi[1] + std::cos(qi[2] - offset) - std::cos(qi[2]);
        q[2] = qi[2] - offset;
    } else if (segmentType == 'S') {
        q[0] = qi[0] + std::cos(qi[2]) * offset;
        q[1] = qi[1] + std::sin(qi[2]) * offset;
        q[2] = qi[2];
    }
    return q;
}



inline DubinsManeuver3D::DubinsManeuver3D(const DubinsState3D& start, const DubinsState3D& end, double rhomin, double pitchMin, double pitchMax)
    : qi(start), qf(end), rhomin(rhomin), pitchMin(pitchMin), pitchMax(pitchMax), length(std::numeric_limits<double>::infinity()) {}

inline std::vector<DubinsManeuver2D> DubinsManeuver3D::tryToConstructGL(double horizontalRadius) {
    std::vector<double> qi2D = {qi.x, qi.y, qi.theta};
    std::vector<double> qf2D = {qf.x, qf.y, qf.theta};
    DubinsManeuver2D Dlat(qi2D, qf2D, horizontalRadius);
    double horizontalLength = Dlat.maneuver.length;
    std::vector<double> qi3D = {0.0, qi.z, qi.pitch};
    std::vector<double> qf3D = {horizontalLength, qf.z, qf.pitch};
    
    double verticalCurvature = std::sqrt((1.0 / (rhomin * rhomin)) - (1.0 / (horizontalRadius * horizontalRadius)));
    //cerr  << rhomin << " " <<  horizontalRadius << " " << verticalCurvature << " " << horizontalLength << endl;
    if (verticalCurvature < 1e-5) return {};
    
    double verticalRadius = 1.0 / verticalCurvature;
    DubinsManeuver2D Dlon(qi3D, qf3D, verticalRadius);
    
    //cerr << "------>" << endl;
    if (Dlon.maneuver.path_type == "RLR") {
        return {};
    }

    if (Dlon.maneuver.path_type[0] == 'R' && (qi.pitch - Dlon.maneuver.t < pitchMin)) {
        return {};
    }

    if (Dlon.maneuver.path_type[0] == 'L' && (qi.pitch + Dlon.maneuver.t > pitchMax)) {
        return {};
    } 
    //cerr << " " << Dlat.maneuver.length  << "  " << Dlon.maneuver.length  << endl;
    return {Dlat, Dlon};
}

inline void DubinsManeuver3D::planPathGL() {
    double zi = qi.z;
    double zf = qf.z;
    double dz = zf - zi;
    double a = 1.0;
    double b = 1.0;
    auto fa = tryToConstructGL(rhomin * a);
    auto fb = tryToConstructGL(rhomin * b);
    while (fb.size() < 2) {
        b *= 2.0;
        fb = tryToConstructGL(rhomin * b);
    }
    //cerr << fb.size() << endl;
    if (!fa.empty()) {
        path = fa;
    } else if (fb.size() >= 2) {
        path = fb;
    } else {
        throw std::runtime_error("No maneuver exists");
    }
    
    double step = 0.1;
    while (std::abs(step) > 1e-10) {
        double c = b + step;
        if (c < 1.0) c = 1.0;
        auto fc = tryToConstructGL(rhomin * c);
        if (fc.size() > 0) {
            if (fc[1].maneuver.length < fb[1].maneuver.length) 
            {
                b = c;
                fb = fc;
                step *= 2;
                continue;
            }
        }
        step *= -0.1;
    }
    path = fb;
    length = path[1].maneuver.length;
}


inline ompl::base::Cost pathLengthGL(const ompl::base::State *src, const ompl::base::State *sgl)
{
    const ompl::base::RealVectorStateSpace::StateType *st_   = src->as<ob::RealVectorStateSpace::StateType>();
    const ompl::base::RealVectorStateSpace::StateType *gl_   = sgl->as<ob::RealVectorStateSpace::StateType>();
    
    DubinsState3D src_(st_->values[0],st_->values[1],st_->values[2],st_->values[3],st_->values[4]);
    DubinsState3D sgl_(gl_->values[0],gl_->values[1],gl_->values[2],gl_->values[3],gl_->values[4]);
    
    //cerr << "start " << st_->values[0] << " 11 " << st_->values[1] << " "  << st_->values[2] << " " << st_->values[3] << " ---" << st_->values[4] <<endl;
    
    //cerr << "goal " << gl_->values[0] << "  " << gl_->values[1] << " "  << gl_->values[2] << " " << gl_->values[3] << " " << gl_->values[4] <<endl;
    maneuverI_ =  DubinsManeuver3D(DubinsState3D(0, 0, 0, 0, 0), DubinsState3D(0, 0, 0, 0, 0), 0.3, 0, 0);
    double rhomin = 0.6; 
    double pitchMin = (M_PI*-15/180);
    double pitchMax = (M_PI*20/180);
    //cerr << pitchMin << " " << pitchMax << endl;
    maneuverI_ = DubinsManeuver3D(src_, sgl_, rhomin, pitchMin, pitchMax);
    maneuverI_.planPathGL();
    //maneuver_ = maneuver;
    // Return the total path length
    return ompl::base::Cost(maneuverI_.length);
}


inline void interpolateGL(double seg, ompl::base::State *state, bool pathN)
{
     const ob::RealVectorStateSpace::StateType *rstate = static_cast<ob::RealVectorStateSpace::StateType *>(state);  
     
     std::vector<double> qSZ = maneuverI_.path[1].getCoordinatesAtGL(seg);
     std::vector<double> qXY = maneuverI_.path[0].getCoordinatesAtGL(qSZ[0]);
     
     if(pathN)
       cerr << "(" <<qXY[0] << "," << qXY[1] << "," << qSZ[1] << ")," <<endl;
     
     rstate->values[0] = qXY[0];
     rstate->values[1] = qXY[1];
     rstate->values[2] = qSZ[1];
      
     if(pathN)
     {
        rstate->values[3] = qXY[2];
        rstate->values[4] = qSZ[2]; 
     }
}

inline void plotTrajGL(double seg, ompl::base::State *state, bool pathN)
{

     //cerr << "-----------" <<endl;
        for(int i = 0; i < 200; i++)
        {
              double arrive_ =(static_cast<double>(i) / 199.0)*seg;
              //cerr << arrive_ << endl;
              interpolateGL(arrive_,state,true);
        
        }

}


#endif
