/**
 * @file GravityFactor
 * @brief Factor for estimating spherical harmonics of a gravitational field
 * @author Andrew Melim
 */

#pragma once

//BOOST
#include <boost/lexical_cast.hpp>
#include <boost/math/special_functions/legendre.hpp>
//GTSAM
#include <gtsam/geometry/concepts.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/LieVector.h>
#include <gtsam/geometry/SimpleCamera.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

#include <math.h>

#define _USE_MATH_DEFINES

namespace ast {

  class GravityFactor : public gtsam::NoiseModelFactor3<gtsam::LieVector, gtsam::LieVector, gtsam::Point3>
  {
    protected:

      gtsam::Point2 measured_;
      gtsam::Point2 LatLon_;
      boost::shared_ptr<gtsam::Cal3_S2> K_;
     
    public:
      typedef gtsam::NoiseModelFactor3<gtsam::LieVector, gtsam::LieVector, gtsam::Point3> Base;
      typedef GravityFactor This;
      typedef boost::shared_ptr<This> shared_ptr;

      virtual ~GravityFactor() {}

      //Default Constructor
      GravityFactor() {
      }

      GravityFactor(gtsam::Point2& measured, gtsam::SharedNoiseModel& model,
        gtsam::Key sCofKey, gtsam::Key cCofKey, gtsam::Key pointKey, 
        boost::shared_ptr<gtsam::Cal3_S2>& K);

      //Make deep copy
      virtual gtsam::NonlinearFactor::shared_ptr clone() const{
        return boost::static_pointer_cast<gtsam::NonlinearFactor>(
            gtsam::NonlinearFactor::shared_ptr(new GravityFactor(*this)));
      }


      void print(const std::string& s) const;
      
      virtual bool equals(const gtsam::NonlinearFactor& p, double tol=1e-9) const {
        const This* e = dynamic_cast<const This*>(&p);
        return e
          && Base::equals(p, tol)
          && this->measured_.equals(e->measured_, tol);
          //&& this->K_.equals(*e->K_, tol);
      }

      gtsam::Vector evaluateError(const gtsam::LieVector& s_cof, const gtsam::LieVector& c_cof, 
          const gtsam::Point3& point,
          boost::optional<gtsam::Matrix&> H1 = boost::none,
          boost::optional<gtsam::Matrix&> H2 = boost::none,
          boost::optional<gtsam::Matrix&> H3 = boost::none) const{
        try{

          gtsam::Pose3 pose;
          gtsam::PinholeCamera<gtsam::Cal3_S2> camera(pose, *K_);
          gtsam::Point2 reprojectionError(camera.project(point) - measured_);
          return reprojectionError.vector();

        } catch( gtsam::CheiralityException& e){
          std::cout << e.what() << std::endl; 
        }

        return gtsam::ones(2) * 2.0 * K_->fx();
      }

      const gtsam::Point2& measured() const {
        return measured_;
      }

      double gravityEst(const gtsam::LieVector& s_cof, const gtsam::LieVector& c_cof, 
          const gtsam::Pose3& pose)
      {
        //Convert Cart to LatLon
       
        double r = sqrt(pow(pose.x(), 2) + pow(pose.y(), 2) + pow(pose.z(),2));

        //Longitude in Rads
        double lon_ = atan2(pose.y(), pose.x());
        //lon_ = lon_*180/M_PI;
        //Latitude in Rads
        double lat_ = atan(pose.z()/sqrt(pow(pose.x(),2) + pow(pose.y(), 2)));
        //lat_ = lat*180/M_PI;
        
        double sLat = sin(lat_);
        int s_dim = s_cof.size();
        //Reference Radius Konopliv 12
        double R = 265000;
        //Mass of Vesta
        double mass =2.589e020;
        //Gravitational Constant
        double g_const = 6.67384e-11;
        double u = 0;

        int k = 1;
        for(int n = 0; n < s_dim; n++)
        {
          //Compute i legendre polynomials 
          for(int m = 0; m < n; m++)
          {
            u += pow(R/r, n)*boost::math::legendre_p(n,m,sLat)*(c_cof[k]*cos(m*lon_) + s_cof[k]*sin(m*lat_));
          }
          //k=(i-1)*(2+i)/2+j+1;
          //u=u+(R/r)^i*P(j+1)*(C(k)*cosd(j*long)+S(k)*sind(j*long));
        }

        return u*g_const*mass/r;
      }
  };//End Class
} //End Namespace
