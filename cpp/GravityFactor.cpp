#include "GravityFactor.h"


using namespace gtsam;

namespace ast {
  
  GravityFactor::GravityFactor(Point2& measured, SharedNoiseModel& model,
    Key sCofKey, Key cCofKey, Key pointKey, boost::shared_ptr<gtsam::Cal3_S2>& K):
    Base(model, sCofKey, cCofKey, pointKey), measured_(measured), K_(K){
  }

  void GravityFactor::print(const std::string& s = "") const {
    std::cout << s << "Gravity factor" << std::endl;
  }
}
