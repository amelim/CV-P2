/*
 * @file testGravityFactor
 * @author Andrew Melim
 */

#include <CppUnitLite/TestHarness.h>
#include <Asteroid/GravityFactor.h>

using namespace gtsam;
using namespace std;
using namespace ast;

static double fov = 60;
static size_t w=640,h=480;
static Cal3_S2::shared_ptr K(new Cal3_S2(fov,w,h));

static SharedNoiseModel model(noiseModel::Unit::Create(2));


Test(GravityFactor, GravityEst){


}
/* *********************** */
int main() { TestResult tr; return TestRegistry::runAllTests(tr);}
/* *********************** */
