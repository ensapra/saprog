#include <cstdlib>
#include <iostream>
#include "saprog/imu_filter.hpp"

using namespace saprog::imu;

static void check(bool condition, const char * message) {
  if (!condition) { std::cerr << message << '\n'; std::exit(1); }
}
static bool same_pose(const Quat & a, const Quat & b, double tolerance=1e-7) {
  double dot=0;
  for (int i=0; i<4; ++i) { dot+=a[i]*b[i]; }
  return std::abs(std::abs(dot)-1) < tolerance;
}

int main() {
  Sample parsed;
  check(parse_sample("123,0,0,1,0,0,0,10,20,30",parsed),"calculated CSV rejected");
  check(parsed.accel[2]==1 && parsed.mag[2]==30,"incorrect CSV field order");
  check(!parse_sample("123,0,0,1,0,0,0,10,20,30,1",parsed),"extra fields accepted");
  check(!parse_sample("123,0,0,nan,0,0,0,10,20,30",parsed),"NaN accepted");
  check(!parse_sample("123,0,0,1junk,0,0,0,10,20,30",parsed),"invalid numeric suffix accepted");
  check(!parse_sample("123,0,0,1,0,0,0,10,20,30,",parsed),"trailing comma accepted");

  for (int axis=0; axis<3; ++axis) {
    Filter f;
    f.update({0,{0,0,1},{0,0,0},{0,0,0}});
    f.zero();
    Vec rate{}; rate[axis]=720;
    for (int i=1; i<=50; ++i) {
      f.update({i*10.0,{0.6,0,0.8},rate,{0,0,0}});
      check(!f.correcting_tilt,"movement acceleration used as gravity");
    }
    check(same_pose(f.attitude,{1,0,0,0}),"full rapid turn did not return to initial pose");
    check(same_pose(f.relative(),{1,0,0,0}) && f.zeroed,"saved zero changed while moving");
  }
  Filter f;
  f.update({0,{0,0,1},{0,0,0},{0,0,0}});
  f.update({10,{0,0,0},{0,0,90},{0,0,0}});
  check(yaw(f.attitude)>0,"zero acceleration incorrectly stopped gyro integration");
  f.zero();
  const auto absolute=f.attitude;
  check(same_pose(f.relative(),{1,0,0,0}),"zero transform is not identity at reference pose");
  check(same_pose(f.attitude,absolute),"zero changed the absolute estimate");
  f.clear_zero();
  check(same_pose(f.relative(),absolute),"clear zero did not restore estimate");
  f.update({5000,{0,0,1},{0,0,90},{0,0,0}});
  check(f.time_gaps==1,"long sensor timestamp gap was not reported");

  Filter calibration;
  calibration.update({0,{0,0,1},{0.3,-0.2,0.5},{0,0,0}});
  calibration.start_calibration();
  for (int i=1; i<=401; ++i) {
    calibration.update({i*10.0,{0,0,1},{0.3,-0.2,0.5},{0,0,0}});
  }
  check(calibration.calibrated && calibration.zeroed && !calibration.calibrating,"stationary calibration failed");
  check(norm(subtract(calibration.bias,{0.3,-0.2,0.5}))<1e-9,"incorrect gyro bias");
  for (int i=402; i<1000; ++i) {
    calibration.update({i*10.0,{0,0,1},{0.3,-0.2,0.5},{0,0,0}});
  }
  check(same_pose(calibration.relative(),{1,0,0,0}),"resting bias still causes drift");
  calibration.start_calibration();
  for (int i=1000; i<=1400; ++i) {
    calibration.update({i*10.0,{0,0,1},{10,0,0},{0,0,0}});
  }
  check(!calibration.calibrating && calibration.calibration_status.find("rejected")!=std::string::npos,"moving calibration accepted");
  check(norm(subtract(calibration.bias,{0.3,-0.2,0.5}))<1e-9,"rejected calibration overwrote bias");
  std::cout << "IMU parsing, rapid rotation, zero, gap and calibration checks passed\n";
}
