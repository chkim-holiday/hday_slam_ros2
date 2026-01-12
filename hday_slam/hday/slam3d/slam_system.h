#ifndef HDAY_SLAM3D_SLAM_SYSTEM_H_
#define HDAY_SLAM3D_SLAM_SYSTEM_H_

namespace hday {
namespace slam3d {

struct Parameters {};

class SlamSystem {
 public:
  explicit SlamSystem(const Parameters& params);

 private:
  Parameters params_;
};

}  // namespace slam3d
}  // namespace hday

#endif  // HDAY_SLAM3D_SLAM_SYSTEM_H_
