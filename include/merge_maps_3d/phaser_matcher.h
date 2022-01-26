#ifndef MERGE_MAPS_3D_PHASER_MERGER_H_
#define MERGE_MAPS_3D_PHASER_MERGER_H_

#include "merge_maps_3d/cloud_merger.h"
#include <phaser/controller/cloud-controller.h>

namespace merge_maps_3d
{
class PhaserMatcher final : public CloudMerger
{
public:
  PhaserMatcher();
  ~PhaserMatcher() override;

  std::optional<std::pair<Eigen::Matrix4f, Cloud::Ptr>>
  match(Cloud::Ptr orig, Cloud::Ptr addition, std::optional<Eigen::Matrix4f> initial_guess = std::nullopt) override;

private:
  std::shared_ptr<phaser_core::CloudController> controller_;
};
} // namespace merge_maps_3d

#endif
