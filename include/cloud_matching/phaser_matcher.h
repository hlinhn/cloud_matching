#ifndef CLOUD_MATCHING_PHASER_MERGER_H_
#define CLOUD_MATCHING_PHASER_MERGER_H_

#include "cloud_matching/cloud_merger.h"
#include <phaser/controller/cloud-controller.h>

namespace cloud_matching
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
} // namespace cloud_matching

#endif
