#include "merge_maps_3d/ndt_matcher.h"
#include "merge_maps_3d/phaser_matcher.h"
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

std::optional<Cloud::Ptr>
readFile(std::string filename)
{
  if (filename.size() <= 4)
  {
    return std::nullopt;
  }
  Cloud::Ptr cloud(new Cloud());
  std::string suffix = filename.substr(filename.size() - 3);
  if (suffix.compare(std::string("ply")) == 0)
  {
    bool read_success = pcl::io::loadPLYFile(filename, *cloud);
    if (!read_success)
    {
      return std::nullopt;
    }
    return cloud;
  }
  if (suffix.compare(std::string("pcd")) == 0)
  {
    bool read_success = pcl::io::loadPCDFile(filename, *cloud);
    if (!read_success)
    {
      return std::nullopt;
    }
    return cloud;
  }
  return std::nullopt;
}

int
main(int argc, char** argv)
{
  if (argc < 3)
  {
    std::cerr << "Need two paths" << std::endl;
    return 1;
  }
  auto test_matcher = merge_maps_3d::PhaserMatcher();
  auto small = readFile(std::string(argv[1]));
  if (!small)
  {
    std::cerr << "Failed to load file " << argv[1] << std::endl;
    return 1;
  }

  auto big = readFile(std::string(argv[2]));
  if (!big)
  {
    std::cerr << "Failed to load file " << argv[2] << std::endl;
    return 1;
  }

  auto results = test_matcher.match(small.value(), big.value());
  if (!results)
  {
    std::cerr << "Failed to match\n";
  }
  else
  {
    pcl::io::savePCDFileBinary("matching_result.pcd", *results.value().second);
  }
  return 0;
}
