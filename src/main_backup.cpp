#include <iostream>

#include <pcl/io/pcd_io.h>
#include <synthesizers/virtual_scan_partial_input.h>

int main(int argc, char** argv)
{
  pcl::PointCloud<pcl::PointXYZ>::Ptr target_map_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr virtual_scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);

  // if (pcl::io::loadPCDFile("/home/autoware/shared_dir/kitti/segmap/segmap.pcd", *target_map_cloud) == -1)
  if (pcl::io::loadPCDFile("/home/autoware/shared_dir/kitti/kitti_og_0.1.pcd", *target_map_cloud) == -1)
  {
    std::cout << "Failed loading target map file." << std::endl;
    exit(1);
  }

  pcl::VirtualScanPartialInput<pcl::PointXYZ> tmp;
  // tmp.setInputCloud(target_map_cloud);
  tmp.addPartialInputCloud(target_map_cloud);
  pcl::VirtualScanPartialInput<pcl::PointXYZ>::VehicleConfig vc;
  vc.velocity = 108;
  tmp.setVehicleConfig(vc);
  tmp.synthesize(*virtual_scan_cloud);
  std::vector<int> colli = tmp.getCollisionCountsPerInput();

  std::cout << colli.size() << std::endl;
  for (int i = 0; i < colli.size(); i++)
  {
    std::cout << "colli: " << colli[i] << std::endl;
  }
  // tmp.synthesize(*virtual_scan_cloud);
  // rt.compute(*virtual_scan_cloud);
  
  std::cout << "Finished generating virtual scan." << std::endl;

  if (pcl::io::savePCDFile("/home/autoware/shared_dir/kitti/segmap/108km.pcd", *virtual_scan_cloud) == -1)
  {
    std::cout << "Failed saving virtual scan file." << std::endl;
    exit(1);
  }

  return 0;
}