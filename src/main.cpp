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

  std::vector<Eigen::Matrix4f> pose_mat;
  std::ifstream in("/home/autoware/shared_dir/kitti/dataset/poses/10_keep.txt");
	if (!in)
	{
		std::cout << "Cannot open file.\n";
	}
	for (int i = 0; i < 99999; i++)
	{
    double temp;
    Eigen::Matrix4f tmp_mat;
    in >> tmp_mat(0, 0);
    in >> tmp_mat(0, 1);
    in >> tmp_mat(0, 2);
    in >> tmp_mat(0, 3);

    in >> tmp_mat(1, 0);
    in >> tmp_mat(1, 1);
    in >> tmp_mat(1, 2);
    in >> tmp_mat(1, 3);
    
    in >> tmp_mat(2, 0);
    in >> tmp_mat(2, 1);
    in >> tmp_mat(2, 2);
    in >> tmp_mat(2, 3);

    in >> tmp_mat(3, 0);
    in >> tmp_mat(3, 1);
    in >> tmp_mat(3, 2);
    in >> tmp_mat(3, 3);

    pose_mat.push_back(tmp_mat);
    // std::cout << "\n" << tmp_mat << std::endl;
    if (in.eof())
			break;
    
	}
	in.close();
  pose_mat.pop_back();

  int prev_idx = 500;
  int next_idx = 503;

  double yaw = atan2(pose_mat[next_idx](1, 3) - pose_mat[prev_idx](1, 3), pose_mat[next_idx](0, 3) - pose_mat[prev_idx](0, 3));
  std::cout << "yaw: " << yaw << ", " << yaw * (180.0 / M_PI) << std::endl;

  pcl::VirtualScanPartialInput<pcl::PointXYZ> tmp;
  // tmp.setInputCloud(target_map_cloud);
  tmp.addPartialInputCloud(target_map_cloud);
  pcl::VirtualScanPartialInput<pcl::PointXYZ>::VehicleConfig vc;
  
  // velocity (km/h)
  vc.velocity = 0;
  
  // position x, y, z
  vc.position(0) = pose_mat[prev_idx](0, 3);
  vc.position(1) = pose_mat[prev_idx](1, 3);
  vc.position(2) = pose_mat[prev_idx](2, 3);

  // orientation roll, pitch, yaw
  vc.orientation = Eigen::Vector3d(0.0, 0.0, yaw);
  
  // set vehicle configuration
  tmp.setVehicleConfig(vc);

  // synthesize virtual scan
  tmp.synthesize(*virtual_scan_cloud);

  std::vector<int> colli = tmp.getFinalCollisionCountsPerInput();

  std::cout << colli.size() << std::endl;
  for (int i = 0; i < colli.size(); i++)
  {
    std::cout << "colli: " << colli[i] << std::endl;
  }
  // tmp.synthesize(*virtual_scan_cloud);
  // rt.compute(*virtual_scan_cloud);
  
  std::cout << "Finished generating virtual scan." << std::endl;

  if (pcl::io::savePCDFile("/home/autoware/shared_dir/kitti/segmap/nocorrect_0km_500.pcd", *virtual_scan_cloud) == -1)
  {
    std::cout << "Failed saving virtual scan file." << std::endl;
    exit(1);
  }

  return 0;
}