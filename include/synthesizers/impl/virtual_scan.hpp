
#ifndef _VIRTUAL_SCAN_HPP_
#define _VIRTUAL_SCAN_HPP_

#include <synthesizers/virtual_scan.h>

template <typename PointT>
void pcl::VirtualScan<PointT>::applySynthesizer (PointCloud &out_cloud)
{   
  PointCloudPtr ops_map_ptr(new PointCloud);
  typename OctreeT::AlignedPointTVector voxel_centers;
  double min_x, min_y, min_z;
  double max_x, max_y, max_z;

  pcl::octree::OctreePointCloudSearch<PointT> octree(resolution_);
  pcl::octree::OctreePointCloudSearch<PointT> ops_octree(resolution_);

  octree.setInputCloud(input_);
  octree.defineBoundingBox(-500, -500, -500, 500, 500, 500);
  octree.addPointsFromInputCloud();
  octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
  octree.getOccupiedVoxelCenters(voxel_centers);

  for (int p_idx = 0; p_idx < voxel_centers.size(); p_idx++)
  {
    ops_map_ptr->push_back(voxel_centers[p_idx]);
  }

  ops_octree.setInputCloud(ops_map_ptr);
  ops_octree.defineBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
  ops_octree.addPointsFromInputCloud();

  launchVirtualBeams(ops_octree, ops_map_ptr, out_cloud);
}

template <typename PointT>
void pcl::VirtualScan<PointT>::launchVirtualBeams(const pcl::octree::OctreePointCloudSearch<PointT> &ops_octree,
                                                  const PointCloudPtr &ops_map,
                                                  PointCloud &out_cloud)
{
  Eigen::Vector3d v_p(vehicle_config_.position(0), vehicle_config_.position(1), vehicle_config_.position(2));
  Eigen::Vector3d v_a(vehicle_config_.orientation(0), vehicle_config_.orientation(1), vehicle_config_.orientation(2));

  for (int a_idx = 0; a_idx < lidar_config_.num_azimuth; a_idx++)
  {
    // need to correct azimuth angle for each channel
    // why is minus vangle? plus angle???
    float azimuth_angle = 2 * M_PI / lidar_config_.num_azimuth * (a_idx % lidar_config_.num_azimuth) + v_a(2);

    for (int c_idx = 0; c_idx < lidar_config_.num_channels; c_idx++)
    {
      float coeff;
      int num_intersected_voxel;
      std::vector<int> voxel_indices;
      Eigen::Vector3d v_dir, e_p;

      // need to correct altitude angle (channel angle)
      float altitude_angle = (lidar_config_.max_altitude - 2 * lidar_config_.max_altitude / lidar_config_.num_channels * c_idx) * M_PI / 180;

      e_p(0) = v_p(0) + lidar_config_.range * cos(azimuth_angle) * cos(altitude_angle);
      e_p(1) = v_p(1) + lidar_config_.range * sin(azimuth_angle) * cos(altitude_angle);
      e_p(2) = v_p(2) + lidar_config_.range * sin(altitude_angle);
      v_dir = (e_p - v_p).normalized();

      num_intersected_voxel = ops_octree.getIntersectedVoxelIndices(v_p.cast<float>(), v_dir.cast<float>(), voxel_indices);

      if (num_intersected_voxel > 0 && getCoeffCollisionPoint(voxel_indices, v_p, e_p, v_dir, ops_map, coeff))
      {
        // std::cout << "num_intersected_voxel: " << num_intersected_voxel << std::endl;

        pcl::PointXYZ collision_point_pcl;
        Eigen::Vector3d collision_point(v_p + coeff * v_dir);

        collision_point_pcl.x = collision_point(0);
        collision_point_pcl.y = collision_point(1);
        collision_point_pcl.z = collision_point(2);

        out_cloud.push_back(collision_point_pcl);
      }
    }
  }
}

template <typename PointT>
bool pcl::VirtualScan<PointT>::getCoeffCollisionPoint(std::vector<int> voxel_indices,
                                                      const Eigen::Vector3d &vh_position,
                                                      const Eigen::Vector3d &end_position,
                                                      const Eigen::Vector3d &vh_direction,
                                                      const PointCloudPtr &ops_map,
                                                      float &coeff)
{
  for (uint32_t idx = 0; idx < voxel_indices.size(); idx++)
  {
    double distance, random;
    Eigen::Vector3d p;
    int v_idx = voxel_indices[idx];
    if (v_idx == -1)
    {
      continue;
    }

    distance = (ops_map->at(v_idx).getVector3fMap() - vh_position.cast<float>()).norm();
    // distance = std::sqrt(std::pow(ops_map->points[v_idx].x - vpoint(0), 2) + std::pow(ops_map->points[v_idx].y - vpoint(1), 2) + std::pow(ops_map->points[v_idx].z - vpoint(2), 2));
    if (distance > 120.0)
    {
      break;
    }

    p(0) = ops_map->at(v_idx).x - vh_position.cast<float>()(0);
    p(1) = ops_map->at(v_idx).y - vh_position.cast<float>()(1);
    p(2) = ops_map->at(v_idx).z - vh_position.cast<float>()(2);

    coeff = vh_direction.dot(p.normalized()) * p.norm();

    return true;

    // random sampling

    // std::vector<int> pointIdxRadiusSearch;
    // std::vector<float> pointRadiusSquaredDistance;

    // random = rand() / (float)RAND_MAX * 1.0f;
    // if(random <= point_map->points[index].intensity){
    //   p(0) = point_map->points[index].x - vpoint(0);
    //   p(1) = point_map->points[index].y - vpoint(1);
    //   p(2) = point_map->points[index].z - vpoint(2);

    //   coeff = vdirection.dot(p.normalized()) * p.norm();

    //   return true;
    // }
  }

  return false;
}

#endif // _VIRTUAL_SCAN_HPP_