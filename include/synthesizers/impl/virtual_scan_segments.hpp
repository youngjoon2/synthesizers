
#ifndef _VIRTUAL_SCAN_SEGMENTS_HPP_
#define _VIRTUAL_SCAN_SEGMENTS_HPP_

#include <synthesizers/virtual_scan_segments.h>

template <typename PointT>
void pcl::VirtualScanSegments<PointT>::applySynthesizer (PointCloud &out_cloud)
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
void pcl::VirtualScanSegments<PointT>::launchVirtualBeams(const OctreeT &ops_octree,
                                                          const PointCloudPtr &ops_map,
                                                          PointCloud &out_cloud)
{
  if (lidar_config_.hertz == 0)
  {
    PCL_ERROR ("[pcl::VirtualScanSegments] Hertz must be not zero.\n!");
    exit (1);
  }

  if (lidar_config_.start_azimuth >= lidar_config_.num_azimuth)
  {
    PCL_ERROR ("[pcl::VirtualScanSegments] start_azimuth must be < num_azimuth.\n!");
    exit (1);
  }

  Eigen::Vector3d vh_curr_pos;

  double vh_tot_dist = vehicle_config_.velocity / (3.6 * lidar_config_.hertz); 
  double vh_tot_offset_x = vh_tot_dist * std::cos(-vehicle_config_.orientation(1)) * std::cos(vehicle_config_.orientation(2));
  double vh_tot_offset_y = vh_tot_dist * std::cos(-vehicle_config_.orientation(1)) * std::sin(vehicle_config_.orientation(2));
  double vh_tot_offset_z = vh_tot_dist * std::sin(-vehicle_config_.orientation(1));

  double vh_offset_x = vh_tot_offset_x / static_cast<double>(lidar_config_.num_azimuth);
  double vh_offset_y = vh_tot_offset_y / static_cast<double>(lidar_config_.num_azimuth);
  double vh_offset_z = vh_tot_offset_z / static_cast<double>(lidar_config_.num_azimuth);

  // start position of vehicle
  vh_curr_pos(0) = vehicle_config_.position(0) - vh_tot_offset_x / 2.0;
  vh_curr_pos(1) = vehicle_config_.position(1) - vh_tot_offset_y / 2.0;
  vh_curr_pos(2) = vehicle_config_.position(2) - vh_tot_offset_z / 2.0;
  
  for (int a_idx = lidar_config_.start_azimuth; a_idx < lidar_config_.start_azimuth + lidar_config_.num_azimuth; a_idx++)
  {
    vh_curr_pos(0) += vh_offset_x;
    vh_curr_pos(1) += vh_offset_y;
    vh_curr_pos(2) += vh_offset_z;

    Eigen::Vector3d ld_curr_pos(vh_curr_pos(0) + lidar_config_.mount_position(0), 
                                vh_curr_pos(1) + lidar_config_.mount_position(1), 
                                vh_curr_pos(2) + lidar_config_.mount_position(2));

    // need to correct azimuth angle for each channel
    // why is minus vangle? plus angle???
    float azimuth_angle = 2 * M_PI / lidar_config_.num_azimuth * (a_idx % lidar_config_.num_azimuth) + vehicle_config_.orientation(2);

    for (int c_idx = 0; c_idx < lidar_config_.num_channels; c_idx++)
    {
      float coeff;
      int num_intersected_voxel;
      std::vector<int> voxel_indices;
      Eigen::Vector3d beam_dir, e_p;

      // need to correct altitude angle (channel angle)
      float altitude_angle = (lidar_config_.max_altitude - 2 * lidar_config_.max_altitude / lidar_config_.num_channels * c_idx) * M_PI / 180;

      e_p(0) = ld_curr_pos(0) + lidar_config_.range * std::cos(azimuth_angle) * std::cos(altitude_angle);
      e_p(1) = ld_curr_pos(1) + lidar_config_.range * std::sin(azimuth_angle) * std::cos(altitude_angle);
      e_p(2) = ld_curr_pos(2) + lidar_config_.range * std::sin(altitude_angle);
      beam_dir = (e_p - ld_curr_pos).normalized();

      num_intersected_voxel = ops_octree.getIntersectedVoxelIndices(ld_curr_pos.cast<float>(), beam_dir.cast<float>(), voxel_indices);

      if (num_intersected_voxel > 0 && getCoeffCollisionPoint(voxel_indices, ld_curr_pos, e_p, beam_dir, ops_map, coeff))
      {
        pcl::PointXYZ collision_point_pcl;
        Eigen::Vector3d collision_point(ld_curr_pos + coeff * beam_dir);

        collision_point_pcl.x = collision_point(0);
        collision_point_pcl.y = collision_point(1);
        collision_point_pcl.z = collision_point(2);

        out_cloud.push_back(collision_point_pcl);
      }
    }
  }
}

template <typename PointT>
bool pcl::VirtualScanSegments<PointT>::getCoeffCollisionPoint(std::vector<int> voxel_indices,
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

#endif // _VIRTUAL_SCAN_SEGMENTS_HPP_