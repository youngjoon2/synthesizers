/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#ifndef _PCL_VIRTUAL_SCAN_PARTIAL_INPUT_H_
#define _PCL_VIRTUAL_SCAN_PARTIAL_INPUT_H_

#include <synthesizers/virtual_scan.h>
#include <pcl/kdtree/kdtree_flann.h>

namespace pcl
{
  template <typename PointT>
  class VirtualScanPartialInput : public VirtualScan<PointT>
  {
    protected:

      using VirtualScan<PointT>::synthesizer_name_;
      using VirtualScan<PointT>::getClassName;
      using VirtualScan<PointT>::indices_;
      using VirtualScan<PointT>::input_;

      using VirtualScan<PointT>::lidar_config_;
      using VirtualScan<PointT>::vehicle_config_;
      using VirtualScan<PointT>::resolution_;
      using VirtualScan<PointT>::correct_distortion_;
      using VirtualScan<PointT>::in_map_frame_;

      typedef typename VirtualScan<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef typename VirtualScan<PointT>::OctreeT OctreeT;

    public:
    
      typedef typename VirtualScan<PointT>::VehicleConfig VehicleConfig;
      typedef typename VirtualScan<PointT>::LidarConfig LidarConfig;

      /** \brief constructor. */
      VirtualScanPartialInput () :
        curr_seq_ (0)
      {
        synthesizer_name_ = "VirtualScanPartialInput";
      
        partial_inputs_.clear ();
        collision_counts_per_input_.clear ();
        ids_each_point_.clear ();
      }

      inline virtual void
      setInputCloud (const PointCloudConstPtr &cloud)
      {
        PCL_WARN ("[pcl::VirtualScanPartialInput] The setInputCloud funtion can't be used in this class. Instead, use the addPartialInputCloud funtion.\n");
      }

      inline virtual void 
      addPartialInputCloud (const PointCloudPtr &partial_input)
      {
        partial_inputs_.push_back(partial_input);
        collision_counts_per_input_.push_back(0);
        for (int i = 0; i < partial_input->size(); i++)
        {
          ids_each_point_.push_back(curr_seq_);
        }

        curr_seq_++;
      }

      inline void
      getFinalCollisionCountsPerInput(std::vector<int> &output)
      {
        output = collision_counts_per_input_;
      }

      inline std::vector<int>
      getFinalCollisionCountsPerInput()
      {
        return (collision_counts_per_input_);
      }

      inline void
      synthesize (PointCloud &output)
      {
        applySynthesizer(output);
      }

    protected:

      int curr_seq_;

      std::vector<PointCloudPtr> partial_inputs_;

      std::vector<int> collision_counts_per_input_;

      std::vector<int> ids_each_point_; // The ids of points
    
      void
      applySynthesizer (PointCloud &output);

      void 
      launchVirtualBeams(const OctreeT &ops_octree,
                          const PointCloudPtr &ops_map,
                          PointCloud &vscan);

      bool 
      getCoeffCollisionPoint(std::vector<int> voxel_indices,
                              const Eigen::Vector3d &vh_position,
                              const Eigen::Vector3d &end_position,
                              const Eigen::Vector3d &vh_direction,
                              const PointCloudPtr &ops_map,
                              float &coeff);
  };

} // namespace pcl

#include <synthesizers/impl/virtual_scan_partial_input.hpp>

#endif // _PCL_VIRTUAL_SCAN_PARTIAL_INPUT_H_