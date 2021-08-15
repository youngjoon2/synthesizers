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

#ifndef _PCL_VIRTUAL_SCAN_H_
#define _PCL_VIRTUAL_SCAN_H_

#include <synthesizers/synthesizer.h>
#include <pcl/octree/octree_search.h>

namespace pcl
{
  template <typename PointT>
  class VirtualScan : public Synthesizer<PointT>
  {
    protected:
      using Synthesizer<PointT>::synthesizer_name_;
      using Synthesizer<PointT>::getClassName;
      using Synthesizer<PointT>::input_;
      using Synthesizer<PointT>::indices_;

      typedef typename Synthesizer<PointT>::PointCloud PointCloud;
      typedef typename PointCloud::Ptr PointCloudPtr;
      typedef typename PointCloud::ConstPtr PointCloudConstPtr;
      typedef boost::shared_ptr< VirtualScan<PointT> > Ptr;
      typedef boost::shared_ptr< const VirtualScan<PointT> > ConstPtr;

      typedef typename pcl::octree::OctreePointCloudSearch<PointT> OctreeT;

    public:

      struct LidarConfig
      {
        LidarConfig() : 
          num_channels (64), 
          hertz (10.0), 
          num_azimuth (1024), 
          start_azimuth (0),
          max_altitude (16.6), 
          range (120.0), 
          mount_position (0.0, 0.0, 0.0),
          mount_orientation (0.0, 0.0, 0.0),
          used_all_beams (true)
        {
        }

        int num_channels;

        double hertz;

        int num_azimuth;

        int start_azimuth;

        double max_altitude;

        double range;

        Eigen::Vector3d mount_position;

        Eigen::Vector3d mount_orientation;

        bool used_all_beams;
      };

        // vehicle configuration
      struct VehicleConfig
      {
        VehicleConfig() : 
          velocity(0.0), 
          position(0.0, 0.0, 0.0), 
          orientation(0.0, 0.0, 0.0)
        {
        }

        double velocity; // km/h
        
        Eigen::Vector3d position;

        Eigen::Vector3d orientation;
      };


      /** \brief Empty constructor. */
      VirtualScan () : 
        resolution_ (0.1)
      {
        synthesizer_name_ = "VirtualScan";
      }

      /** \brief Destructor. */
      virtual ~VirtualScan ()
      {
      }

      inline void 
      setVehicleConfig (const VehicleConfig &config)
      {
        vehicle_config_ = config;
      }

      inline void 
      setLidarConfig (const LidarConfig &config)
      {
        lidar_config_ = config;
      }

      inline void 
      setResolution (float resolution) 
      { 
        resolution_ = resolution;
      }

    protected:

      LidarConfig lidar_config_;

      VehicleConfig vehicle_config_;

      float resolution_;

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

#include <synthesizers/impl/virtual_scan.hpp>

#endif // _PCL_VIRTUAL_SCAN_H_