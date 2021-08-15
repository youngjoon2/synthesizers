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

#ifndef _PCL_SYNTHESIZER_H_
#define _PCL_SYNTHESIZER_H_

#include <pcl/pcl_base.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <synthesizers/boost.h>
#include <cfloat>
#include <pcl/PointIndices.h>

namespace pcl
{

  template <typename PointT>
  class Synthesizer : public PCLBase<PointT>
  {
  public:

    using PCLBase<PointT>::indices_;
    using PCLBase<PointT>::input_;

    typedef boost::shared_ptr<Synthesizer<PointT>> Ptr;
    typedef boost::shared_ptr<const Synthesizer<PointT>> ConstPtr;

    typedef pcl::PointCloud<PointT> PointCloud;
    typedef typename PointCloud::Ptr PointCloudPtr;
    typedef typename PointCloud::ConstPtr PointCloudConstPtr;

    Synthesizer()
    {
    }

    virtual ~Synthesizer()
    {
    }

    inline void
    synthesize (PointCloud &output)
    {
      if (!initCompute())
        return;

      if (input_.get() == &output) // cloud_in = cloud_out
      {
        PointCloud output_temp;
        applySynthesizer(output_temp);
        output_temp.header = input_->header;
        output_temp.sensor_origin_ = input_->sensor_origin_;
        output_temp.sensor_orientation_ = input_->sensor_orientation_;
        pcl::copyPointCloud(output_temp, output);
      }
      else
      {
        output.header = input_->header;
        output.sensor_origin_ = input_->sensor_origin_;
        output.sensor_orientation_ = input_->sensor_orientation_;
        applySynthesizer(output);
      }

      deinitCompute();
    }

  protected:

    using PCLBase<PointT>::initCompute;
    using PCLBase<PointT>::deinitCompute;

    std::string synthesizer_name_;

    virtual void
    applySynthesizer(PointCloud &output) = 0;

    inline const std::string&
    getClassName() const
    {
      return (synthesizer_name_);
    }
  };

} // namespace pcl

#include <synthesizers/impl/synthesizer.hpp>

#endif // _PCL_SYNTHESIZER_H_