/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
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
 * $Id: host_device.h 1370 2011-06-19 01:06:01Z jspricke $
 *
 */

#ifndef PCL_CUDA_HOST_DEVICE_H_
#define PCL_CUDA_HOST_DEVICE_H_

#include "pcl/cuda/point_cloud.h"

namespace pcl
{
  template <typename T>
  class PointCloud;

  struct PointXYZRGB;

  namespace cuda
  {

    template <template <typename> class Storage, template <typename> class OtherStorage>
    typename PointCloudAOS<OtherStorage>::Ptr toStorage (const PointCloudAOS<Storage> &input);
  
    template <template <typename> class Storage>
    void toHost (const PointCloudAOS<Storage> &input, PointCloudAOS<Host> &output);
  
    template <template <typename> class Storage>
    void toDevice (const PointCloudAOS<Storage> &input, PointCloudAOS<Device> &output);

  } // namespace
} // namespace

#endif  //#ifndef PCL_CUDA_HOST_DEVICE_H_
