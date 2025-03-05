// Copyright (c) 2023 Direct Drive Technology Co., Ltd. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TITA_UTILS__TITA_PCL_HPP_
#define TITA_UTILS__TITA_PCL_HPP_

#include "pcl/filters/passthrough.h"
#include "pcl/filters/voxel_grid.h"
#include "pcl/point_types.h"

namespace tita_pcl
{

using PointType = pcl::PointXYZ;
using PointCloudType = pcl::PointCloud<PointType>;
using CloudPtr = PointCloudType::Ptr;

using rgbPointType = pcl::PointXYZRGB;
using rgbPointCloudType = pcl::PointCloud<rgbPointType>;
using rgbCloudPtr = rgbPointCloudType::Ptr;

void voxel_grid(CloudPtr cloud, float voxel_size);
void pass_through_box(CloudPtr incloud, CloudPtr outcloud, double * limited);

}  // namespace tita_pcl

#endif  // TITA_UTILS__TITA_PCL_HPP_
