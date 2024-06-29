/*******************************************************************************
 * Copyright (C) 2022 MINIEYE L4 Department. All Rights Reserved
 *
 ******************************************************************************
 */

#include <iostream>
#include <unordered_map>
#include <string>

#include <stdint.h>

#define PCDIO_ENABLE_CONSOLE_LOG
#include "pcdio/pcdio.h"

/**
 *  example to convert between custom pointcloud and pcl::PCLPointCloud2
 */
struct KittiPoint {
  float x = 0;
  float y = 0;
  float z = 0;
  float intensity = 0;
};
using KittiPointCloud = std::vector<KittiPoint>;

/**
 * Save kitti pointcloud to kitti bin file
 */
bool SaveKittiPointCloud(const std::string& filename,
                         const KittiPointCloud& pointcloud) {
  std::ofstream of(filename, std::ios::binary);
  if (!of) {
    std::cerr << "[ERROR] fail to open file for writing: " << filename
              << std::endl;
    return false;
  }

  auto ptr = reinterpret_cast<const char*>(pointcloud.data());
  size_t size = pointcloud.size() * sizeof(KittiPoint);
  if (0 == size) {
    std::cout << "[WARN] 0 point to write: " << filename << "\n";
  } else {
    of.write(ptr, size);
  }

  of.close();
  return true;
}

bool FromPCLPointCloud2(const pcl::PCLPointCloud2& pc2,
                        KittiPointCloud& pointcloud) {
  std::unordered_map<std::string, pcl::PCLPointField> fields;
  for (auto& f : pc2.fields) {
    fields[f.name] = f;
  }

  // check x,y,z
  std::string xyz_name[] = {"x", "y", "z"};
  for (int i = 0; i < 3; i++) {
    if (fields.end() == fields.find(xyz_name[i])) {
      std::cerr << "[ERROR] not find field: " << xyz_name[i] << "\n";
      return false;
    }
  }

  auto ii = fields.find("intensity");
  bool has_intensity = fields.end() != ii;
  if (!has_intensity) {
    std::cout << "[WARN] not find field: intensity\n";
  }

  bool intensity_uint8 = false;
  if (ii->second.datatype == pcl::PCLPointField::UINT8) {
    intensity_uint8 = true;
  } else if (ii->second.datatype == pcl::PCLPointField::FLOAT32) {
    intensity_uint8 = false;
  } else {
    std::cerr << "[ERROR] unsupport inensity datatype: " << ii->second.datatype
              << "\n";
    return false;
  }

  size_t N = pc2.width * pc2.height;
  pointcloud.clear();
  pointcloud.resize(N);

  // we assume [count = 1]
  uint32_t ox = fields["x"].offset;
  uint32_t oy = fields["y"].offset;
  uint32_t oz = fields["z"].offset;
  uint32_t oi = ii->second.offset;

  for (int i = 0; i < N; i++) {
    auto& p = pointcloud[i];
    p.x = pc2.at<float>(i, ox);
    p.y = pc2.at<float>(i, oy);
    p.z = pc2.at<float>(i, oz);
    if (intensity_uint8) {
      p.intensity = static_cast<float>(pc2.at<uint8_t>(i, oi)) / UINT8_MAX;
    } else {
      p.intensity = pc2.at<float>(i, oi);
    }
  }

  return true;
}

bool ToPCLPointCloud2(const KittiPointCloud& pointcloud,
                      pcl::PCLPointCloud2& pc2) {
  pc2.height = 1;
  pc2.width = pointcloud.size();
  pc2.point_step = sizeof(KittiPoint);
  pc2.row_step = pc2.point_step * pc2.width;
  pc2.is_dense = 0;

  pc2.fields.clear();

  pcl::PCLPointField field;
  field.name = "x";
  field.count = 1;
  field.datatype = pcl::PCLPointField::FLOAT32;
  field.offset = 0;
  pc2.fields.push_back(field);

  field.name = "y";
  field.offset += sizeof(float);
  pc2.fields.push_back(field);

  field.name = "z";
  field.offset += sizeof(float);
  pc2.fields.push_back(field);

  field.name = "intensity";
  field.offset += sizeof(float);
  pc2.fields.push_back(field);

  size_t size = pointcloud.size() * sizeof(KittiPoint);
  pc2.data.resize(size);
  memcpy(pc2.data.data(), pointcloud.data(), size);
  return true;
}

int main(int argc, char const* argv[]) {
  if (argc < 2) {
    std::cerr << "[ERROR] no enough argument\n"
              << "usage: " << argv[0] << " <pcd_file_name>\n";
    return 1;
  }

  pcl::PCLPointCloud2 cloud;
  if (0 != pcl::io::loadPCDFile(argv[1], cloud)) {
    std::cerr << "[ERROR] failed to read pcd file: " << argv[1] << "\n";
    return 2;
  }

  std::cout << "\nwidth:      " << cloud.width
            << "\nheight:     " << cloud.height
            << "\npoint_step: " << cloud.point_step
            << "\nrow_step:   " << cloud.row_step
            << "\nis_dense:   " << static_cast<int>(cloud.is_dense)
            << "\nfields num: " << cloud.fields.size() << std::endl;

  for (auto& f : cloud.fields) {
    std::cout << f << "\n";
  }

  std::cout << "[INFO] try to save as new file\n";

  if (0 != pcl::io::savePCDFile("new.pcd", cloud)) {
    std::cerr << "[ERROR] failed to write new.pcd\n";
  } else {
    std::cout << "[INFO] success to write new.pcd\n";
  }

  KittiPointCloud kitti_point_cloud;
  if (!FromPCLPointCloud2(cloud, kitti_point_cloud)) {
    std::cerr << "[ERROR] failed to convert PCD to KITTI pointcloud\n";
    return 0;
  }

  if (SaveKittiPointCloud("kitti.bin", kitti_point_cloud)) {
    std::cout << "[INFO] save the PCD file to KITTI bin file: kitti.bin"
              << std::endl;
  } else {
    std::cerr << "[ERROR] failed to save KITTI point cloud" << std::endl;
  }

  return 0;
}
