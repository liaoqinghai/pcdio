#include <iostream>

#define PCDIO_ENABLE_CONSOLE_LOG
#include "pcdio/pcdio.h"

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

  return 0;
}
