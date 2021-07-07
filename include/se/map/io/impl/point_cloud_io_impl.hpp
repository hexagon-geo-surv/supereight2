#ifndef SE_POINT_CLOUD_IO_IMPL_HPP
#define SE_POINT_CLOUD_IO_IMPL_HPP

int save_point_cloud_vtk(se::Image<Eigen::Vector3f>& point_cloud,
                         const std::string&          filename,
                         const Eigen::Matrix4f&      T_WC)
{

  // Open the file for writing.
  std::ofstream file (filename.c_str());
  if (!file.is_open())
  {
    std::cerr << "Unable to write file " << filename << "\n";
    return 1;
  }

  file << "# vtk DataFile Version 1.0" << std::endl;
  file << "vtk mesh generated from KFusion" << std::endl;
  file << "ASCII" << std::endl;
  file << "DATASET POLYDATA" << std::endl;

  file << "POINTS " << point_cloud.size() << " FLOAT" << std::endl;

  // Write the point data.
  for (size_t i = 0; i < point_cloud.size(); ++i)
  {
    const Eigen::Vector3f point_W = (T_WC * point_cloud[i].homogeneous()).head(3);
    file << (point_W.x() + 3) / 0.04 << " "
         << (point_W.y() + 3) / 0.04 << " "
         << (point_W.z() + 3) / 0.04 << "\n";
  }

  file.close();
  return 0;
}

#endif // SE_POINT_CLOUD_IO_IMPL_HPP

