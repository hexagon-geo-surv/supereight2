#ifndef SE_POINT_CLOUD_IO_HPP
#define SE_POINT_CLOUD_IO_HPP

/**
 * \brief Save a point cloud as a VTK file.
 *
 * Documentation for the VTK file format available here
 * https://vtk.org/wp-content/uploads/2015/04/file-formats.pdf.
 *
 * \param[in] point_cloud The pointcloud to save.
 * \param[in] filename    The name of the PCD file to create.
 * \param[in] T_WC        The pose from which the point cloud was observed.
 * \return 0 on success, nonzero on error.
 */
int save_point_cloud_vtk(se::Image<Eigen::Vector3f>& point_cloud,
                         const std::string&          filename,
                         const Eigen::Matrix4f&      T_WC);



#include "impl/point_cloud_io_impl.hpp"

#endif //SE_POINT_CLOUD_IO_HPP
