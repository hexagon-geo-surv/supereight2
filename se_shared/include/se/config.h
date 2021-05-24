#ifndef CONFIG_H
#define CONFIG_H

#include <string>

#include <Eigen/Dense>



namespace se {
  struct Configuration {
    /**
     * A 4x4 transformation matrix post-multiplied with all poses read from the
     * ground truth file. It is used if the ground truth poses are in some
     * frame B other than the sensor frame C.
     *
     * <br>\em Default: Eigen::Matrix4f::Identity()
     */
    Eigen::Matrix4f T_BC;

    /**
     * The initial pose of the body in world frame expressed in a 4x4
     * transformation matrix.
     *
     * \note If T_BC is the Idenity matrix init_T_WB equals init_T_WC
     *
     * <br>\em Default: Eigen::Matrix4f::Identity()
     */
    Eigen::Matrix4f init_T_WB;

    /*
     * TODO
     * <br>\em Default: ""
     */
    std::string output_render_file;

    /**
     * Whether to filter the depth input frames using a bilateral filter.
     * Filtering using a bilateral filter helps to reduce the measurement
     * noise.
     *
     * <br>\em Default: false
     */
    bool bilateral_filter;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  };
} // namespace se

#endif

