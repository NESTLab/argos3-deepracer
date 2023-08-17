#include "deepracer_measures.h"

/****************************************/
/****************************************/

/* LiDAR used is: SLAMTEC RPLidar A1 https://www.slamtec.ai/home/rplidar_a1/ */

/**
 * Unless otherwise specified, the values are obtained from
 * https://github.com/aws-deepracer/aws-deepracer/tree/25a64337a28d3ddd7aa3509534dce02e4b5bef9d/deepracer_description/models/xacro/urdf/deepracer_stereo_cameras_and_lidar_urdf.xacro
 */
namespace argos {
    const Real DEEPRACER_BASE_ELEVATION = 0.023249;
    const Real DEEPRACER_BASE_HEIGHT    = 0.165; // measured
    const Real DEEPRACER_BASE_TOP       = DEEPRACER_BASE_ELEVATION + DEEPRACER_BASE_HEIGHT;

    const Real DEEPRACER_WHEEL_RADIUS       = 0.035;    // measured; 0.005 more than specified in the urdf file
    const Real DEEPRACER_WHEEL_DISTANCE     = 0.159202; // from deepracer_ros_control.xacro
    const Real DEEPRACER_WHEELBASE_DISTANCE = 0.164023; // from deepracer_ros_control.xacro

    const CVector3 DEEPRACER_REAR_LEFT_WHEEL_POS_WRT_BASE   = CVector3(-0.081663, 0.08105, 0.01575);
    const CVector3 DEEPRACER_REAR_RIGHT_WHEEL_POS_WRT_BASE  = CVector3(-0.081663, -0.08105, 0.01575);
    const CVector3 DEEPRACER_FRONT_LEFT_WHEEL_POS_WRT_BASE  = CVector3(0.082311, 0.079601, 0.011759);
    const CVector3 DEEPRACER_FRONT_RIGHT_WHEEL_POS_WRT_BASE = CVector3(0.082311, -0.079601, 0.011759);

    const CVector3 DEEPRACER_LEFT_CAMERA_SENSORS_POS_WRT_BASE  = CVector3(0.091711 + 0.044755, 0.03, 0.080023 + 0.04);
    const CVector3 DEEPRACER_RIGHT_CAMERA_SENSORS_POS_WRT_BASE = CVector3(0.091711 + 0.044755, -0.03, 0.080023 + 0.04);

    const Real         DEEPRACER_LIDAR_POS_X_WRT_BASE     = 0.02913;
    const Real         DEEPRACER_LIDAR_POS_Z_WRT_BASE     = DEEPRACER_BASE_ELEVATION + 0.16145;
    const Real         DEEPRACER_LIDAR_SENSORS_FAN_RADIUS = 0.0;                                            // no radius offset; the min observable range is already included below
    const CRadians     DEEPRACER_LIDAR_ANGLE_SPAN(ToRadians(CDegrees(300)));                                // from deepracer.xacro
    const CRadians     DEEPRACER_LIDAR_ANGLE_START = (CRadians::TWO_PI - DEEPRACER_LIDAR_ANGLE_SPAN) * 0.5; // starting sweep angle
    const CRadians     DEEPRACER_LIDAR_ANGLE_END   = -DEEPRACER_LIDAR_ANGLE_START;                          // ending sweep angle
    const CRange<Real> DEEPRACER_LIDAR_SENSORS_RING_RANGE(0.15, 10.0);                                      // from deepracer.xacro
}

/****************************************/
/****************************************/
