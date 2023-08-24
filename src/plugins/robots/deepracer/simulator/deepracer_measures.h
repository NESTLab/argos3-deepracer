#ifndef DEEPRACER_MEASURES_H
#define DEEPRACER_MEASURES_H

#include <argos3/core/utility/datatypes/datatypes.h>
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {
    extern const Real DEEPRACER_BASE_ELEVATION;
    extern const Real DEEPRACER_BASE_HEIGHT;
    extern const Real DEEPRACER_BASE_LENGTH;
    extern const Real DEEPRACER_BASE_WIDTH;
    extern const Real DEEPRACER_BASE_TOP;
    extern const Real        DEEPRACER_INERTIA_XX;
    extern const Real        DEEPRACER_INERTIA_YY;
    extern const Real        DEEPRACER_INERTIA_ZZ;

    extern const CVector2 DEEPRACER_BASE_REAR_LEFT;
    extern const CVector2 DEEPRACER_BASE_FRONT_LEFT;
    extern const CVector2 DEEPRACER_BASE_FRONT_RIGHT;
    extern const CVector2 DEEPRACER_BASE_REAR_RIGHT;

    extern const Real DEEPRACER_WHEEL_RADIUS;
    extern const Real DEEPRACER_WHEEL_DISTANCE;
    extern const Real DEEPRACER_WHEELBASE_DISTANCE;

    extern const CVector3 DEEPRACER_REAR_LEFT_WHEEL_POS_WRT_BASE;
    extern const CVector3 DEEPRACER_REAR_RIGHT_WHEEL_POS_WRT_BASE;
    extern const CVector3 DEEPRACER_FRONT_LEFT_WHEEL_POS_WRT_BASE;
    extern const CVector3 DEEPRACER_FRONT_RIGHT_WHEEL_POS_WRT_BASE;

    extern const CVector3 DEEPRACER_LEFT_CAMERA_SENSORS_POS_WRT_BASE;
    extern const CVector3 DEEPRACER_RIGHT_CAMERA_SENSORS_POS_WRT_BASE;

    extern const Real         DEEPRACER_LIDAR_POS_X_WRT_BASE;
    extern const Real         DEEPRACER_LIDAR_POS_Z_WRT_BASE;
    extern const Real         DEEPRACER_LIDAR_SENSORS_FAN_RADIUS;
    extern const CRadians     DEEPRACER_LIDAR_ANGLE_SPAN;
    extern const CRadians     DEEPRACER_LIDAR_ANGLE_START;
    extern const CRadians     DEEPRACER_LIDAR_ANGLE_END;
    extern const CRange<Real> DEEPRACER_LIDAR_SENSORS_RING_RANGE;
}

#endif // DEEPRACER_MEASURES_H
