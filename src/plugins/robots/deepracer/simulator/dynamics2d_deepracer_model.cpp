#include "dynamics2d_deepracer_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

#include "deepracer_measures.h"

namespace argos {

    /****************************************/
    /****************************************/

    static const Real DEEPRACER_MASS       = 1.9068f; // https://github.com/aws-deepracer/aws-deepracer/blob/25a64337a28d3ddd7aa3509534dce02e4b5bef9d/deepracer_description/models/xacro/urdf/deepracer_stereo_cameras_and_lidar_urdf.xacro#L27C20-L27C26
    static const Real DEEPRACER_MAX_FORCE  = 1.5f;    // max force to get the controlled body to match the control (virtual) body's velocity; value is the same as defined in other robots
    static const Real DEEPRACER_MAX_TORQUE = 1.5f;    // max torque to get the controlled body to match the control (virtual) body's velocity; value is the same as defined in other robots

    enum DEEPRACER_WHEELS {
        DEEPRACER_REAR_LEFT_WHEEL   = 0,
        DEEPRACER_REAR_RIGHT_WHEEL  = 1,
        DEEPRACER_FRONT_LEFT_WHEEL  = 2,
        DEEPRACER_FRONT_RIGHT_WHEEL = 3
    };

    /****************************************/
    /****************************************/

    CDynamics2DDeepracerModel::CDynamics2DDeepracerModel(CDynamics2DEngine& c_engine,
                                                         CDeepracerEntity&  c_entity)
        : CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
          m_cDeepracerEntity(c_entity),
          m_cAckerWheeledEntity(m_cDeepracerEntity.GetWheeledEntity()),
          m_cAckerSteering(c_engine,
                           DEEPRACER_MAX_FORCE,
                           DEEPRACER_MAX_TORQUE,
                           DEEPRACER_WHEEL_DISTANCE,
                           DEEPRACER_WHEELBASE_DISTANCE,
                           c_entity.GetConfigurationNode()),
          m_pfCurrentWheelThrottleSpeed(m_cAckerWheeledEntity.GetWheelVelocities()),
          m_pfCurrentSteeringAngle(m_cAckerWheeledEntity.GetSteeringAngle()) {
        /* Create the body with initial position and orientation */
        cpBody* ptBody =
            cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                           cpBodyNew(DEEPRACER_MASS,
                                     cpMomentForBox(DEEPRACER_MASS,
                                                    DEEPRACER_BASE_LENGTH,
                                                    DEEPRACER_BASE_WIDTH)));
        const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;

        ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
        CRadians cXAngle, cYAngle, cZAngle;
        GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        cpBodySetAngle(ptBody, cZAngle.GetValue());

        /* Create the body shape */
        m_ptConvexHullCpVect[0].x = DEEPRACER_BASE_REAR_LEFT.GetX();
        m_ptConvexHullCpVect[0].y = DEEPRACER_BASE_REAR_LEFT.GetY();
        m_ptConvexHullCpVect[1].x = DEEPRACER_BASE_FRONT_LEFT.GetX();
        m_ptConvexHullCpVect[1].y = DEEPRACER_BASE_FRONT_LEFT.GetY();
        m_ptConvexHullCpVect[2].x = DEEPRACER_BASE_FRONT_RIGHT.GetX();
        m_ptConvexHullCpVect[2].y = DEEPRACER_BASE_FRONT_RIGHT.GetY();
        m_ptConvexHullCpVect[3].x = DEEPRACER_BASE_REAR_RIGHT.GetX();
        m_ptConvexHullCpVect[3].y = DEEPRACER_BASE_REAR_RIGHT.GetY();
        cpShape* ptShape =
            cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                            cpPolyShapeNew(ptBody,
                                           4,
                                           m_ptConvexHullCpVect,
                                           cpvzero));
        ptShape->e = 0.0; // No elasticity
        ptShape->u = 0.7; // Lots of friction
        /* Constrain the actual base body to follow the diff steering control */
        m_cAckerSteering.AttachTo(ptBody);
        /* Set the body so that the default methods work as expected */
        SetBody(ptBody, DEEPRACER_BASE_TOP);
    }

    /****************************************/
    /****************************************/

    CDynamics2DDeepracerModel::~CDynamics2DDeepracerModel() {
        m_cAckerSteering.Detach();
    }

    /****************************************/
    /****************************************/

    void CDynamics2DDeepracerModel::Reset() {
        CDynamics2DSingleBodyObjectModel::Reset();
        m_cAckerSteering.Reset();
    }

    /****************************************/
    /****************************************/

    void CDynamics2DDeepracerModel::UpdateFromEntityStatus() { // TODO: implement correct code
        /* Do we want to move? */
        if ((m_pfCurrentWheelThrottleSpeed[DEEPRACER_REAR_LEFT_WHEEL] != 0.0f) ||
            (m_pfCurrentWheelThrottleSpeed[DEEPRACER_REAR_RIGHT_WHEEL] != 0.0f) ||
            (m_pfCurrentWheelThrottleSpeed[DEEPRACER_FRONT_LEFT_WHEEL] != 0.0f) ||
            (m_pfCurrentWheelThrottleSpeed[DEEPRACER_FRONT_RIGHT_WHEEL] != 0.0f)) {
            m_cAckerSteering.SetSteeringAndThrottle(*m_pfCurrentSteeringAngle,
                                                    m_pfCurrentWheelThrottleSpeed[DEEPRACER_REAR_LEFT_WHEEL]); // all 4 wheels have the same speed
        } else {
            /* No, we don't want to move - zero all speeds */
            m_cAckerSteering.Reset();
        }
    }

    /****************************************/
    /****************************************/

    REGISTER_STANDARD_DYNAMICS2D_OPERATIONS_ON_ENTITY(CDeepracerEntity, CDynamics2DDeepracerModel);

    /****************************************/
    /****************************************/

}
