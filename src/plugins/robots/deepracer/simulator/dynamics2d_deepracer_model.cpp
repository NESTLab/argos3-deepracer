#include "dynamics2d_deepracer_model.h"

#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_engine.h>

#include "deepracer_measures.h"

namespace argos {

    /****************************************/
    /****************************************/

    static const Real DEEPRACER_MASS       = 0.4f; // TODO
    static const Real DEEPRACER_MAX_FORCE  = 1.5f; // TODO
    static const Real DEEPRACER_MAX_TORQUE = 1.5f; // TODO

    enum DEEPRACER_WHEELS {
        DEEPRACER_REAR_LEFT_WHEEL   = 0,
        DEEPRACER_REAR_RIGHT_WHEEL  = 1,
        DEEPRACER_FRONT_LEFT_WHEEL  = 2,
        DEEPRACER_FRONT_RIGHT_WHEEL = 3
    };

    /****************************************/
    /****************************************/

    CDynamics2DDeepracerModel::CDynamics2DDeepracerModel(CDynamics2DEngine& c_engine,
                                                         CDeepracerEntity& c_entity)
        : CDynamics2DSingleBodyObjectModel(c_engine, c_entity),
          m_cDeepracerEntity(c_entity),
          m_cWheeledEntity(m_cDeepracerEntity.GetWheeledEntity()),
          m_cAckerSteering(c_engine,
                           DEEPRACER_MAX_FORCE,
                           DEEPRACER_MAX_TORQUE,
                           DEEPRACER_WHEEL_DISTANCE,
                           DEEPRACER_WHEELBASE_DISTANCE,
                           c_entity.GetConfigurationNode()),
          m_fCurrentWheelVelocity(m_cWheeledEntity.GetWheelVelocities()) {
        /* Create the body with initial position and orientation */
        // TODO: change shape
        cpBody* ptBody =
            cpSpaceAddBody(GetDynamics2DEngine().GetPhysicsSpace(),
                           cpBodyNew(DEEPRACER_MASS,
                                     cpMomentForCircle(DEEPRACER_MASS,
                                                       0.0f,
                                                       DEEPRACER_BASE_RADIUS + DEEPRACER_BASE_RADIUS,
                                                       cpvzero)));
        const CVector3& cPosition = GetEmbodiedEntity().GetOriginAnchor().Position;

        ptBody->p = cpv(cPosition.GetX(), cPosition.GetY());
        CRadians cXAngle, cYAngle, cZAngle;
        GetEmbodiedEntity().GetOriginAnchor().Orientation.ToEulerAngles(cZAngle, cYAngle, cXAngle);
        cpBodySetAngle(ptBody, cZAngle.GetValue());
        /* Create the body shape */
        // TODO: change shape
        cpShape* ptShape =
            cpSpaceAddShape(GetDynamics2DEngine().GetPhysicsSpace(),
                            cpCircleShapeNew(ptBody,
                                             DEEPRACER_BASE_RADIUS,
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
        if ((m_fCurrentWheelVelocity[DEEPRACER_LEFT_WHEEL] != 0.0f) ||
            (m_fCurrentWheelVelocity[DEEPRACER_RIGHT_WHEEL] != 0.0f)) {
            m_cAckerSteering.SetSteeringAndThrottle(m_fCurrentWheelVelocity[DEEPRACER_LEFT_WHEEL],
                                                    m_fCurrentWheelVelocity[DEEPRACER_RIGHT_WHEEL]);
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
