#ifndef DYNAMICS2D_DEEPRACER_MODEL_H
#define DYNAMICS2D_DEEPRACER_MODEL_H

namespace argos {
    class CDynamics2DAckermannSteeringControl;
    class CDynamics2DDeepracerModel;
}

#include <argos3/plugins/robots/deepracer/simulator/deepracer_entity.h>
#include <argos3/plugins/simulator/physics_engines/dynamics2d/dynamics2d_single_body_object_model.h>

#include "dynamics2d_ackermannsteering_control.h" // TODO: may need to switch paths if implementation is generic enough

namespace argos {

    class CDynamics2DDeepracerModel : public CDynamics2DSingleBodyObjectModel {
    public:

        CDynamics2DDeepracerModel(CDynamics2DEngine& c_engine,
                                  CDeepracerEntity& c_entity);
        virtual ~CDynamics2DDeepracerModel();

        virtual void Reset();

        virtual void UpdateFromEntityStatus();

    private:

        CDeepracerEntity& m_cDeepracerEntity;

        CAckermannWheeledEntity& m_cAckerWheeledEntity;

        CDynamics2DAckermannSteeringControl m_cAckerSteering;

        const Real* m_pfCurrentWheelThrottleSpeed;

        const Real* m_pfCurrentSteeringAngle;

        cpVect m_ptConvexHullCpVect[4];
    };

}

#endif
