#ifndef DEEPRACER_IMU_SENSOR_EQUIPPED_ENTITY_H
#define DEEPRACER_IMU_SENSOR_EQUIPPED_ENTITY_H

namespace argos {
    class CDeepracerIMUSensorEquippedEntity;
    class CEmbodiedEntity;
    struct SAnchor;
}

#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/core/simulator/entity/positional_entity.h>
#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/utility/math/vector3.h>

#include <map>

namespace argos {

    class CDeepracerIMUSensorEquippedEntity : public CPositionalEntity {
    public:

        ENABLE_VTABLE();

    public:

        CDeepracerIMUSensorEquippedEntity(CComposableEntity* pc_parent);

        CDeepracerIMUSensorEquippedEntity(CComposableEntity* pc_parent,
                                          const std::string& str_id,
                                          SAnchor& s_anchor,
                                          const CVector3& c_pos_offset = CVector3(),
                                          const CQuaternion& c_ang_offset = CQuaternion());

        virtual ~CDeepracerIMUSensorEquippedEntity();

        virtual void Init(TConfigurationNode& t_tree);

        virtual std::string GetTypeDescription() const {
            return "deepracer_imu_sensor";
        }

        virtual void Enable();

        virtual void Disable();

        void SetAngVelocities(Real f_roll, Real f_pitch, Real f_yaw);

        void SetLinAccelerations(Real f_x, Real f_y, Real f_z);

        inline const CVector3* GetAngVelocities() const {
            return m_pcAngVelocities;
        }

        inline const CVector3* GetLinAccelerations() const {
            return m_pcLinAccelerations;
        }

        void AddSensor(const CVector3& c_position,
                       const CVector3& c_orientation,
                       SAnchor& s_anchor);

    protected:

        SAnchor* m_psAnchor;

        CVector3 m_cPosOffset;

        CQuaternion m_cRotOffset;

        CVector3* m_pcAngVelocities;

        CVector3* m_pcLinAccelerations;
    };

}

#endif