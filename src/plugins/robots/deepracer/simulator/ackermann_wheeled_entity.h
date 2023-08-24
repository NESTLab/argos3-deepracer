#ifndef ACKERMANN_WHEELED_ENTITY_H
#define ACKERMANN_WHEELED_ENTITY_H

#include <argos3/core/simulator/entity/entity.h>
#include <argos3/core/utility/math/vector3.h>

namespace argos {

    class CAckermannWheeledEntity : public CEntity {
    public:

        ENABLE_VTABLE();

    public:

        CAckermannWheeledEntity(CComposableEntity* pc_parent);

        CAckermannWheeledEntity(CComposableEntity* pc_parent,
                                const std::string& str_id);

        virtual ~CAckermannWheeledEntity();

        virtual void Reset();

        inline size_t GetNumWheels() const {
            return m_unNumWheels;
        }

        void SetWheel(UInt32          un_index,
                      const CVector3& c_position,
                      Real            f_radius);

        const CVector3& GetWheelPosition(size_t un_index) const;

        inline const CVector3* GetWheelPositions() const {
            return m_pcWheelPositions;
        }

        Real GetWheelRadius(size_t un_index) const;

        inline const Real* GetWheelRadia() const {
            return m_pfWheelRadia;
        }

        inline const Real* GetSteeringAngle() const {
            return m_pfSteeringAngle;
        }

        Real GetWheelVelocity(size_t un_index) const;

        inline const Real* GetWheelVelocities() const {
            return m_pfWheelVelocities;
        }

        void SetSteeringAndThrottle(Real f_steering_ang, Real f_throttle_speed);

        virtual std::string GetTypeDescription() const {
            return "wheels";
        }

    private:

        const size_t m_unNumWheels = 4;
        CVector3*    m_pcWheelPositions;
        Real*        m_pfSteeringAngle;
        Real*        m_pfWheelRadia;
        Real*        m_pfWheelVelocities;
    };

}

#endif