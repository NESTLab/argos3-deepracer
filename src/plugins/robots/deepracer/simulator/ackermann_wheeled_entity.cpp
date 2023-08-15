#include "ackermann_wheeled_entity.h"

#include <argos3/core/simulator/space/space.h>

namespace argos {

    /****************************************/
    /****************************************/

    CAckermannWheeledEntity::CAckermannWheeledEntity(CComposableEntity* pc_parent)
        : CEntity(pc_parent) {
        m_pcWheelPositions = new CVector3[m_unNumWheels];
        m_pfWheelRadia     = new Real[m_unNumWheels];
        ::memset(m_pfWheelRadia, 0, m_unNumWheels * sizeof(Real));
        m_pfWheelVelocities = new Real[m_unNumWheels];
        ::memset(m_pfWheelVelocities, 0, m_unNumWheels * sizeof(Real));
        m_fSteeringAngle = 0.0;
        Disable();
    }

    /****************************************/
    /****************************************/

    CAckermannWheeledEntity::CAckermannWheeledEntity(CComposableEntity* pc_parent,
                                                     const std::string& str_id)
        : CEntity(pc_parent, str_id) {
        m_pcWheelPositions = new CVector3[m_unNumWheels];
        m_pfWheelRadia     = new Real[m_unNumWheels];
        ::memset(m_pfWheelRadia, 0, m_unNumWheels * sizeof(Real));
        m_pfWheelVelocities = new Real[m_unNumWheels];
        ::memset(m_pfWheelVelocities, 0, m_unNumWheels * sizeof(Real));
        m_fSteeringAngle = 0.0;
        Disable();
    }

    /****************************************/
    /****************************************/

    CAckermannWheeledEntity::~CAckermannWheeledEntity() {
        delete[] m_pcWheelPositions;
        delete[] m_pfWheelRadia;
        delete[] m_pfWheelVelocities;
    }

    /****************************************/
    /****************************************/

    void CAckermannWheeledEntity::Reset() {
        ::memset(m_pfWheelVelocities, 0, m_unNumWheels * sizeof(Real));
    }

    /****************************************/
    /****************************************/

    void CAckermannWheeledEntity::SetWheel(UInt32          un_index,
                                           const CVector3& c_position,
                                           Real            f_radius) {
        if (un_index < m_unNumWheels) {
            m_pcWheelPositions[un_index] = c_position;
            m_pfWheelRadia[un_index]     = f_radius;
        } else {
            THROW_ARGOSEXCEPTION("CAckermannWheeledEntity::SetWheel() : index " << un_index << " out of bounds (allowed [0:" << m_unNumWheels << "])");
        }
    }

    /****************************************/
    /****************************************/

    const CVector3& CAckermannWheeledEntity::GetWheelPosition(size_t un_index) const {
        if (un_index < m_unNumWheels) {
            return m_pcWheelPositions[un_index];
        } else {
            THROW_ARGOSEXCEPTION("CAckermannWheeledEntity::GetWheelPosition() : index " << un_index << " out of bounds (allowed [0:" << m_unNumWheels << "])");
        }
    }

    /****************************************/
    /****************************************/

    Real CAckermannWheeledEntity::GetWheelRadius(size_t un_index) const {
        if (un_index < m_unNumWheels) {
            return m_pfWheelRadia[un_index];
        } else {
            THROW_ARGOSEXCEPTION("CAckermannWheeledEntity::GetWheelRadius() : index " << un_index << " out of bounds (allowed [0:" << m_unNumWheels << "])");
        }
    }

    /****************************************/
    /****************************************/

    Real CAckermannWheeledEntity::GetWheelVelocity(size_t un_index) const {
        if (un_index < m_unNumWheels) {
            return m_pfWheelVelocities[un_index];
        } else {
            THROW_ARGOSEXCEPTION("CAckermannWheeledEntity::GetWheelVelocity() : index " << un_index << " out of bounds (allowed [0:" << m_unNumWheels << "])");
        }
    }

    /****************************************/
    /****************************************/

    void CAckermannWheeledEntity::SetSteeringAndThrottle(Real f_steering_ang, Real f_throttle_speed) {
        m_fSteeringAngle = f_steering_ang;
        ::memcpy(m_pfWheelVelocities, &f_throttle_speed, m_unNumWheels * sizeof(Real));
    }

    /****************************************/
    /****************************************/

    REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CAckermannWheeledEntity);

    /****************************************/
    /****************************************/

}