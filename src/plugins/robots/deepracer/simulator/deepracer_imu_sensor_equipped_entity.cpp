#include "deepracer_imu_sensor_equipped_entity.h"

#include <argos3/core/simulator/space/space.h>

namespace argos {

    /****************************************/
    /****************************************/

    CDeepracerIMUSensorEquippedEntity::CDeepracerIMUSensorEquippedEntity(CComposableEntity* pc_parent)
        : CEntity(pc_parent) {
        m_pcAngVelocities = new CVector3();
        m_pcLinAccelerations = new CVector3();
        Disable();
    }

    /****************************************/
    /****************************************/

    CDeepracerIMUSensorEquippedEntity::CDeepracerIMUSensorEquippedEntity(CComposableEntity* pc_parent,
                                                                         const std::string& str_id,
                                                                         SAnchor& s_anchor,
                                                                         const CVector3& c_pos_offset = CVector3(),
                                                                         const CQuaternion& c_rot_offset = CQuaternion())
        : CEntity(pc_parent, str_id), m_psAnchor(&s_anchor) {

        CVector3 cPos = c_pos_offset;
        cPos.Rotate(s_anchor.Orientation);
        cPos += s_anchor.Position;
        SetInitPosition(cPos);
        SetPosition(cPos);
        SetInitOrientation(s_anchor.Orientation * c_rot_offset);
        SetOrientation(GetInitOrientation());

        m_pcAngVelocities = new CVector3();
        m_pcLinAccelerations = new CVector3();
        Disable();
    }

    /****************************************/
    /****************************************/

    CDeepracerIMUSensorEquippedEntity::~CDeepracerIMUSensorEquippedEntity() {
        delete m_pcAngVelocities;
        delete m_pcLinAccelerations;
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUSensorEquippedEntity::Reset() {
        m_pcAngVelocities->Set(CVector3::ZERO);
        m_pcLinAccelerations->Set(CVector3::ZERO);
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUSensorEquippedEntity::SetAngVelocities(Real f_roll, Real f_pitch, Real f_yaw) {
        m_pcAngVelocities->SetX(f_roll);
        m_pcAngVelocities->SetY(f_pitch);
        m_pcAngVelocities->SetZ(f_yaw);
    }

    /****************************************/
    /****************************************/

    void CDeepracerIMUSensorEquippedEntity::SetLinAccelerations(Real f_x, Real f_y, Real f_z) {
        m_pcLinAccelerations->SetX(f_x);
        m_pcLinAccelerations->SetY(f_y);
        m_pcLinAccelerations->SetZ(f_z);
    }

    /****************************************/
    /****************************************/

    REGISTER_STANDARD_SPACE_OPERATIONS_ON_ENTITY(CDeepracerIMUSensorEquippedEntity);

    /****************************************/
    /****************************************/

}  // namespace argos