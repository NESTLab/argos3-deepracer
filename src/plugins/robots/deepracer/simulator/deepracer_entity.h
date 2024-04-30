#ifndef DEEPRACER_ENTITY_H
#define DEEPRACER_ENTITY_H

namespace argos {
    class CControllableEntity;
    class CEmbodiedEntity;
    class CDeepracerEntity;
    class CRABEquippedEntity;
    class CProximitySensorEquippedEntity;
    class CBatteryEquippedEntity;
}

#include <argos3/core/simulator/entity/composable_entity.h>

#include <argos3/plugins/robots/deepracer/simulator/ackermann_wheeled_entity.h> // TODO: fix path when the CAckermannWheeledEntity class gets integrated to the main ARGoS3 code

namespace argos {

    class CDeepracerEntity : public CComposableEntity {
    public:

        ENABLE_VTABLE();

    public:

        CDeepracerEntity();

        CDeepracerEntity(const std::string& str_id,
                         const std::string& str_controller_id,
                         const CVector3&    c_position       = CVector3(),
                         const CQuaternion& c_orientation    = CQuaternion(),
                         Real               f_rab_range      = 3.0f,
                         size_t             un_rab_data_size = 50,
                         const std::string& str_bat_model    = "");

        virtual void Init(TConfigurationNode& t_tree);
        virtual void Reset();
        virtual void Destroy();

        virtual void UpdateComponents();

        inline CControllableEntity& GetControllableEntity() {
            return *m_pcControllableEntity;
        }

        inline CEmbodiedEntity& GetEmbodiedEntity() {
            return *m_pcEmbodiedEntity;
        }

        inline CProximitySensorEquippedEntity& GetLIDARSensorEquippedEntity() {
            return *m_pcLIDARSensorEquippedEntity;
        }

        inline CRABEquippedEntity& GetRABEquippedEntity() {
            return *m_pcRABEquippedEntity;
        }

        inline CAckermannWheeledEntity& GetWheeledEntity() {
            return *m_pcAckermannWheeledEntity;
        }

        virtual std::string GetTypeDescription() const {
            return "deepracer";
        }

    private:

        CControllableEntity*            m_pcControllableEntity;
        CEmbodiedEntity*                m_pcEmbodiedEntity;
        CProximitySensorEquippedEntity* m_pcLIDARSensorEquippedEntity;
        CRABEquippedEntity*             m_pcRABEquippedEntity;
        CAckermannWheeledEntity*        m_pcAckermannWheeledEntity;
        CBatteryEquippedEntity*         m_pcBatteryEquippedEntity;
    };
}

#endif // DEEPRACER_ENTITY_H