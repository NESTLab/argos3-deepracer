#include "ci_deepracer_imu_sensor.h"

#ifdef ARGOS_WITH_LUA
#include <argos3/core/wrappers/lua/lua_utility.h>
#endif

namespace argos {

    /****************************************/
    /****************************************/

    const CCI_DeepracerIMUSensor::SReading& CCI_DeepracerIMUSensor::GetReading() const {
        return m_sReading;
    }

    /****************************************/
    /****************************************/

#ifdef ARGOS_WITH_LUA
    void CCI_DeepracerIMUSensor::CreateLuaState(lua_State* pt_lua_state) {
        CLuaUtility::StartTable(pt_lua_state, "imu");
        CLuaUtility::AddToTable(pt_lua_state, "angular_velocity", m_sReading.AngVelocity);
        CLuaUtility::AddToTable(pt_lua_state, "linear_acceleration", m_sReading.LinAcceleration);
        CLuaUtility::EndTable(pt_lua_state);
    }
#endif

    /****************************************/
    /****************************************/

#ifdef ARGOS_WITH_LUA
    void CCI_DeepracerIMUSensor::ReadingsToLuaState(lua_State* pt_lua_state) {
        lua_getfield(pt_lua_state, -1, "imu");
        CLuaUtility::AddToTable(pt_lua_state, "angular_velocity", m_sReading.AngVelocity);
        CLuaUtility::AddToTable(pt_lua_state, "linear_acceleration", m_sReading.LinAcceleration);
        lua_pop(pt_lua_state, 1);
    }
#endif

    /****************************************/
    /****************************************/

}