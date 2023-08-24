#include "qtopengl_deepracer.h"
#include "deepracer_entity.h"
#include "deepracer_measures.h"
#include <argos3/core/utility/math/vector2.h>
#include <argos3/core/simulator/entity/embodied_entity.h>
#include <argos3/plugins/simulator/entities/led_equipped_entity.h>
#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_widget.h>
#include <QImage>

namespace argos {

   /****************************************/
   /****************************************/

   CQTOpenGLDeepracer::CQTOpenGLDeepracer() :
      m_cBodyModel("deepracer.obj") {
   }

   /****************************************/
   /****************************************/

   CQTOpenGLDeepracer::~CQTOpenGLDeepracer() {
   }

   /****************************************/
   /****************************************/

   void CQTOpenGLDeepracer::Draw(CDeepracerEntity& c_entity) {
      m_cBodyModel.Draw();
   }

   /****************************************/
   /****************************************/

   class CQTOpenGLOperationDrawDeepracerNormal : public CQTOpenGLOperationDrawNormal {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CDeepracerEntity& c_entity) {
         static CQTOpenGLDeepracer m_cModel;
         c_visualization.DrawRays(c_entity.GetControllableEntity());
         c_visualization.DrawEntity(c_entity.GetEmbodiedEntity());
         m_cModel.Draw(c_entity);
      }
   };

   class CQTOpenGLOperationDrawDeepracerSelected : public CQTOpenGLOperationDrawSelected {
   public:
      void ApplyTo(CQTOpenGLWidget& c_visualization,
                   CDeepracerEntity& c_entity) {
         c_visualization.DrawBoundingBox(c_entity.GetEmbodiedEntity());
      }
   };

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawNormal, CQTOpenGLOperationDrawDeepracerNormal, CDeepracerEntity);

   REGISTER_QTOPENGL_ENTITY_OPERATION(CQTOpenGLOperationDrawSelected, CQTOpenGLOperationDrawDeepracerSelected, CDeepracerEntity);

   /****************************************/
   /****************************************/

}