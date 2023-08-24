#ifndef QTOPENGL_DEEPRACER_H
#define QTOPENGL_DEEPRACER_H

namespace argos {
   class CQTOpenGLDeepracer;
   class CDeepracerEntity;
}

#ifdef __APPLE__
#include <gl.h>
#else
#include <GL/gl.h>
#endif

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_obj_model.h>

namespace argos {

   class CQTOpenGLDeepracer {

   public:

      CQTOpenGLDeepracer();

      virtual ~CQTOpenGLDeepracer();

      virtual void Draw(CDeepracerEntity& c_entity);

   private:

      CQTOpenGLObjModel m_cBodyModel;

   };

}

#endif