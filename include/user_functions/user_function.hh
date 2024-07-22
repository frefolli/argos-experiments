#ifndef USER_FUNCTIONS_HH
#define USER_FUNCTIONS_HH
/** @file user_functions.hh */

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/eye-bot/simulator/eyebot_entity.h>
#include <argos3/plugins/simulator/entities/light_entity.h>

namespace prez
{
  class UserFunction : public argos::CQTOpenGLUserFunctions
  {
  public:
    UserFunction();
    virtual ~UserFunction() {}
    void DrawDroneLabel(argos::CEyeBotEntity &c_entity);
    void DrawTargetLabel(argos::CLightEntity &c_entity);
  };
}
#endif
