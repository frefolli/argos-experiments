#ifndef USER_FUNCTIONS_HH
#define USER_FUNCTIONS_HH
/** @file user_functions.hh */

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>
#include <argos3/plugins/robots/eye-bot/simulator/eyebot_entity.h>

namespace prez
{
  class UserFunction : public argos::CQTOpenGLUserFunctions
  {
  public:
    UserFunction();
    virtual ~UserFunction() {}
    void Draw(argos::CEyeBotEntity &c_entity);
  };
}
#endif
