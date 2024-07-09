#ifndef USER_FUNCTIONS2_HH
#define USER_FUNCTIONS2_HH
/** @file user_functions2.hh */

#include <argos3/plugins/simulator/visualizations/qt-opengl/qtopengl_user_functions.h>

#include <argos3/plugins/simulator/entities/light_entity.h>
namespace prez
{
  class UserFunction2 : public argos::CQTOpenGLUserFunctions
  {
  public:
    UserFunction2();
    virtual ~UserFunction2() {}
    void Draw(argos::CLightEntity &c_entity);
  };
}
#endif
