#include <sstream>
#include <user_functions/user_function.hh>
#include <controllers/controller.hh>
/****************************************/
/****************************************/

prez::UserFunction::UserFunction() {
   RegisterUserFunction<prez::UserFunction, argos::CEyeBotEntity>(&prez::UserFunction::DrawDroneLabel);
   RegisterUserFunction<prez::UserFunction, argos::CLightEntity>(&prez::UserFunction::DrawTargetLabel);
}

/****************************************/
/****************************************/

void prez::UserFunction::DrawDroneLabel(argos::CEyeBotEntity &c_entity) {
   /* The position of the text is expressed wrt the reference point of the eyebot
    * For a eye-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q eye-bot
    */
  prez::Controller& controller = static_cast<prez::Controller&>(c_entity.GetControllableEntity().GetController());
  std::ostringstream out ("");
  out << c_entity.GetId().c_str() << " -> " << controller.GetTask().target << std::endl;
   DrawText(argos::CVector3(0.0, 0.0, 0.3), // position
            out.str());      // text
}

void prez::UserFunction::DrawTargetLabel(argos::CLightEntity &c_entity) {
   /* The position of the text is expressed wrt the reference point of the eyebot
    * For a eye-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q eye-bot
    */
   DrawText(argos::CVector3(0.0, 0.0, 0.3), // position
            c_entity.GetId().c_str());      // text
}

/****************************************/
/****************************************/

using namespace prez;
REGISTER_QTOPENGL_USER_FUNCTIONS(UserFunction, "user_function")
