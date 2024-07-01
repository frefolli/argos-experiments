#include <user_functions/user_function.hh>
/****************************************/
/****************************************/

prez::UserFunction::UserFunction() {
   RegisterUserFunction<prez::UserFunction, argos::CEyeBotEntity>(&prez::UserFunction::Draw);
}

/****************************************/
/****************************************/

void prez::UserFunction::Draw(argos::CEyeBotEntity& c_entity) {
   /* The position of the text is expressed wrt the reference point of the eyebot
    * For a eye-bot, the reference point is the center of its base.
    * See also the description in
    * $ argos3 -q eye-bot
    */
   DrawText(argos::CVector3(0.0, 0.0, 0.3),   // position
            c_entity.GetId().c_str()); // text
}

/****************************************/
/****************************************/

using namespace prez;
REGISTER_QTOPENGL_USER_FUNCTIONS(UserFunction, "user_function")
