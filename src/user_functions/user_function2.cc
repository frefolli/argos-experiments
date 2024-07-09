#include <user_functions/user_function2.hh>
/****************************************/
/****************************************/

prez::UserFunction2::UserFunction2()
{
   RegisterUserFunction<prez::UserFunction2, argos::CLightEntity>(&prez::UserFunction2::Draw);
}

/****************************************/
/****************************************/

void prez::UserFunction2::Draw(argos::CLightEntity &c_entity)
{
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
REGISTER_QTOPENGL_USER_FUNCTIONS(UserFunction2, "user_function2")
