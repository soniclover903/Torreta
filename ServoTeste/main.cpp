#include <FL/Fl.H>
#include "CtrlServoTeste.h"



int main (int argc, char ** argv)
{
    //long long int= duration_cast<seconds>(steady_clock::now().time_since_epoch()).count();
    ControleServoTeste UmServo;
    UmServo.MyShow();
    return(Fl::run());
}
