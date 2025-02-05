#include "CtrlServoTeste.h"
#include <string>
#include <iostream>


// copiado de um video do sonego

void ControleServoTeste::MyShow(){
    this -> UmaJanela -> show();
}

void ControleServoTeste::close(){
    this -> UmaJanela -> hide();
}

void ControleServoTeste::atualizar(double valor){
    this -> potenciometro -> value(valor);
    this -> slider -> value(valor);
}
void ControleServoTeste::conect(){
    int soren = SerialPort::baud9600;
    std::cout << soren << std::endl;
    this -> serial.openPort("\\\\.\\COM3");
    std::cout << "COM 1 aqui" << std::endl;
    this -> serial.configurePort(soren, SerialPort::parityNone);
}

void ControleServoTeste::converca(){
    //port
    this -> serial.clearBuffer();
    //this -> serial << "dance";
    //oq funciona
    int heidegger;
    heidegger = this -> potenciometro -> value();
    std::cout << heidegger << std::endl;
    //novo
    std::string ang = std::to_string(heidegger);
    std::string pos = "LR_TURN_TO:" + ang;
    std::cout << pos << std::endl;
    this -> serial << pos;
}
