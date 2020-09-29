#include "widget.h"

#include <QApplication>
#include <QDebug>
#include <iostream>

#include "C:/eigen/Eigen/Dense"

#include "serial.h"

using namespace Eigen;


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    w.setWindowTitle("Robot Manipulator Kinematics");

    return a.exec();
}
