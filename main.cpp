#include "widget.h"
#include "serial.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    Widget w;
    w.show();
    w.setWindowTitle("Robot Manipulator Kinematics");

    return a.exec();
}
