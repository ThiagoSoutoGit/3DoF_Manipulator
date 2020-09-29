#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <GL/glu.h>
#include <QMouseEvent>

#include <QSurfaceFormat>




//#include "widget.h"
//#include "ui_widget.h"
//#include <glm/mat4x4.hpp>

//For mouse moviment:
//https://www.bogotobogo.com/Qt/Qt5_OpenGL_QGLWidget.php


#include <QWidget>

class MyGLWidget : public QOpenGLWidget, public QOpenGLFunctions/*, public QWidget*/
{
    Q_OBJECT
public:
    MyGLWidget(QWidget* parent=nullptr);


protected:
    //This is a virtual funciton
    void initializeGL() override;

    //This is also a virtual funciton
    void paintGL() override;

    void resizeGL(int width, int height);

    QSize minimumSizeHint() const;
    QSize sizeHint() const;
    void mousePressEvent(QMouseEvent *event);
    void mouseMoveEvent(QMouseEvent *event);

//    void draw(Ui::Widget *ui);
    void draw();


public slots:
    // slots for xyz-rotation slider
    void setXRotation(int angle);
    void setYRotation(int angle);
    void setZRotation(int angle);


signals:
    // signaling rotation from mouse movement
    void xRotationChanged(int angle);
    void yRotationChanged(int angle);
    void zRotationChanged(int angle);


private:
    int xRot;
    int yRot;
    int zRot;

    QPoint lastPos;

};

#endif // MYGLWIDGET_H
