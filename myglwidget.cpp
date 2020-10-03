#include "myglwidget.h"

MyGLWidget::MyGLWidget(QWidget* parent):
    QOpenGLWidget(parent)
{
    xRot = 0;
    yRot = 0;
    zRot = 0;

}


QSize MyGLWidget::minimumSizeHint() const
{
    return QSize(50, 50);
}


QSize MyGLWidget::sizeHint() const
{
    return QSize(400, 400);
}



static void qNormalizeAngle(int &angle)
{
    while (angle < 0)
        angle += 360 * 16;
    while (angle > 360)
        angle -= 360 * 16;
}



void MyGLWidget::setXRotation(int angle)
{
    qNormalizeAngle(angle);
        if (angle != xRot) {
            xRot = angle;
            emit xRotationChanged(angle);
            update();
        }
}


void MyGLWidget::setYRotation(int angle)
{
    qNormalizeAngle(angle);
        if (angle != yRot) {
            yRot = angle;
            emit yRotationChanged(angle);
            update();
        }
}


void MyGLWidget::setZRotation(int angle)
{
    qNormalizeAngle(angle);
        if (angle != zRot) {
            zRot = angle;
            emit zRotationChanged(angle);
            update();
        }
}


void MyGLWidget::initializeGL()
{
    //This is compulsory
    this->initializeOpenGLFunctions();
    glEnable(GL_CULL_FACE);
    glShadeModel(GL_SMOOTH);

}


void MyGLWidget::paintGL()
{

    glEnable( GL_LINE_SMOOTH );
    glEnable( GL_POLYGON_SMOOTH );
    glHint( GL_LINE_SMOOTH_HINT, GL_NICEST );
    glHint( GL_POLYGON_SMOOTH_HINT, GL_NICEST );

    glClearColor(1, 1, 1, 1);
    glClear(GL_COLOR_BUFFER_BIT);
    glLineWidth(3);
    glMatrixMode(GL_PROJECTION);

    glLoadIdentity();
    glTranslatef(0.0, 0.0, 0.0);
    glRotatef(xRot / 16.0, 1.0, 0.0, 0.0);
    glRotatef(yRot / 16.0, 0.0, 1.0, 0.0);
    glRotatef(zRot / 16.0, 0.0, 0.0, 1.0);

    draw();

    glFlush();

}


void MyGLWidget::resizeGL(int width, int height)
{
    int side = qMin(width, height);
        glViewport((width - side) / 2, (height - side) / 2, side, side);

        glMatrixMode(GL_PROJECTION);
        glLoadIdentity();
    #ifdef QT_OPENGL_ES_1
        glOrthof(-2, +2, -2, +2, 1.0, 15.0);
    #else
        glOrtho(-2, +2, -2, +2, 1.0, 15.0);
    #endif
        glMatrixMode(GL_MODELVIEW);

}


void MyGLWidget::mousePressEvent(QMouseEvent *event)
{
    lastPos = event->pos();
}


void MyGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        setXRotation(xRot + 8 * dy);
        setYRotation(yRot + 8 * dx);
    } else if (event->buttons() & Qt::RightButton) {
        setXRotation(xRot + 8 * dy);
        setZRotation(zRot + 8 * dx);
    }

    lastPos = event->pos();
}


void MyGLWidget::draw()
{
    glBegin(GL_LINES);

        glColor3f(0,0,0);

        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.5f, 0.0f, 0.0f);

        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.5f, 0.0f);

        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, 0.5f);

    glEnd();
}
