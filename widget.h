#ifndef WIDGET_H
#define WIDGET_H

#include <QWidget>
#include "C:/eigen/Eigen/Dense"
using namespace Eigen;

#include <QOpenGLWidget>
#include <QOpenGLFunctions>
#include <QSurfaceFormat>

#include <vector>
#include <numeric>

#include <QtCharts>
#include <QChartView>
#include <QLineSeries>
#include "serial.h"
#include "myglwidget.h"

#include <iostream>
#include <cstdio>
#include <string>


QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget, public QOpenGLFunctions
{
    Q_OBJECT

    friend class serial;

public:
    Widget(QWidget *parent = nullptr);
    ~Widget();

    Ui::Widget *ui;

    QChart *chart = new QChart();

    double theta_1_value;
    double theta_2_value;
    double theta_3_value;

    double l_1_value;
    double l_2_value;
    double l_3_value;

    Matrix4d t_0_1_matrix;
    Matrix4d t_1_2_matrix;
    Matrix4d t_2_3_matrix;
    Matrix4d t_3_4_matrix;

    Matrix4d t_0_3_matrix;

    Matrix4d t_0_2_matrix;

    Matrix4d t_1_5_matrix;
    Matrix4d t_1_4_matrix;

    Matrix4d T;

    QString Pos_x;
    QString Pos_y;
    QString Pos_z;
    QString value;
    QPoint C_Position;
    QPoint C_Position_Begining;

    int tx01;
    int tx12;
    int tx23;
    int tx34;

    int tx02;
    int tx03;

    int ty01;
    int ty12;
    int ty23;
    int ty34;

    int ty02;
    int ty03;

    int tz01;
    int tz12;
    int tz23;
    int tz34;

    int tz02;
    int tz03;

    int Tx;
    int Ty;
    int Tz;

    float x1;
    float y1;

    std::vector<QPoint> Write;

    void readNumber();

    void drawChart();
    void printMatrix(Matrix4d M);

    void f0();
    void f1();
    void f4();
    void f6();
    void f8();
    void f9();


public slots:

    void on_btn_Inverse_Kinematics_clicked();

    void on_btn_Forward_Kinematics_clicked();

    void on_send_instructions_pushButton_clicked();

    void on_rotXSlider_sliderMoved(int position);

    void on_rotYSlider_sliderMoved(int position);

    void on_rotZSlider_sliderMoved(int position);


signals:

    void dataChanged();

private slots:

    void on_WriteNumber_PushButton_clicked();
    void on_DrawNumber_pushButton_clicked();
};
#endif // WIDGET_H
