#include "widget.h"
#include "./ui_widget.h"
#include <QDebug>
#include <QString>
#include <iostream>
#include <cmath>



QT_CHARTS_USE_NAMESPACE

using namespace Eigen;

#include "C:/eigen/Eigen/Dense"


Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);

    //Connect Opengl to the Sliders
//    connect(ui->openGLWidget, SIGNAL(xRotationChanged(int)), ui->rotXSlider, SLOT(setValue(int)));
//    connect(ui->openGLWidget, SIGNAL(yRotationChanged(int)), ui->rotYSlider, SLOT(setValue(int)));
//    connect(ui->openGLWidget, SIGNAL(zRotationChanged(int)), ui->rotZSlider, SLOT(setValue(int)));


    //    QObject::connect(chart->scene(), &QGraphicsScene::changed,
    //                         &series, &Series::handleSceneChanged);


//    connect(ui->rotXSlider, SIGNAL(dataChanged()), ui->chartFrame, SLOT(repaint()));
//    connect(ui->rotYSlider, SIGNAL(dataChanged()), ui->chartFrame, SLOT(update()));
//    connect(ui->rotZSlider, SIGNAL(dataChanged()), ui->chartFrame, SLOT(update()));



    // Defining intial values for the fields

    //Project robot arm
    ui->L_1->setText("230");
    ui->L_2->setText("500");
    ui->L_3->setText("500");

    ui->Theta_1->setText("0");
    ui->Theta_2->setText("0");
    ui->Theta_3->setText("0");


    ui->matrixTextEdit->clear();

}

Widget::~Widget()
{
    delete ui;

}



Matrix4d rot_x(double alpha){
    Matrix4d T;
    T << 1, 0, 0, 0,
         0, cos(alpha), -sin(alpha), 0,
         0, sin(alpha), cos(alpha), 0,
         0, 0, 0, 1;
    return T;
}

Matrix4d trans_x(double a){
    Matrix4d T;
    T << 1, 0, 0, a,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return T;
}

Matrix4d rot_z(double theta){
    Matrix4d T;
    T << cos(theta), -sin(theta), 0, 0,
         sin(theta), cos(theta), 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return T;
}

Matrix4d trans_z(double d){
    Matrix4d T;
    T << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, d,
         0, 0, 0, 1;
    return T;
}

Matrix4d h_T(double alpha, double a, double theta, double d){
    Matrix4d T;
    T = rot_x(alpha) * trans_x(a) * rot_z(theta) * trans_z(d);
    return T;
}


void Widget::on_btn_Inverse_Kinematics_clicked()
{
    //Theta 1

    float x3 = ui->X_Pos->text().toFloat();
    float y3 = ui->Y_Pos->text().toFloat();
    float z3 = ui->Z_Pos->text().toFloat();
    float a1 = ui->L_1->text().toFloat();
    float a2 = ui->L_2->text().toFloat();
    float a3 = ui->L_3->text().toFloat();


    double theta_1, theta_2, theta_3, phi_1, phi_2, phi_3, r1, r2, r3;


    //Theta 1

    theta_1 = atan(y3/x3);  //(1)

    QString Theta1_data = QString::number(theta_1 * 180/M_PI);
    ui->Theta_1->setText(Theta1_data);


    //Theta 2

    r1 = sqrt(pow(x3, 2) + pow(y3, 2)); //(2)
    r2 = z3 - a1;    //(3)
    r3 = sqrt(pow(r1, 2) + pow(r2, 2)); //(4)

    phi_1 = acos((pow(a3, 2)-pow(a2, 2)-pow(r3, 2))/(-2*a2*r3));   //(5)

    phi_2 = atan(r2/r1);    //(6)

    theta_2 = phi_2 - phi_1;    //(7)

    QString Theta2_data = QString::number(theta_2 * 180/M_PI);
    ui->Theta_2->setText(Theta2_data);


    //Theta 3

    phi_3 = acos((pow(r3, 2)-pow(a2, 2)-pow(a3, 2))/(-2*a2*a3));    //(8)

    theta_3 = M_PI - phi_3; //(9)

    QString Theta3_data = QString::number(theta_3 * 180/M_PI);
    ui->Theta_3->setText(Theta3_data);

}


void Widget::on_btn_Forward_Kinematics_clicked()
{
    ui->matrixTextEdit->clear();
    theta_1_value = (ui->Theta_1->text()).toDouble()* M_PI/180;
    theta_2_value = (ui->Theta_2->text()).toDouble()* M_PI/180;
    theta_3_value = (ui->Theta_3->text()).toDouble()* M_PI/180;

    l_1_value = ((ui->L_1->text()).toDouble());
    l_2_value = ((ui->L_2->text()).toDouble());
    l_3_value = ((ui->L_3->text()).toDouble());

    IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");
    std::cout.precision(3);


    std::cout << std::endl <<  "Transform Matrices:" << std::endl << std::endl;;
    ui->matrixTextEdit->appendPlainText("Transform Matrices:");

    double alpha_1 = (90 * M_PI/180);
    t_0_1_matrix = h_T(0, 0, theta_1_value, l_1_value);
    std::cout << "Matrix T01:" << std::endl << t_0_1_matrix.format(CleanFmt) << std::endl << std::endl;

    tx01 = t_0_1_matrix(0, 3);
    ty01 = t_0_1_matrix(1, 3);
    tz01 = t_0_1_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT01 Matrix: \n");

    printMatrix(t_0_1_matrix);


    t_1_2_matrix = h_T(alpha_1, 0, theta_2_value, 0);
    std::cout << "Matrix T12:" << std::endl << t_0_1_matrix.format(CleanFmt) << std::endl << std::endl;

    tx12 = t_1_2_matrix(0, 3);
    ty12 = t_1_2_matrix(1, 3);
    tz12 = t_1_2_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT12 Matrix: \n");

    printMatrix(t_1_2_matrix);


    t_2_3_matrix = h_T(0, l_2_value, theta_3_value, 0);
    std::cout << "Matrix T12:" << std::endl << t_0_1_matrix.format(CleanFmt) << std::endl << std::endl;

    tx23 = t_2_3_matrix(0, 3);
    ty23 = t_2_3_matrix(1, 3);
    tz23 = t_2_3_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT23 Matrix: \n");

    printMatrix(t_2_3_matrix);


    t_3_4_matrix = h_T(0, l_3_value, 0, 0);
    std::cout << "Matrix T12:" << std::endl << t_3_4_matrix.format(CleanFmt) << std::endl << std::endl;

    tx34 = t_3_4_matrix(0, 3);
    ty34 = t_3_4_matrix(1, 3);
    tz34 = t_3_4_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT23 Matrix: \n");

    printMatrix(t_3_4_matrix);


    t_0_2_matrix = t_0_1_matrix * t_1_2_matrix;

    tx02 = t_0_2_matrix(0, 3);
    ty02 = t_0_2_matrix(1, 3);
    tz02 = t_0_2_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT02 Matrix: \n");

    printMatrix(t_0_2_matrix);


    t_0_3_matrix = t_0_1_matrix * t_1_2_matrix * t_2_3_matrix;

    tx03 = t_0_3_matrix(0, 3);
    ty03 = t_0_3_matrix(1, 3);
    tz03 = t_0_3_matrix(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT03 Matrix: \n");

    printMatrix(t_0_3_matrix);


    T = t_0_1_matrix * t_1_2_matrix * t_2_3_matrix * t_3_4_matrix;

    Tx = T(0, 3);
    Ty = T(1, 3);
    Tz = T(2, 3);

    ui->matrixTextEdit->appendPlainText("\nT Matrix: \n");

    printMatrix(T);


    Pos_x = QString::number(T(0,3));
    ui->X_Pos->setText(Pos_x);
    ui->rotXSlider->setSliderPosition(ui->Theta_1->text().toInt());
    Widget::update();


    Pos_y = QString::number(T(1,3));
    ui->Y_Pos->setText(Pos_y);
    ui->rotYSlider->setSliderPosition(ui->Theta_2->text().toInt());


    Pos_z = QString::number(T(2,3));
    ui->Z_Pos->setText(Pos_z);
    ui->rotZSlider->setSliderPosition(ui->Theta_3->text().toInt());


    //Sending Data
    ui->data_to_be_sent_textEdit->clear();

    QString data_to_be_sent;
    data_to_be_sent.append("Position (x,y,z): (");
    data_to_be_sent.append(Pos_x);
    data_to_be_sent.append(" ,");
    data_to_be_sent.append(Pos_y);
    data_to_be_sent.append(" ,");
    data_to_be_sent.append(Pos_z);
    data_to_be_sent.append(")");


    ui->data_to_be_sent_textEdit->append(data_to_be_sent);


    drawChart();
}

void Widget::drawChart()
{
    QLineSeries *series = new QLineSeries();

    *series << QPointF(0, 0) <<
               QPointF((int)tx01, (int)tz01) <<
               QPointF((int)tx03, (int)tz03) <<
               QPointF((int)Tx, (int)Tz);
    series->setPointsVisible();


    QChart *chart = new QChart();
    chart->legend()->hide();
    chart->addSeries(series);
    chart->createDefaultAxes();
    chart->setTitle("Robot Manipulator");


    QChartView *chartView = new QChartView(chart);
    chartView->setRenderHint(QPainter::Antialiasing);
    chartView->setParent(ui->chartFrame);

    QValueAxis *axisX = new QValueAxis;
    float range;
    range = (ui->L_2->text().toFloat())+
            (ui->L_3->text().toFloat());
    axisX->setRange(-range, range);
    axisX->setTickCount(10);
    axisX->setLabelFormat("%.2f");
    chartView->chart()->setAxisX(axisX, series);

    QValueAxis *axisY = new QValueAxis;
    axisY->setRange(-range, range);
    axisY->setTickCount(10);
    axisY->setLabelFormat("%.2f");
    chartView->chart()->setAxisY(axisY, series);
    chartView->update();
    Widget::update();

}

void Widget::printMatrix(Matrix4d M)
{
    for(int i = 0; i <= 3 ; ++i){
        QString row_String;
        for(int j = 0; j <= 3; ++j){
            value = QString::number(M(i,j));
            row_String.append(value);
            if(j!=3){
                row_String.append(", ");
            }
        }
        ui->matrixTextEdit->appendPlainText(row_String);
    }
}


void Widget::on_send_instructions_pushButton_clicked()
{

    Serial *s = new Serial();

    s->open(ui, "COM7");

    s->write(ui, "Hello world!");

}

void Widget::on_rotXSlider_sliderMoved(int position)
{
    QString Theta1Slider = QString::number(position);
    ui->Theta_1->setText(Theta1Slider);
    Widget::update();

    on_btn_Forward_Kinematics_clicked();
//    on_btn_Inverse_Kinematics_clicked();
//    emit dataChanged();

}

void Widget::on_rotYSlider_sliderMoved(int position)
{

   QString Theta2Slider = QString::number(position);
   ui->Theta_2->setText(Theta2Slider);

   on_btn_Forward_Kinematics_clicked();
//    on_btn_Inverse_Kinematics_clicked();
//    emit dataChanged();
}

void Widget::on_rotZSlider_sliderMoved(int position)
{
    QString Theta3Slider = QString::number(position);
    ui->Theta_3->setText(Theta3Slider);

    on_btn_Forward_Kinematics_clicked();
//    on_btn_Inverse_Kinematics_clicked();
//    emit dataChanged();
}



void Widget::on_Theta_1_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

void Widget::on_Theta_2_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

void Widget::on_Theta_3_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

void Widget::on_L_1_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

void Widget::on_L_2_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

void Widget::on_L_3_textChanged(const QString &arg1)
{
    on_btn_Forward_Kinematics_clicked();
}

//void Widget::on_X_Pos_textChanged(const QString &arg1)
//{
//    on_btn_Inverse_Kinematics_clicked();
//}

//void Widget::on_Y_Pos_textChanged(const QString &arg1)
//{
//    on_btn_Inverse_Kinematics_clicked();
//}

//void Widget::on_Z_Pos_textChanged(const QString &arg1)
//{
//    on_btn_Inverse_Kinematics_clicked();
//}

