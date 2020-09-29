#ifndef SERIAL_H
#define SERIAL_H

#include <QObject>
#include <QtSerialPort>
#include <iostream>

#include "widget.h"
#include "ui_widget.h"

#include <QWidget>

class Serial : public QWidget
{
    Q_OBJECT
public:
    explicit Serial(QWidget *parent = nullptr);

    void open(Ui::Widget *ui, const QString &name);
    void write(Ui::Widget *ui, const QString &message);

    QSerialPort *serialPort;


signals:

    void sig_quit();


public slots:

//    void slot_messageWritten(Ui::Widget *ui);
    void slot_messageWritten();

};


#endif // SERIAL_H








