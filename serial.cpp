#include "serial.h"


Serial::Serial(QWidget *parent) : QWidget(parent)
{

    serialPort = new QSerialPort();

//    QObject::connect(serialPort, SIGNAL(bytesWritten(qint64)), this, SLOT(slot_messageWritten(Ui::Widget *ui)));
    QObject::connect(serialPort, SIGNAL(bytesWritten(qint64)), this, SLOT(slot_messageWritten()));

}


void Serial::open(Ui::Widget *ui, const QString &name)
{
    serialPort->setPortName(name);
    serialPort->setBaudRate(115200);
    serialPort->setDataBits(QSerialPort::Data8);
    serialPort->setParity(QSerialPort::NoParity);
    serialPort->setStopBits(QSerialPort::OneStop);

    if(!serialPort->isOpen())
    {
        serialPort->close();
    }
    std::cout << "Serial Port " << name.toStdString() << " opened!" << std::endl;
    QString communication;
    communication.append("Serial Port " + name + " opened!");
    ui->communication_textEdit->append(communication);

    serialPort->open(QIODevice::ReadWrite);
}


void Serial::write(Ui::Widget *ui, const QString &message)
{
    QString Theta1_sent;

    int bytesWritten = serialPort->write(message.toUtf8());

    std::cout << "Bytes written: " << bytesWritten << std::endl;

    QString communication;
    communication.append("Bytes written: " + message);
    ui->communication_textEdit->append(communication + "\n");



}


//void Serial::slot_messageWritten(Ui::Widget *ui)
void Serial::slot_messageWritten()

{
    std::cout << "Serial Port closed!" << std::endl;
//    ui->communication_textEdit->append("Serial Port closed!");

    serialPort->close();

    emit sig_quit();
}
