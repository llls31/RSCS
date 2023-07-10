#include "serialport.h"
#include <QtDebug>
#include <QCoreApplication>

SerialPort::SerialPort(QObject *parent)
    : QObject(parent)
    , m_serialPort(new QSerialPort(this))
{
    qDebug() << "constuctur  SerialPort";
}

SerialPort::~SerialPort()
{
    qDebug() << "destuctor  SerialPort";
}

void SerialPort::searchCOMPort()
{
    QStringList portList;
    const auto serialPortInfos = QSerialPortInfo::availablePorts();

    for (const QSerialPortInfo &serialPortInfo : serialPortInfos) {
        portList.append(serialPortInfo.portName());
    }
    emit listCOMPort(portList);
}

void SerialPort::portManagement(bool _flagPort, QString COM)
{
    qDebug() << _flagPort << " " << COM;
    if (_flagPort == true) {
        connectPotr(COM);
        connect(this, SIGNAL(received(QByteArray)),&m_gps,SLOT(data(QByteArray)));
        connect(&m_gps, SIGNAL(reciveDataXYH(double, double, double)) ,this ,SLOT(validationGPS(double, double, double)));
        connect(&m_gps, SIGNAL(reciveDataSpeed(double)), this, SLOT(validationSpeed(double)));

    } else {
        qDebug() << "serial Close";
        m_serialPort -> close();
    }
}

void SerialPort::handleReadyRead()
{
    QByteArray arr = m_serialPort->readAll();
        if (arr.size()) {
            m_content.clear();
            m_content.append(arr);
            emit received(arr);
        }
}

void SerialPort::handleError(QSerialPort::SerialPortError serialPortError)
{

}

void SerialPort::handleStop()
{


}
void SerialPort::validationGPS(double x, double y, double h)
{
    qDebug() << "validationGPS";
    emit reciveDataGPS(x, y, h);
}

void SerialPort::validationSpeed(double speed)
{
    qDebug() << "validationSpeed" << speed;
    if (speed) {
        emit reciveDataSpeed(speed);
    }
}

void SerialPort::handelTime()
{

}

void SerialPort::connectPotr(QString port)
{
    qDebug() << "SerialPort: connectPort";
    m_serialPort ->setPortName(port);
    m_serialPort ->setBaudRate(QSerialPort::Baud9600);
    m_serialPort ->setDataBits(QSerialPort::Data8);
    m_serialPort ->setParity(QSerialPort::NoParity);
    m_serialPort ->setStopBits(QSerialPort::OneStop);

    connect(m_serialPort, &QSerialPort::errorOccurred, this, &SerialPort::handleError);
    connect(m_serialPort, &QSerialPort::readyRead, this, &SerialPort::handleReadyRead);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(handelTime()));

    if (m_serialPort->open(QIODevice::ReadWrite)){
        qDebug() << "SerialPort: port is open";
        emit errorGPS(0);
    } else{
        qDebug() << "SerialPort: port is not open";
        timer->stop();
        m_serialPort -> close();
        emit errorGPS(1);
    }
}

int SerialPort::available()
{

}

QByteArray SerialPort::content()
{

}
