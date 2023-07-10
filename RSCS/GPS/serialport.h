#ifndef SERIALPORT_H
#define SERIALPORT_H

#include <QObject>
#include <QSerialPort>
#include <QSerialPortInfo>
#include <QTimer>

#include "nmeaparser.h"


class SerialPort : public QObject
{
    Q_OBJECT
public:
    explicit SerialPort(QObject *parent = nullptr);
    ~SerialPort();
    int available();
    QByteArray content();


private slots:
    void connectPotr(QString port);
    void handleReadyRead();
    void handleError(QSerialPort::SerialPortError serialPortError);
    void handleStop();

public slots:
    void portManagement(bool _flagPort, QString COM);
    void validationGPS(double x, double y, double h);
    void validationSpeed(double speed);
    void searchCOMPort();
    void handelTime();

signals:
    void received(QByteArray);
    void errorGPS(int errorCode);
    void reciveDatatoForm(QString GPS);
    void stopSerialPort();
    void reciveDataGPS(double x, double y, double h);
    void reciveDataSpeed(double speed);
    void errorCoordinator(int error);
    void listCOMPort(QStringList portList);

private:
    QSerialPort *m_serialPort = nullptr;
    bool _flagStatusError = false;
    QTimer m_timer;
    QTimer *timer;
    QByteArray m_content;
    NMEAParser m_gps;
    bool _flagSendcoordinates = false;
    int minutes = 0;
};

#endif // SERIALPORT_H
