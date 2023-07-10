#include "nmeaparser.h"
#include "qdebug.h"

#define X_LENGTH 7
#define Y_LENGTH 8
#define H_LENGTH 3

NMEAParser::NMEAParser(QObject *parent)
    : QObject{parent}
{
    qDebug() << "constuctur Class NMEAParser";
}

NMEAParser::~NMEAParser()
{
    qDebug() << "destuctor Class NMEAParser";
}

XYDOUBLE NMEAParser::parserGLL(QStringList &data)
{
    double latDeg = data[1].left(2).toDouble();
    double latMinutes = data[1].right(data[1].length()-2).toDouble();
    double latDecimals = latDeg + latMinutes / 60;

    double longDeg = data[3].left(3).toDouble();
    double longMinutes = data[3].right(data[3].length()-3).toDouble();
    double longDecimals = longDeg + longMinutes / 60;
    XYDOUBLE res;
    res.X = longDecimals;
    res.Y = latDecimals;
    qDebug() << longDecimals << " " << latDecimals;
    return res;
}


double NMEAParser::parserGGA(QStringList &data)
{
    qDebug() << data.at(9).toDouble();
    return data.at(9).toDouble();
}
double NMEAParser::parserGNRMC (const QString &line)
{
    if (line.startsWith("$GNRMC")) {
            QStringList data = line.split(",");
            if (data.size() > 7) {
                bool ok;
                double speedKnots = data[7].toDouble(&ok);
                if (ok) {
                    double speedKph = speedKnots * 1.852; // Преобразование узлов в км/ч
                    return speedKph;
                }
            }
        }
    return -1;
}
void NMEAParser::data(QByteArray d) {
    QString data = QString(d);
    qDebug() << data;
    QStringList itemDataList = data.split(",");
    if (itemDataList[0].right(3) == "GLL") {
        XYDOUBLE xy = parserGLL(itemDataList);
        gpsDataWGS84.X = xy.X;
        gpsDataWGS84.Y = xy.Y;
    } else if (itemDataList[0].right(3) == "GGA") {
        gpsDataWGS84.H = parserGGA(itemDataList);
    }


        speed = parserGNRMC(data);
        if (speed > -1) {
            emit reciveDataSpeed(speed);
        }

    //if (gpsDataWGS84.X > 1 && gpsDataWGS84.Y > 1 && gpsDataWGS84.H> 1) {
        emit reciveDataXYH(gpsDataWGS84.X, gpsDataWGS84.Y, gpsDataWGS84.H);
    //}
}
