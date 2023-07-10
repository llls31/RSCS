#ifndef NMEAPARSER_H
#define NMEAPARSER_H

#include <QObject>
#include <QStringList>

struct XYHDOUBLE
{
    double X;
    double Y;
    double H;
};
struct XYDOUBLE
{
    double X;
    double Y;
};
class NMEAParser : public QObject
{
    Q_OBJECT
public:
    explicit NMEAParser(QObject *parent = nullptr);
    ~NMEAParser();
    void f_get_position(float *latitude, float *longitude, unsigned long *fix_age = nullptr);
    XYDOUBLE parserGLL(QStringList &data);
    double parserGGA(QStringList &data);
    double parserGNRMC(const QString &line);

signals:
    void reciveDataXYH(double x, double y, double h);
    void reciveDataSpeed(double speed);

public slots:
    void data(QByteArray a);

private:
    XYHDOUBLE gpsDataWGS84;
    double speed;
};

#endif // NMEAPARSER_H
