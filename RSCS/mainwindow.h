#ifndef MAINWINDOW_H
#define MAINWINDOW_H

// Библиотека для работы с главным окном приложения Qt
#include <QMainWindow>
// Библиотека для работы с таймерами Qt
#include <QTimer>
// Библиотека для работы с диалоговыми окнами выбора файлов
#include <QFileDialog>
// Библиотека для работы с QWebEngineView, виджетом для отображения веб-страниц
#include <QtWebEngineWidgets/QWebEngineView>
// Библиотека для работы с графическими сценами в Qt
#include <QtWidgets/QGraphicsScene>
// Библиотека для работы с графическими представлениями в Qt
#include <QtWidgets/QGraphicsView>


// Заголовочный файл для работы с последовательными портами (GPS)
#include "GPS/serialport.h"
// Заголовочный файл для работы с QWebEngineView, виджетом для отображения веб-страниц
#include "QtWebEngineWidgets/qwebengineview.h"
// Заголовочный файл для класса ObjectDetector, отвечающего за распознавание объектов на изображении
#include "objectdetector.h"


// Заголовочный файл с макросами и определениями QObject
#include "QtCore/qobjectdefs.h"


// Заголовочные файлы для работы с изображениями и кодированием/декодированием изображений в OpenCV
#include <opencv2/imgcodecs.hpp>
// Заголовочные файлы для работы с основными структурами данных и функциями OpenCV
#include <opencv2/core/core.hpp>
// Заголовочные файлы для работы с различными функциями обработки изображений в OpenCV
#include <opencv2/opencv.hpp>
// Заголовочные файлы для работы с функциями обработки изображений, связанными с цветом и геометрией
#include <opencv2/imgproc.hpp>
// Заголовочные файлы для работы с нейронными сетями в OpenCV
#include <opencv2/dnn.hpp>


// Значения фокусного расстояния и размера сенсора для iPhone 14 Pro (примерные значения)
const double FOCAL_LENGTH_MM = 26.0; // фокусное расстояние в миллиметрах
const double SENSOR_HEIGHT_MM = 5.6; // высота сенсора в миллиметрах
const double SENSOR_WIDTH_MM = 7.01; // ширина сенсора в миллиметрах

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

extern "C" void requestCameraPermission();

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    // Конструктор класса MainWindow с опциональным родительским виджетом
    MainWindow(QWidget *parent = nullptr);
    // Деструктор класса MainWindow
    ~MainWindow();
    // Объект детектора объектов для обработки изображений
    ObjectDetector detector;


public slots:
    // Слот для получения списка доступных COM-портов
    void comPortList(QStringList portList);
    // Слот для вывода координат GNSS (x, y, h)
    void printCoordinate(double x, double y, double h);
    // Слот для вывода скорости
    void printSpeed(double speed);


signals:
    // Сигнал для запроса доступных COM-портов
    void receiveCOMPort();
    // Сигнал для управления COM-портом
    void controlCOM(bool _flagPort, QString COM);


private slots:
    // Обновление текущего кадра с видеопотока
    void updateFrame();
    // Переключение между камерами
    void toggleCamera();


private:
    // Указатель на пользовательский интерфейс MainWindow
    Ui::MainWindow *ui;
    // Объект для работы с последовательным портом GNSS
    SerialPort serialGNSS;
    // Объект для отображения веб-карт
    QWebEngineView *webView;
    // Объект для захвата видеопотока с камеры
    cv::VideoCapture cap;
    // Объект нейронной сети для обработки изображений
    cv::dnn::Net net;
    // Вектор с именами классов, используемых в нейронной сети
    std::vector<std::string> classNames;
    // Изображение автомобиля для отображения на пользовательском интерфейсе
    cv::Mat egoCarImg;
    // Таймер для обновления кадров с камеры
    QTimer *timer;
    // Флаг состояния управления COM-портом
    bool _flagControlCOM_GNSS = false;


private:
    // Вектор для хранения предыдущих расстояний до обнаруженных объектов
    std::vector<double> m_prevDistances;
    // Рассчитывает дистанцию до объекта, используя его реальную высоту, высоту изображения, высоту датчика, фокусное расстояние и разрешение по высоте изображения
    double calculateDistance(double realHeight, double imgHeight, double sensorHeight, double focalLength, double imgHeightResolution);
    // Конвертирует объект cv::Mat в QImage
    QImage matToQImage(const cv::Mat &mat);
    // Рассчитывает угол между двумя точками (x1, y1) и (x2, y2) в градусах
    double calculateAngle(int x1, int y1, int x2, int y2);
    // Отображает результаты распознавания объектов на кадре, используя информацию о classIds, confidences и boxes
    void displayResults(cv::Mat &frame, const std::vector<int> &classIds, const std::vector<float> &confidences, const std::vector<cv::Rect> &boxes);
    // Рисует пользовательский интерфейс с информацией о машине, детектированных объектах, их расстоянии и предыдущих расстояниях
    void drawCarInterface(cv::Mat &interface, const std::vector<int> &classIds, const std::vector<cv::Rect> &boxes, const std::vector<double> &distances, const std::vector<double> &prevDistances);
    // Обнаруживает дорожные разметки на входном изображении и возвращает результат в output
    void detectLanes(const cv::Mat &input, cv::Mat &output);
    // Определяет направление движения автомобиля на основе его положения (x) относительно ширины экрана
    int getCarDirection(int x, int screenWidth);
    // Проверяет пересечение двух прямоугольников rect1 и rect2 и возвращает true, если они пересекаются
    bool checkIntersection(const cv::Rect &rect1, const cv::Rect &rect2);
    // Определяет цвет светофора на изображении в указанном прямоугольнике, используя размер буфера кадров
    cv::Scalar getTrafficLightColor(const cv::Mat &image, const cv::Rect &box, int frameBufferSize);
};

#endif // MAINWINDOW_H
