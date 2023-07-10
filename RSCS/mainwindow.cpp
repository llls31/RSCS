#include "mainwindow.h"
#include "ui_mainwindow.h"

// Конструктор класса MainWindow с опциональным родительским виджетом
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
       , ui(new Ui::MainWindow)
       , detector("/Users/nikita/Projects/Qt/RSCS/RSCS/yolov3-spp.weights",
                  "/Users/nikita/Projects/Qt/RSCS/RSCS/yolov3-spp.cfg", "darknet",
                   {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
                  "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
                  "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
                  "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis",
                  "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
                  "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon",
                  "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog",
                  "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed", "diningtable",
                  "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
                  "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase",
                  "scissors", "teddy bear", "hair drier", "toothbrush"}) {
    ui->setupUi(this);
    // GNSS Port

    // Установить соединение между сигналом controlCOM и слотом portManagement для управления GPS-портом
    connect(this, SIGNAL(controlCOM(bool, QString)), &serialGNSS, SLOT(portManagement(bool, QString)));
    // Установить соединение между сигналом receiveCOMPort и слотом searchCOMPort для поиска доступных COM-портов
    connect(this, SIGNAL(receiveCOMPort()), &serialGNSS, SLOT(searchCOMPort()));
    // Установить соединение между сигналом listCOMPort и слотом comPortList для получения списка доступных COM-портов
    connect(&serialGNSS, SIGNAL(listCOMPort(QStringList)), this, SLOT(comPortList(QStringList)));
    // Установить соединение между сигналом reciveDataSpeed и слотом printSpeed для вывода скорости
    connect(&serialGNSS, SIGNAL(reciveDataSpeed(double)), this, SLOT(printSpeed(double)));
    // Установить соединение между сигналом reciveDataGPS и слотом printCoordinate для вывода координат
    connect(&serialGNSS, SIGNAL(reciveDataGPS(double,double,double)), this, SLOT(printCoordinate(double,double,double)));

    // btn_search_COM
    connect(ui->btn_searchCOM, &QAbstractButton::clicked, ui->btn_searchCOM, [this] { emit receiveCOMPort(); });

    connect(ui->btn_controlCOM_GNSS, &QAbstractButton::clicked, ui->btn_controlCOM_GNSS, [this] {
        _flagControlCOM_GNSS =! _flagControlCOM_GNSS;
        emit controlCOM(_flagControlCOM_GNSS, ui->comboBox_GNSS->currentText());
    });

    // stackedWidget page Situation
    connect(ui->btn_situation, &QAbstractButton::clicked, ui->btn_situation, [this] { ui->stackedWidget->setCurrentIndex(0); });

    // stackedWidget page Settings
    connect(ui->btn_settings, &QAbstractButton::clicked, ui->btn_settings, [this] { ui->stackedWidget->setCurrentIndex(2); });

    // stackedWidget page Map
    connect (ui -> btn_map, &QAbstractButton::clicked, ui -> btn_map, [this] { ui -> stackedWidget -> setCurrentIndex(1); });

    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::toggleCamera);
    timer = new QTimer(this);
    connect(timer, &QTimer::timeout, this, &MainWindow::updateFrame);

    classNames = {"person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
                  "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
                  "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack",
                  "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball",
                  "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle",
                  "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
                  "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
                  "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave",
                  "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

    QGraphicsView* map = ui->map;
    QGraphicsScene* scene = new QGraphicsScene();
    map->setScene(scene);

    QWebEngineView* webView = new QWebEngineView(map);
    map->setViewport(webView);

    QUrl url("https://www.openstreetmap.org");
    webView->load(url);

    QWebEnginePage *page = webView->page();

    requestCameraPermission();
    qDebug() << "constuctur MainWindow";
}

// Деструктор класса MainWindow
MainWindow::~MainWindow()
{
    qDebug() << "destuctor MainWindow";
    delete ui;
}


// Обновление текущего кадра с видеопотока
void MainWindow::updateFrame()
{
    cv::Mat frame;
    cap.read(frame);

    if (frame.empty()) {
        qDebug() << "Error: Cannot read a frame from the camera.";
        return;
    }

    std::vector<int> classIds;
    std::vector<float> confidences;
    std::vector<cv::Rect> boxes;

    detector.detectObjects(frame, classIds, confidences, boxes);

    displayResults(frame, classIds, confidences, boxes);

    QImage qFrame = matToQImage(frame);
    ui->label->setPixmap(QPixmap::fromImage(qFrame).scaled(ui->label->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
}


// Отображает результаты распознавания объектов на кадре, используя информацию о classIds, confidences и boxes
void MainWindow::displayResults(cv::Mat &frame, const std::vector<int> &classIds, const std::vector<float> &confidences, const std::vector<cv::Rect> &boxes) {
    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, 0.5, 0.4, indices);

    cv::Mat laneDetectionResult;
    detectLanes(frame, laneDetectionResult);
    cv::addWeighted(frame, 1.0, laneDetectionResult, 0.3, 0, frame);

    cv::Mat carInterface = cv::Mat::zeros(frame.rows, frame.cols, CV_8UC3);
    cv::Mat combinedResult = laneDetectionResult.clone();

    std::vector<double> distances;
    for (int idx : indices) {
        int classId = classIds[idx];

        if (classId != 0 && classId != 1 && classId != 2 && classId != 3 && classId != 5 && classId != 6 && classId != 7 && classId != 8 && classId != 9 && classId != 10 && classId != 11 && classId != 12) {
            continue;
        }

        cv::Rect box = boxes[idx];
        float confidence = confidences[idx];
        cv::Scalar trafficLightColor = cv::Scalar(0, 255, 0);

        if (classId == 9) {
            trafficLightColor = getTrafficLightColor(frame, box, 20);

            std::string colorText;
            if (trafficLightColor == cv::Scalar(0, 0, 255)) {
                colorText = "Red";
                qDebug() << "getTrafficLightColor ( Red )";

                ui->label_trafficLights_red->setStyleSheet("background: #FF0000;  border: 1px solid #000000; border-radius: 60px;");
                ui->label_trafficLights_green->setStyleSheet("background: #434546; border: 1px solid #000000; border-radius: 60px;");

            } else if (trafficLightColor == cv::Scalar(0, 255, 0)) {
                colorText = "Green";
                qDebug() << "getTrafficLightColor ( Green )";

                ui->label_trafficLights_red->setStyleSheet("background: #434546;  border: 1px solid #000000; border-radius: 60px;");
                ui->label_trafficLights_green->setStyleSheet("background: #3CC539; border: 1px solid #000000; border-radius: 60px;");
            } else {
                colorText = "Unknown";
                ui->label_trafficLights_red->setStyleSheet("background: #434546;  border: 1px solid #000000; border-radius: 60px;");
                ui->label_trafficLights_green->setStyleSheet("background: #434546; border: 1px solid #000000; border-radius: 60px;");
            }

            cv::putText(frame, colorText, cv::Point(box.x, box.y - 5), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 255, 255), 2);
        }

        cv::rectangle(frame, box, trafficLightColor, 4);

        double realHeight = 100.0;
        double distance = calculateDistance(realHeight, box.height, SENSOR_HEIGHT_MM, FOCAL_LENGTH_MM, frame.rows) / 1000;

        distances.push_back(distance);

        std::stringstream distanceText;
        distanceText << "Distance to " << classNames[classId] << ": " << std::fixed << std::setprecision(2) << distance << " m";
        ui->textBrowser_log->append(QString::fromStdString(distanceText.str()));

        int frameCenterX = frame.cols / 2;
        int frameCenterY = frame.rows / 2;
        int objectCenterX = box.x + box.width / 2;
        int objectCenterY = box.y + box.height / 2;
        int deltaX = objectCenterX - frameCenterX;
        int deltaY = objectCenterY - frameCenterY;
        cv::line(frame, cv::Point(frameCenterX, frameCenterY), cv::Point(objectCenterX, objectCenterY), cv::Scalar(255, 0, 0), 2);

        std::string direction;
        if (std::abs(deltaX) < frame.cols * 0.1 && std::abs(deltaY) < frame.rows * 0.1) {
            direction = "Center";
        } else {
            if (deltaX > 0) {
                direction = "Right";
            } else {
                direction = "Left";
            }
            if (deltaY > 0) {
                direction += " Bottom";
            } else {
                direction += " Top";
            }
        }

        std::stringstream objectInfo;
        objectInfo << "Distance to " << classNames[classId] << ": " << std::fixed << std::setprecision(2) << distance << " m, Direction: " << direction;
        ui->textBrowser_log->append(QString::fromStdString(objectInfo.str()));

        char labelBuffer[50];
            std::sprintf(labelBuffer, "%s: %.2fm", classNames[classId].c_str(), distance);
            std::string label(labelBuffer);
            int baseLine;
            cv::Size labelSize = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 1.0, 2, &baseLine);
            int top = std::max(box.y, labelSize.height);
            cv::rectangle(frame, cv::Point(box.x, top - labelSize.height), cv::Point(box.x + labelSize.width, top + baseLine), cv::Scalar(255, 255, 255), cv::FILLED);
            cv::putText(frame, label, cv::Point(box.x, top), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 0, 0), 2);
        }
    m_prevDistances = distances;
    drawCarInterface(combinedResult, classIds, boxes, distances,m_prevDistances);
}


// Конвертирует объект cv::Mat в QImage
QImage MainWindow::matToQImage(const cv::Mat &mat)
{
    switch (mat.type()) {
        case CV_8UC1:
            return QImage(mat.data, mat.cols, mat.rows, mat.step, QImage::Format_Grayscale8);
        case CV_8UC3: {
            cv::Mat rgb;
            cvtColor(mat, rgb, cv::COLOR_BGR2RGB);
            return QImage(rgb.data, rgb.cols, rgb.rows, rgb.step, QImage::Format_RGB888);
        }
            default:
                qWarning() << "Unsupported Mat format: " << mat.type();
                return QImage();
        }
}


// Рассчитывает дистанцию до объекта, используя его реальную высоту, высоту изображения, высоту датчика, фокусное расстояние и разрешение по высоте изображения
double MainWindow::calculateDistance(double realHeight, double imgHeight, double sensorHeight, double focalLength, double imgHeightResolution)
{
    double imgHeightMM = (imgHeight / imgHeightResolution) * sensorHeight;
    double distance = (realHeight * focalLength) / imgHeightMM;
    return distance;
}


// Определяет цвет светофора на изображении в указанном прямоугольнике, используя размер буфера кадров
cv::Scalar MainWindow::getTrafficLightColor(const cv::Mat &image, const cv::Rect &box, int frameBufferSize) {
    static std::deque<std::pair<int, int>> frameBuffer;

    cv::Rect validROI = box & cv::Rect(0, 0, image.cols, image.rows);
    cv::Mat croppedImage = image(validROI);

    cv::Mat hsvImage;
    cv::cvtColor(croppedImage, hsvImage, cv::COLOR_BGR2HSV);

    int redCount = 0;
    int greenCount = 0;

    for (int i = 0; i < hsvImage.rows; ++i) {
        for (int j = 0; j < hsvImage.cols; ++j) {
            cv::Vec3b pixel = hsvImage.at<cv::Vec3b>(i, j);

            int hue = pixel[0];
            int saturation = pixel[1];
            int value = pixel[2];

            if (hue >= 0 && hue <= 10 && saturation >= 100 && value >= 100) {
                redCount += 6; // Increase sensitivity for red
            } else if (hue >= 40 && hue <= 80 && saturation >= 100 && value >= 100) {
                greenCount += 1; // Increase sensitivity for green
            }
        }
    }

    frameBuffer.push_back(std::make_pair(redCount, greenCount));
    if (frameBuffer.size() > frameBufferSize) {
        frameBuffer.pop_front();
    }
    int avgRedCount = 0;
    int avgGreenCount = 0;
    for (const auto& counts : frameBuffer) {
        avgRedCount += counts.first;
        avgGreenCount += counts.second;
    }
    avgRedCount /= frameBuffer.size();
    avgGreenCount /= frameBuffer.size();

    if (avgRedCount > avgGreenCount) {
        return cv::Scalar(0, 0, 255); // Red
    } else {
        return cv::Scalar(0, 255, 0); // Green
    }
}


// Определяет направление движения автомобиля на основе его положения (x) относительно ширины экрана
int MainWindow::getCarDirection(int x, int screenWidth) {
    if (x < screenWidth / 2) {
        return -1; // Left
    } else {
        return 1; // Right
    }
}


// Проверяет пересечение двух прямоугольников rect1 и rect2 и возвращает true, если они пересекаются
bool MainWindow::checkIntersection(const cv::Rect &rect1, const cv::Rect &rect2) {
    return (rect1 & rect2).area() > 0;
}


// Рассчитывает угол между двумя точками (x1, y1) и (x2, y2) в градуса
double MainWindow::calculateAngle(int x1, int y1, int x2, int y2) {
    int dx = x2 - x1;
    int dy = y2 - y1;
    return atan2(dy, dx) * 180 / CV_PI;
}


// Рисует пользовательский интерфейс с информацией о машине, детектированных объектах, их расстоянии и предыдущих расстояниях
void MainWindow::drawCarInterface(cv::Mat &interface, const std::vector<int> &classIds, const std::vector<cv::Rect> &boxes, const std::vector<double> &distances, const std::vector<double> &prevDistances) {
    qDebug() << "Start of drawCarInterface";
    // Очистить интерфейс (например, фон дороги)
    interface.setTo(cv::Scalar(255, 255, 255));
    // Нарисовать дорогу
    cv::rectangle(interface, cv::Rect(0, interface.rows / 2 - 375, 1400, 750), cv::Scalar(100, 100, 100), -1);
    // Нарисовать полосы
    int numLanes = 3;
    int laneWidth = 1400 / numLanes;
    for (int i = 1; i < numLanes; ++i) {
        cv::line(interface, cv::Point(i * laneWidth, interface.rows / 2 - 375), cv::Point(i * laneWidth, interface.rows / 2 + 375), cv::Scalar(255, 255, 255), 2);
    }
    // Нарисовать машину-эгоиста (прямоугольник)
    int egoCarWidth = 50;
    int egoCarHeight = 100;
    int egoCarX = 1400 / 2 - egoCarWidth / 2;
    int egoCarY = interface.rows / 2 - egoCarHeight / 2;
    cv::Rect egoCarRect(egoCarX, egoCarY, egoCarWidth, egoCarHeight);
    cv::rectangle(interface, egoCarRect, cv::Scalar(255, 0, 0), -1);
    // Нарисовать стрелку направления для машины-эгоиста
    cv::arrowedLine(interface, cv::Point(egoCarX + egoCarWidth / 2, egoCarY + egoCarHeight / 2),
                    cv::Point(egoCarX + egoCarWidth / 2, egoCarY - egoCarHeight / 2),
                    cv::Scalar(0, 255, 0), 2);

    // ... остальная часть функции
    for (size_t i = 0; i < boxes.size(); ++i) {
            if (classIds[i] != 2) { // Пропустить, если объект не машина
                continue;
            }
            // Проверить минимальное расстояние между машинами (1 метр)
               if (distances[i] <= 1) {
                   continue; // Если машина слишком близко, не рисовать ее
               }

               int carWidth = 30; // Можно установить размер в зависимости от distance
               int carHeight = 60;

               // Определить, находится ли машина слева, справа или перед машины эгоиста
               bool isCarLeft = boxes[i].x + boxes[i].width / 2 < egoCarX - egoCarWidth / 4;
               bool isCarRight = boxes[i].x + boxes[i].width / 2 > egoCarX + egoCarWidth * 3 / 4;

               // Вычислить X координату машины на интерфейсе
               int carX;
               if (isCarLeft) {
                   carX = egoCarX - (egoCarWidth / 2 + (distances[i] - egoCarWidth / 2) * 5); // Машина слева от эгоиста
               } else if (isCarRight) {
                   carX = egoCarX + egoCarWidth / 2 + (distances[i] - egoCarWidth / 2) * 5; // Машина справа от эгоиста
               } else {
                   carX = egoCarX; // Машина перед эгоистом
               }

               // Вычислить Y координату машины на интерфейсе
               int carY;
               if (isCarLeft || isCarRight) {
                   carY = egoCarY - (distances[i] * 5); // Машина слева или справа от эгоиста
               } else {
                   carY = egoCarY - egoCarHeight - (prevDistances[i] - distances[i]) * 5; // Машина перед эгоистом
               }

               cv::Rect carRect(carX, carY, carWidth, carHeight);
               cv::rectangle(interface, carRect, cv::Scalar(0, 0, 255), -1);

               // Нарисовать стрелку направления для распознанных машин
               cv::arrowedLine(interface, cv::Point(carX + carWidth / 2, carY + carHeight / 2),
                               cv::Point(carX + carWidth / 2, carY - carHeight / 2),
                               cv::Scalar(0, 255, 0), 2);
                   qDebug() << "Car detected: Class ID:" << classIds[i] << "Box:" << boxes[i].x << boxes[i].y << "Distance:" << distances[i] << "Prev Distance:" << prevDistances[i];
                   qDebug() << "Car position on radar: x =" << carX << "y =" << carY;
            }

        QImage qimg = matToQImage(interface);
        ui->label_radar->setPixmap(QPixmap::fromImage(qimg));
}


// Обнаруживает дорожные разметки на входном изображении и возвращает результат в output
void MainWindow::detectLanes(const cv::Mat &input, cv::Mat &output) {
    // Конвертация изображения в оттенки серого
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

    // Применение размытия Гаусса
    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(5, 5), 1.5);

    // Обнаружение границ с помощью Canny
    cv::Mat edges;
    cv::Canny(blurred, edges, 50, 150);

    // Применение преобразования Хафа для линий
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(edges, lines, 1, CV_PI / 180, 50, 60, 10);

    // Отрисовка линий на изображении
    output = cv::Mat::zeros(input.size(), CV_8UC3);
    int y_offset = output.rows * 0.6;
    for (const auto &line : lines) {
        // Определение направления линии
        double angle = std::atan2(line[3] - line[1], line[2] - line[0]) * 180.0 / CV_PI;

        // Отфильтровать линии на основе угла и положения
        if (std::abs(angle) > 30.0 && std::abs(angle) < 150.0 && line[1] > y_offset && line[3] > y_offset) {
            cv::line(output, cv::Point(line[0], line[1]), cv::Point(line[2], line[3]), cv::Scalar(0, 0, 255), 2);
        }
    }
}


// Переключение между камерами
void MainWindow::toggleCamera() {
    if (!cap.isOpened()) {
        QString fileName = QFileDialog::getOpenFileName(this, tr("Выберите видеофайл"), "", tr("Видеофайлы (*.avi *.mp4);;Все файлы (*)"));

        if (fileName.isEmpty()) {
            cap.open(0); // Открываем камеру (0 — индекс камеры)
            if (!cap.isOpened()) {
                qDebug() << "Error: Cannot open the camera.";
                return;
            }
            ui->pushButton->setText("Закрыть камеру");
        } else {
            cap.open(fileName.toStdString()); // Открываем видеофайл
            if (!cap.isOpened()) {
                qDebug() << "Error: Cannot open the video file.";
                return;
            }
            ui->pushButton->setText("Закрыть видео");
        }

        timer->start(30); // Задаем интервал обновления кадра (30 мс)
    } else {
        timer->stop(); // Останавливаем таймер
        cap.release(); // Закрываем камеру или видеофайл
        ui->label->clear(); // Очищаем содержимое метки
        ui->pushButton->setText("Открыть камеру / загрузить видео");
    }

}


// Слот для получения списка доступных COM-портов
void MainWindow::comPortList(QStringList portList) {
    ui -> comboBox_GNSS -> clear();
    for (QStringList::const_iterator it = portList.constBegin(); it != portList.constEnd(); ++it) {
        qDebug() << *it;
        ui -> comboBox_GNSS -> addItem(*it);
    }
}


// Слот для вывода координат GNSS (x, y, h)
void MainWindow::printCoordinate(double x, double y, double h)
{
    qDebug() << x << " " << y << " " << h;

}


// Слот для вывода скорости
void MainWindow::printSpeed(double speed) {
    int intSpeed = static_cast<int>(round(speed));
    QString speedText = QString::number(intSpeed) + " km/h";
    if (intSpeed > 60) {
        ui->label_speed->setStyleSheet("font-size: 32px;color: red;");
    } else {
        ui->label_speed->setStyleSheet("font-size: 32px; color: #FFFFFF;");
    }
    ui->label_speed->setText(speedText);
}


