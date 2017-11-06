#include <iostream>
#include <windows.h>
#include <array>

#include <thread>

#include <chrono>
#include "wrnch/Device.h"
#include "wrnch/DeviceState.h"
#include "wrnch/Camera.h"
#include "wrnch/IMU.h"
#include "wrnch/MDKalman.h"
#include "wrnch/etk/etk.h"
#include "wrnch/ArrayArithmetic.h"

#include "scatterdatamodifier.h"

#include <QtWidgets/QApplication>
#include <QtWidgets/QWidget>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QVBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QComboBox>
#include <QtWidgets/QFontComboBox>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMessageBox>
#include <QtGui/QScreen>
#include <QtGui/QFontDatabase>


const static double IMU_FREQ = 30; //IMU data availability frequency (Hz)
const static double IMU_PERIOD = 1/IMU_FREQ; // IMU data availability period (sec)
const static double IMU_OFFSET = 0.5*IMU_PERIOD; // Offset between reporting of each device (sec)

const static double POS_LAG = 0.1; //Camera position reporting lag (sec)

const static double PRED_FWD_DELTA = 0.2; // Forward time step for prediction (sec)
const static double ALPHA = 0.6; //Constant used in complimentary filter for X position in C's reference frame

std::array<double,3> rotate3DVector(const etk::Matrix<3,3>& cRotation,  std::array<double,3> vector);
std::array<double,3> transformPosToRefFrame (const etk::Matrix<3,3>& rotation, std::array<double,3> position,
                                             std::array<double,3> translation);

int main(int argc, char**argv) {

    Device X;
    Camera C;
    DeviceState xState(X, IMU_PERIOD);
    DeviceState cState(C, IMU_PERIOD);

    std::array<double,3> xPosInCFrame;
    etk::Matrix<3,3> cRotation;
    std::array<double,3> xVelInC;
    std::array<double,3> xFuturePosInC;

    //Assumes C is initially aligned with global reference frame
    // then adjusts based on C's movement
    std::array<double,3> initFrameTranslation = {2,0,0}; //Asummed initial translation between X and C
    std::array<double,3> currTranslation = initFrameTranslation;
    C.setTrackingData(initFrameTranslation);

    //X position in C's reference fram at PRED_FWD_DELTA is based on current estimate of X position in C
    // plus velocity of X in C's reference frame time PRED_FWD_DELTA

    //Current rotation matrix between X and C reference frames from C orientation quaternion
    cRotation = cState.getEstOrientQ().toMatrix();

    //Rotate X's velocity into C's reference frame
    xVelInC = rotate3DVector(cRotation, xState.getEstVel());

    //Update translation from C to X reference frame
    {using namespace wrnch;
        currTranslation = initFrameTranslation + cState.getEstPos();
    }
    //Get current position of X in C's reference frame
    xPosInCFrame = transformPosToRefFrame (cRotation, xState.getEstPos(), currTranslation);

    //If available, adjust with C's 3D position data reported for X
    {using namespace wrnch;
        if (C.isInField())
            xPosInCFrame = ALPHA * xPosInCFrame + (1 - ALPHA) * C.getTrackingData();
    }
    //Predict future position of X at PRED_FWD_DELTA
    {using namespace wrnch;
        xFuturePosInC = xPosInCFrame + xVelInC * PRED_FWD_DELTA;
    }

    for ( auto a: xVelInC)
        std::cout << a << std::endl;

    QApplication app(argc, argv);
    Q3DScatter *graph = new Q3DScatter();
    QWidget *container = QWidget::createWindowContainer(graph);
    //! [0]

    if (!graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return -1;
    }
    QSize screenSize = graph->screen()->size();
    container->setMinimumSize(QSize(screenSize.width() / 2, screenSize.height() / 1.5));
    container->setMaximumSize(screenSize);
    container->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container->setFocusPolicy(Qt::StrongFocus);

    //! [1]
    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);
    //! [1]

    widget->setWindowTitle(QStringLiteral("Object Tracking and Prediction"));

    //! [2]
    ScatterDataModifier *modifier = new ScatterDataModifier(graph);
    //! [2]

    graph->activeTheme()->setType(Q3DTheme::ThemeQt);
    graph->activeTheme()->setGridEnabled(true);
    graph->activeTheme()->setBackgroundEnabled(true);
    graph->activeTheme()->setLabelBackgroundEnabled(false);
    graph->axisX()->setMax(4);
    graph->axisY()->setMax(4);
    graph->axisZ()->setMax(4);
    graph->axisX()->setMin(-3);
    graph->axisY()->setMin(-3);
    graph->axisZ()->setMin(-3);
    graph->axisX()->setTitle("X");
    graph->axisY()->setTitle("Y");
    graph->axisZ()->setTitle("Z");

    QFont serifFont("Times", 10, QFont::Bold);
    serifFont.setPointSizeF(40.0f);
    graph->activeTheme()->setFont(serifFont);

    QScatterDataProxy *proxy = new QScatterDataProxy;
    QScatter3DSeries *series = new QScatter3DSeries(proxy);
    series->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    series->setMesh(QAbstract3DSeries::MeshBevelCube);
    series->setItemSize(0.1f);
    series->setBaseColor(QColor("black"));

    graph->addSeries(series);

    QScatterDataProxy *proxy2 = new QScatterDataProxy;
    QScatter3DSeries *series2 = new QScatter3DSeries(proxy2);
    series2->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    series2->setMesh(QAbstract3DSeries::MeshPyramid);
    series2->setItemSize(0.1f);
    series2->setBaseColor(QColor("blue"));

    graph->addSeries(series2);

    QScatterDataProxy *proxy3 = new QScatterDataProxy;
    QScatter3DSeries *series3 = new QScatter3DSeries(proxy3);
    series3->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    series3->setMesh(QAbstract3DSeries::MeshCube);
    series3->setItemSize(0.1f);

    series3->setBaseColor(QColor("cyan"));

    graph->addSeries(series3);

    QScatterDataArray *dataArray = new QScatterDataArray;
    dataArray->resize(1);
    QScatterDataItem *ptrToDataArray = &dataArray->first();
    ptrToDataArray->setPosition(QVector3D(0,0,0));

    QScatterDataArray *dataArray2 = new QScatterDataArray;
    dataArray2->resize(1);
    QScatterDataItem *ptrToDataArray2 = &dataArray2->first();
    ptrToDataArray2->setPosition(QVector3D(2,0,0));

    QScatterDataArray *dataArray3 = new QScatterDataArray;
    dataArray3->resize(1);
    QScatterDataItem *ptrToDataArray3 = &dataArray3->first();
    ptrToDataArray3->setPosition(QVector3D(-2,0,0));

    graph->seriesList().at(0)->dataProxy()->resetArray(dataArray);
    graph->seriesList().at(1)->dataProxy()->resetArray(dataArray2);
    graph->seriesList().at(2)->dataProxy()->resetArray(dataArray3);
    modifier->addData(QVector3D(-2,0,1),2);

    widget->show();



    return app.exec();

}

std::array<double,3> rotate3DVector(const etk::Matrix<3,3>& rotation,  std::array<double,3> vector){
    using namespace wrnch;
    std::array<double,3> rotated = {0,0,0};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotated[i] = rotated[i] + rotation.cell(i,j)*vector[j];
    return rotated;
}

std::array<double,3> transformPosToRefFrame (const etk::Matrix<3,3>& rotation, std::array<double,3> position,
                                             std::array<double,3> translation){
    using namespace wrnch;
    std::array<double,3> rotated = rotate3DVector(rotation, position);

    // Notation: postion of a in b frame: p_a,b
    // p_x,c = p_x,x*rotation_to_x + translation_to_x
    return rotated + translation;
}
