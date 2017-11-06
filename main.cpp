#include <iostream>
#include <array>
#include <fstream>
#include <thread>
#include <string>
#include <sstream>

#include "wrnch/Device.h"
#include "wrnch/DeviceState.h"
#include "wrnch/Camera.h"
#include "wrnch/IMU.h"
#include "wrnch/MDKalman.h"
#include "wrnch/etk/etk.h"
#include "wrnch/ArrayArithmetic.h"

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
#include <QtDataVisualization/q3dscatter.h>
#include <QtDataVisualization/qabstract3dseries.h>
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>
#include <QtWidgets/QComboBox>
#include <QtGui/QFont>

using namespace QtDataVisualization;

const static double IMU_FREQ = 30; //IMU data availability frequency (Hz)
const static double IMU_PERIOD = 1/IMU_FREQ; // IMU data availability period (sec)
const static double IMU_OFFSET = 0.5*IMU_PERIOD; // Offset between reporting of each device (sec)

const static double C_IMU_LAG = IMU_PERIOD/10; //Lag between IMU sense and reporting
const static double X_IMU_LAG = IMU_PERIOD/10; //Lag between IMU sense and reporting
const static double POS_LAG = 0.01; //Camera position reporting lag (sec)

const std::string IMU_DATA_FILE = "C:\\Users\\daniel.tweed\\Documents\\GitHub\\wrnch-qt\\smTestDataC.csv";
const static double PRED_FWD_DELTA = 0.2; // Forward time step for prediction (sec)
const static double ALPHA = 0.6; //Constant used in complimentary filter for X position in C's reference frame

std::array<double,3> rotate3DVector(const etk::Matrix<3,3>& cRotation,  std::array<double,3> vector);
std::array<double,3> transformPosToRefFrame (const etk::Matrix<3,3>& rotation, std::array<double,3> position,
                                             std::array<double,3> translation);

void initQObjects( Q3DScatter& graph,  QWidget& container);
void setupSeries(Q3DScatter& graph);

std::vector<std::array<std::array<double,3>,3>> getIMUData() {
    std::vector<std::array<std::array<double,3>,3>> imuData;

    try {
        std::string line;
        std::ifstream fileIn(IMU_DATA_FILE);

        while(fileIn.good() && std::getline(fileIn, line) ) {
            std::vector<double> tokens;
            std::string token;
            std::array<std::array<double,3>,3> reading;
            std::istringstream iss(line);
            while (std::getline(iss, token, ',')){
                tokens.push_back(std::stod(token));
            }
            std::vector<double>::iterator it = tokens.begin();
            reading[0] = {*it++, *it++,*it++};
            reading[1] = {*it++, *it++,*it++};
            reading[2] = {*it++, *it++,*it};
            imuData.push_back(reading);
        }
    } catch (std::ifstream::failure &err) {
        throw err;
    }

    return imuData;
}

int main(int argc, char**argv) {

    Device X;
    Camera C;
    DeviceState xState(X, IMU_PERIOD);
    DeviceState cState(C, IMU_PERIOD);

    std::array<double,3> xPosInCFrame;
    std::array<double,3> xEstPosInCFrame;
    etk::Matrix<3,3> cRotation;
    std::array<double,3> xVelInC;
    std::array<double,3> xFuturePosInC;  

    QApplication app(argc, argv);
    Q3DScatter *graph = new Q3DScatter();
    QWidget *container = QWidget::createWindowContainer(graph);
    if (!graph->hasContext()) {
        QMessageBox msgBox;
        msgBox.setText("Couldn't initialize the OpenGL context.");
        msgBox.exec();
        return -1;
    }

    QWidget *widget = new QWidget;
    QHBoxLayout *hLayout = new QHBoxLayout(widget);
    QVBoxLayout *vLayout = new QVBoxLayout();
    hLayout->addWidget(container, 1);
    hLayout->addLayout(vLayout);    
    widget->setWindowTitle(QStringLiteral("Object Tracking and Prediction"));

    initQObjects(*graph, *container);
    setupSeries(*graph);

    std::vector<std::array<std::array<double,3>,3>> data;
    try {
        data = getIMUData();
    } catch (std::ifstream::failure &err) {
        return -1;
    }

    QScatterDataArray *cPosDataArray = new QScatterDataArray;
    cPosDataArray->resize(data.size()+1);
    QScatterDataItem *cPosDataArrayPtr = &cPosDataArray->first();
    cPosDataArrayPtr->setPosition(QVector3D(0,0,0));

    QScatterDataArray *xPosDataArray = new QScatterDataArray;
    xPosDataArray->resize(data.size()+1);
    QScatterDataItem *xPosDataArrayPtr = &xPosDataArray->first();
    xPosDataArrayPtr->setPosition(QVector3D(2,0,0));

    QScatterDataArray *xPredictedPos = new QScatterDataArray;
    xPredictedPos->resize(data.size()+1);
    QScatterDataItem *xPredictedPosPtr = &xPredictedPos->first();
    xPredictedPosPtr->setPosition(QVector3D(2,0,0));

    //Assumes C is initially aligned with global reference frame
    // then adjusts based on C's movement
    std::array<double,3> initFrameTranslation = {2,0,0}; //Asummed initial translation between X and C
    std::array<double,3> currTranslation = initFrameTranslation;
    C.setTrackingData(initFrameTranslation);

std::cout << data.size() << std::endl;
    for (auto dataSet : data) {

        //X position in C's reference fram at PRED_FWD_DELTA is based on current estimate of X position in C
        // plus velocity of X in C's reference frame time PRED_FWD_DELTA
        //Current rotation matrix between X and C reference frames from C orientation quaternion
        //cState.updateFromIMU(dataSet);
        xState.updateFromIMU(dataSet);

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
            C.setTrackingData(xPosInCFrame);//Plus noise
            if (C.isInField())
                xEstPosInCFrame = ALPHA * xPosInCFrame + (1 - ALPHA) * C.getTrackingData();
            else //Else use IMU data
                xEstPosInCFrame = xPosInCFrame;
        //Predict future position of X at PRED_FWD_DELTA

            xFuturePosInC = xEstPosInCFrame + xVelInC * (PRED_FWD_DELTA);
        }

        std::array<double,3> cPos = cState.getEstPos();
        //std::array<double,3> xPos = rotate3DVector(cRotation,xState.getEstPos());

        (cPosDataArrayPtr++)->setPosition(QVector3D(cPos[0],cPos[1],cPos[2]));
        (xPosDataArrayPtr++)->setPosition(QVector3D(xPosInCFrame[0],xPosInCFrame[1],xPosInCFrame[2]));
        (xPredictedPosPtr++)->setPosition(QVector3D(xFuturePosInC[0],xFuturePosInC[1],xFuturePosInC[2]));
    }
    graph->seriesList().at(0)->dataProxy()->resetArray(cPosDataArray);
    graph->seriesList().at(1)->dataProxy()->resetArray(xPosDataArray);
    graph->seriesList().at(2)->dataProxy()->resetArray(xPredictedPos);

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

void initQObjects( Q3DScatter& graph,  QWidget& container) {
    QSize screenSize = graph.screen()->size();//graph->screen()->size();
    container.setMinimumSize(QSize(screenSize.width() / 2, screenSize.height() / 1.5));
    container.setMaximumSize(screenSize);
    container.setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    container.setFocusPolicy(Qt::StrongFocus);

    graph.activeTheme()->setType(Q3DTheme::ThemeQt);
    graph.activeTheme()->setGridEnabled(true);
    graph.activeTheme()->setBackgroundEnabled(true);
    graph.activeTheme()->setLabelBackgroundEnabled(false);
    graph.axisX()->setMax(100);
    graph.axisY()->setMax(100);
    graph.axisZ()->setMax(100);
    graph.axisX()->setMin(-10);
    graph.axisY()->setMin(-10);
    graph.axisZ()->setMin(-10);
    graph.axisX()->setTitle("X");
    graph.axisY()->setTitle("Y");
    graph.axisZ()->setTitle("Z");
    QFont serifFont("Times", 10, QFont::Bold);
    serifFont.setPointSizeF(40.0f);
    graph.activeTheme()->setFont(serifFont);
    graph.scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);
}

void setupSeries(Q3DScatter& graph) {
    QScatterDataProxy *cDataProxy = new QScatterDataProxy;
    QScatter3DSeries *cDataSeries = new QScatter3DSeries(cDataProxy);
    cDataSeries->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    cDataSeries->setMesh(QAbstract3DSeries::MeshBevelCube);
    cDataSeries->setItemSize(0.1f);
    cDataSeries->setBaseColor(QColor("black"));
    cDataSeries->setItemLabelVisible(true);
    cDataSeries->setName(QStringLiteral("C"));

    graph.addSeries(cDataSeries);

    QScatterDataProxy *xDataProxy = new QScatterDataProxy;
    QScatter3DSeries *xDataSeries = new QScatter3DSeries(xDataProxy);
    xDataSeries->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    xDataSeries->setMesh(QAbstract3DSeries::MeshPyramid);
    xDataSeries->setItemSize(0.1f);
    xDataSeries->setBaseColor(QColor("blue"));
    xDataSeries->setItemLabelVisible(true);
    xDataSeries->setName(QStringLiteral("X"));

    graph.addSeries(xDataSeries);

    QScatterDataProxy *xPredDataProxy = new QScatterDataProxy;
    QScatter3DSeries *xPredDataSeries = new QScatter3DSeries(xPredDataProxy);
    xPredDataSeries->setItemLabelFormat(QStringLiteral("@xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    xPredDataSeries->setMesh(QAbstract3DSeries::MeshSphere);
    xPredDataSeries->setItemSize(0.1f);
    xPredDataSeries->setBaseColor(QColor("red"));
    xDataSeries->setItemLabelVisible(true);
    xDataSeries->setName(QStringLiteral("X\'"));

    graph.addSeries(xPredDataSeries);
}
