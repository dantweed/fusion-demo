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

const static double PRED_FWD_DELTA = 0.1; // Forward time step for prediction (sec)

const static double IMU_FREQ = 30;              //IMU data availability frequency (Hz)
const static double IMU_PERIOD = 1/IMU_FREQ;    // IMU data availability period (sec)
const static double IMU_OFFSET = 0.5*IMU_PERIOD;// Offset between reporting of each device (sec)

const static double POS_LAG = 0.01; //Camera position reporting lag (sec)
const static double ALPHA = 0.2; //Constant used in complimentary filter for X position in C's reference frame

const std::string IMU_DATA_FILE = "C:\\Users\\mdani\\Documents\\GitHub\\wrnch-qt\\smTestData.csv";

//Some forward declarations
std::array<double,3> rotate3DVector(const etk::Matrix<3,3>& cRotation,  std::array<double,3> vector);
std::array<double,3> transformPosToRefFrame (const etk::Matrix<3,3>& rotation, std::array<double,3> position,
                                             std::array<double,3> translation);
void initQObjects( Q3DScatter& graph,  QWidget& container);
void setupSeries(Q3DScatter& graph);
std::vector<std::array<std::array<double,3>,3>> getIMUData();

int main(int argc, char**argv) {

    Device X;
    Camera C;
    DeviceState xState(X, IMU_PERIOD);
    DeviceState cState(C, IMU_PERIOD);   

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

    for (auto dataSet : data) {
        std::array<double,3> xPosInCFrame;
        std::array<double,3> xEstPosInCFrame;
        etk::Matrix<3,3> cRotation;
        std::array<double,3> xVelInC;
        std::array<double,3> xFuturePosInCFrame;
        //X position in C's reference fram at PRED_FWD_DELTA is based on current estimate of X position in C
        // plus velocity of X in C's reference frame time PRED_FWD_DELTA
        //Current rotation matrix between X and C reference frames from C orientation quaternion
        cState.updateFromIMU(dataSet);
        xState.updateFromIMU(dataSet);

        cRotation = cState.getEstOrientQ().toMatrix();

        //Rotate X's velocity into C's reference frame
        xVelInC = rotate3DVector(cRotation, xState.getEstVel());
    {using namespace wrnch; //For std::array arithmetic. Operator overload shadows overloaded operators in QtDataVisualization

        //Update translation from C to X reference frame        
        currTranslation = initFrameTranslation + cState.getEstPos();

        //Get current position of X in C's reference frame
        xPosInCFrame = transformPosToRefFrame (cRotation, xState.getEstPos(), currTranslation);

        //If available, adjust with C's 3D position data reported for X
        C.setTrackingData(xPosInCFrame);//TODO: Plus noise, cannot access std::normal_distribution

        if (C.isInField())
            xEstPosInCFrame = ALPHA * xPosInCFrame + (1 - ALPHA) * C.getTrackingData();
        else //Else use IMU data only
            xEstPosInCFrame = xPosInCFrame;

        //Predict future position of X
        xFuturePosInCFrame = xEstPosInCFrame + xVelInC * (PRED_FWD_DELTA);    }

        std::array<double,3> cPos = cState.getEstPos();
        //std::array<double,3> xPos = rotate3DVector(cRotation,xState.getEstPos());

        (cPosDataArrayPtr++)->setPosition(QVector3D(cPos[0],cPos[1],cPos[2]));
        (xPosDataArrayPtr++)->setPosition(QVector3D(xPosInCFrame[0],xPosInCFrame[1],xPosInCFrame[2]));
        (xPredictedPosPtr++)->setPosition(QVector3D(xFuturePosInCFrame[0],xFuturePosInCFrame[1],xFuturePosInCFrame[2]));
    }
    graph->seriesList().at(0)->dataProxy()->resetArray(cPosDataArray); //Add C position data to plot
    graph->seriesList().at(1)->dataProxy()->resetArray(xPosDataArray); //Add X position data
    graph->seriesList().at(2)->dataProxy()->resetArray(xPredictedPos); //Add C's predicted position of X

    widget->show();
    return app.exec();
}

/** Transforms the orientation of a vector from one frame of reference to another
 *   as defined by a rotation matrix
 * @param Rotation matrix
 * @param Vector to be rotated
 * @return Rotated vector
 */
std::array<double,3> rotate3DVector(const etk::Matrix<3,3>& rotation,  std::array<double,3> vector){
    using namespace wrnch;
    std::array<double,3> rotated = {0,0,0};

    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            rotated[i] = rotated[i] + rotation.cell(i,j)*vector[j];
    return rotated;
}

/** Applies rotation and translation to express a vector position in another
 *   frame of reference
 * @param Frame rotation matrix
 * @param Position to be transformed
 * @param Frame translation vector
 * @return
 */
std::array<double,3> transformPosToRefFrame (const etk::Matrix<3,3>& rotation, std::array<double,3> position,
                                             std::array<double,3> translation){
    using namespace wrnch;
    std::array<double,3> rotated = rotate3DVector(rotation, position);

    // Notation: postion of a in b frame: p_a,b
    // p_x,c = p_x,x*rotation_to_x + translation_to_x
    return rotated + translation;
}

/** Helper function to initialize various QObjects */
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
   /* graph.axisX()->setMax(3);
    graph.axisY()->setMax(3);
    graph.axisZ()->setMax(3);
    graph.axisX()->setMin(-3);
    graph.axisY()->setMin(-3);
    graph.axisZ()->setMin(-3);*/
    graph.axisX()->setTitle("X");
    graph.axisY()->setTitle("Y");
    graph.axisZ()->setTitle("Z");
    QFont serifFont("Times", 10, QFont::Bold);
    serifFont.setPointSizeF(40.0f);
    graph.activeTheme()->setFont(serifFont);
    graph.scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);
}

/** Helper function to setup required data series for plotting */
void setupSeries(Q3DScatter& graph) {
    QScatterDataProxy *cDataProxy = new QScatterDataProxy;
    QScatter3DSeries *cDataSeries = new QScatter3DSeries(cDataProxy);
    cDataSeries->setItemLabelFormat(QStringLiteral("(@seriesName) @xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    cDataSeries->setMesh(QAbstract3DSeries::MeshBevelCube);
    cDataSeries->setItemSize(0.1f);
    cDataSeries->setBaseColor(QColor("black"));
    cDataSeries->setItemLabelVisible(true);
    cDataSeries->setName(QStringLiteral("C"));
    graph.addSeries(cDataSeries);

    QScatterDataProxy *xDataProxy = new QScatterDataProxy;
    QScatter3DSeries *xDataSeries = new QScatter3DSeries(xDataProxy);
    xDataSeries->setItemLabelFormat(QStringLiteral("(@seriesName) @xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    xDataSeries->setMesh(QAbstract3DSeries::MeshPyramid);
    xDataSeries->setItemSize(0.1f);
    xDataSeries->setBaseColor(QColor("blue"));
    xDataSeries->setItemLabelVisible(true);
    xDataSeries->setName(QStringLiteral("X"));

    graph.addSeries(xDataSeries);

    QScatterDataProxy *xPredictedDataProxy = new QScatterDataProxy;
    QScatter3DSeries *xPredictedDataSeries = new QScatter3DSeries(xPredictedDataProxy);
    xPredictedDataSeries->setItemLabelFormat(QStringLiteral("(@seriesName) @xTitle: @xLabel @yTitle: @yLabel @zTitle: @zLabel"));
    xPredictedDataSeries->setMesh(QAbstract3DSeries::MeshSphere);
    xPredictedDataSeries->setItemSize(0.1f);
    xPredictedDataSeries->setBaseColor(QColor("red"));
    xPredictedDataSeries->setItemLabelVisible(true);
    xPredictedDataSeries->setName(QStringLiteral("X\'"));

    graph.addSeries(xPredictedDataSeries);
}

/** Helper function to process CSV formatted IMU data for offline processing */
std::vector<std::array<std::array<double,3>,3>> getIMUData() {
    std::vector<std::array<std::array<double,3>,3>> imuData;

    try {
        std::string line;
        std::ifstream fileIn(IMU_DATA_FILE);

        while(fileIn.good() && std::getline(fileIn, line) ) {
            if (line.empty())
                continue;
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
