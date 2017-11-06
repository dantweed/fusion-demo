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

int main(int argc, char **argv)
{
    //! [0]
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
    graph->axisX()->setMax(3);
    graph->axisY()->setMax(3);
    graph->axisZ()->setMax(3);
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

    //! [3]
    widget->show();
    return app.exec();
    //! [3]
}
