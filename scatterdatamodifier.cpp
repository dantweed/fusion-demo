#include "scatterdatamodifier.h"
#include <QtDataVisualization/qscatterdataproxy.h>
#include <QtDataVisualization/qvalue3daxis.h>
#include <QtDataVisualization/q3dscene.h>
#include <QtDataVisualization/q3dcamera.h>
#include <QtDataVisualization/qscatter3dseries.h>
#include <QtDataVisualization/q3dtheme.h>
#include <QtCore/qmath.h>
#include <QtWidgets/QComboBox>

using namespace QtDataVisualization;

ScatterDataModifier::ScatterDataModifier(Q3DScatter *scatter)
    : m_graph(scatter)
{

      m_graph->scene()->activeCamera()->setCameraPreset(Q3DCamera::CameraPresetFront);

}

ScatterDataModifier::~ScatterDataModifier() {
    delete m_graph;
}

void ScatterDataModifier::addData(const QVector3D &vec, int series) {
    m_graph->seriesList().at(series)->dataProxy()->addItem(QScatterDataItem(vec));

}
