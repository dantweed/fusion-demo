#ifndef SCATTERDATAMODIFIER_H
#define SCATTERDATAMODIFIER_H

#include <QtDataVisualization/q3dscatter.h>
#include <QtDataVisualization/qabstract3dseries.h>
#include <QtGui/QFont>

using namespace QtDataVisualization;

class ScatterDataModifier : public QObject
{
    Q_OBJECT
public:
    explicit ScatterDataModifier(Q3DScatter *scatter);
    ~ScatterDataModifier();

public Q_SLOTS:
    void addData(const QVector3D& vec, int series);

Q_SIGNALS:
    void dataAvailable(const QVector3D& vec, int series);

private:    
    Q3DScatter *m_graph;

};

#endif
