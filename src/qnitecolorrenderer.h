#ifndef QNITECOLORRENDERER_H
#define QNITECOLORRENDERER_H

#include <QQuickPaintedItem>

#include <OpenNI.h>
#include <NiTE.h>

class QNiTE;

class QNiTEColorRenderer : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(QObject* kinect READ kinect WRITE setKinect NOTIFY kinectChanged)
    Q_PROPERTY(bool initialized READ initialized NOTIFY initializedChanged)

public:
    QNiTEColorRenderer(QQuickItem * parent = 0);
    ~QNiTEColorRenderer();

    virtual void paint( QPainter * );

    QObject* kinect() const
    {
        return m_kinect;
    }

    bool initialized() const
    {
        return m_initialized;
    }

signals:

    void kinectChanged(QObject* arg);

    void initializedChanged(bool arg);

public slots:

void setKinect(QObject* arg)
{
    if (m_kinect == arg)
        return;

    m_kinect = arg;
    emit kinectChanged(arg);
}

void initialize();

void onNewFrame();

private:
QObject* m_kinect;
QNiTE *m_qnite;

bool m_initialized;


};

#endif // QNITECOLORRENDERER_H
