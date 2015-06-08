#ifndef QNiTETrackerRendererTRACKERRENDERER_H
#define QNiTETrackerRendererTRACKERRENDERER_H

#include <OpenNI.h>
#include <NiTE.h>

#include <QQuickPaintedItem>

#define MAX_DEPTH 10000

class QNiTE;

class QNiTETrackerRenderer : public QQuickPaintedItem
{
    Q_OBJECT
    Q_PROPERTY(QObject* kinect READ kinect WRITE setKinect NOTIFY kinectChanged)
    Q_PROPERTY(bool initialized READ initialized WRITE setInitialized NOTIFY initializedChanged)
    Q_PROPERTY(bool keepHistogram READ keepHistogram WRITE setKeepHistogram NOTIFY keepHistogramChanged)
public:
    explicit QNiTETrackerRenderer(QQuickItem *parent = 0);
    ~QNiTETrackerRenderer();

    bool initialized() const
    {
        return m_initialized;
    }

    virtual void paint(QPainter *painter);

    QObject* kinect() const
    {
        return m_kinect;
    }

    bool keepHistogram() const
    {
        return m_keepHistogram;
    }

signals:

    void initializedChanged(bool arg);
    void newFrameAvailable();

    void kinectChanged(QObject* arg);

    void keepHistogramChanged(bool arg);

public slots:

    void initialize();
    void onNewFrame();
    void setInitialized(bool arg)
    {
        if (m_initialized == arg)
            return;

        m_initialized = arg;
        emit initializedChanged(arg);
    }

    void setKinect(QObject* arg)
    {
        if (m_kinect == arg)
            return;

        m_kinect = arg;
        emit kinectChanged(arg);
    }

    void setKeepHistogram(bool arg)
    {
        if (m_keepHistogram == arg)
            return;

        m_keepHistogram = arg;
        emit keepHistogramChanged(arg);
    }

private:
    void DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData, QPainter *painter);
    void DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, QPainter *painter);

    QNiTE *m_qnite;

    openni::Device * m_device;
    nite::UserTracker * m_userTracker;
    bool m_initialized;

    int m_nTexMapX;
    int m_nTexMapY;
    int g_nXRes;
    int g_nYRes;
    openni::RGB888Pixel * m_pTexMap;

    float m_pDepthHist[MAX_DEPTH];
    bool m_hasHistogram;
    QObject* m_kinect;
    bool m_keepHistogram;
};

#endif // QNiTETrackerRendererTRACKERRENDERER_H
