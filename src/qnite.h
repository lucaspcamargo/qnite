#ifndef QNITE_H
#define QNITE_H

#include <OpenNI.h>
#include <NiTE.h>

#include <QObject>
#include <QVector3D>
#include <QMutex>
#include <QMap>
#include <QElapsedTimer>

#define MAX_DEPTH 10000

class QNiTEUser;

class QNiTE : public QObject, public nite::UserTracker::NewFrameListener, public openni::VideoStream::NewFrameListener
{
    Q_OBJECT
    Q_PROPERTY(bool initialized READ initialized WRITE setInitialized NOTIFY initializedChanged)
    Q_PROPERTY(int userCount READ userCount WRITE setUserCount NOTIFY userCountChanged)
    Q_PROPERTY(int frameIndex READ frameIndex WRITE setFrameIndex NOTIFY frameIndexChanged)
    Q_PROPERTY(int skeletonCount READ skeletonCount WRITE setSkeletonCount NOTIFY skeletonCountChanged)
    Q_PROPERTY(QVector3D groundPoint READ groundPoint WRITE setGroundPoint NOTIFY groundPointChanged)
    Q_PROPERTY(QVector3D groundNormal READ groundNormal WRITE setGroundNormal NOTIFY groundNormalChanged)
    Q_PROPERTY(qreal groundConfidence READ groundConfidence WRITE setGroundConfidence NOTIFY groundConfidenceChanged)
    Q_PROPERTY(bool rgbStreamEnabled READ rgbStreamEnabled WRITE setRgbStreamEnabled NOTIFY rgbStreamEnabledChanged)

public:
    explicit QNiTE(QObject *parent = 0);
    ~QNiTE();

    bool initialized() const
    {
        return m_initialized;
    }

    virtual void onNewFrame(nite::UserTracker&);
    virtual void onNewFrame(openni::VideoStream&);


    int userCount() const
    {
        return m_userCount;
    }

    int frameIndex() const
    {
        return m_frameIndex;
    }

    bool rgbStreamEnabled() const
    {
        return m_rgbStreamEnabled;
    }

    int skeletonCount() const
    {
        return m_skeletonCount;
    }

    QVector3D groundPoint() const
    {
        return m_groundPoint;
    }

    QVector3D groundNormal() const
    {
        return m_groundNormal;
    }

    qreal groundConfidence() const
    {
        return m_groundConfidence;
    }

signals:

    void newTrackerFrame();
    void newRGBFrame();

    void initializedChanged(bool arg);
    void userCountChanged(int arg);
    void frameIndexChanged(int arg);

    void rgbStreamEnabledChanged(bool arg);

    void skeletonCountChanged(int arg);

    void userFound( int id );
    void userLost( int id );

    void groundPointChanged(QVector3D arg);

    void groundNormalChanged(QVector3D arg);

    void groundConfidenceChanged(qreal arg);

public slots:

    void initialize();
    void setInitialized(bool arg)
    {
        if (m_initialized == arg)
            return;

        m_initialized = arg;
        emit initializedChanged(arg);
    }

    void setUserCount(int arg)
    {
        if (m_userCount == arg)
            return;

        m_userCount = arg;
        emit userCountChanged(arg);
    }

    void setFrameIndex(int arg)
    {
        if (m_frameIndex == arg)
            return;

        m_frameIndex = arg;
        emit frameIndexChanged(arg);
    }


    nite::UserTrackerFrameRef * getFrameRef()
    {
        return &m_frameRef;
    }

    void lockFrameRef()
    {
        m_frameRefMutex.lock();
    }

    void unlockFrameRef()
    {
        m_frameRefMutex.unlock();
    }

    void lockRGBFrameRef()
    {
        m_rgbFrameRefMutex.lock();
    }

    void unlockRGBFrameRef()
    {
        m_rgbFrameRefMutex.unlock();
    }

    void processNewFrame();

    void processNewRGBFrame();


    QVector3D toScreenSpace(QVector3D point);

    void setRgbStreamEnabled(bool arg)
    {
        if (m_rgbStreamEnabled == arg)
            return;

        m_rgbStreamEnabled = arg;
        emit rgbStreamEnabledChanged(arg);

        if(arg)
            m_rgbStream->start();
        else
            m_rgbStream->stop();
    }

    void setSkeletonCount(int arg)
    {
        if (m_skeletonCount == arg)
            return;

        m_skeletonCount = arg;
        emit skeletonCountChanged(arg);
    }

    void utilTrimEngineComponentCache();
    void utilStartTimer();
    quint64 utilGetElapsedNanos(bool restartAfter);

    QNiTEUser * getUser(int id)
    {
        return m_users.value(id, 0);
    }

    QNiTEUser * getUserByIndex(int index)
    {
        return m_users.values().value(index);
    }

    void setGroundPoint(QVector3D arg)
    {
        if (m_groundPoint == arg)
            return;

        m_groundPoint = arg;
        emit groundPointChanged(arg);
    }

    void setGroundNormal(QVector3D arg)
    {
        if (m_groundNormal == arg)
            return;

        m_groundNormal = arg;
        emit groundNormalChanged(arg);
    }

    void setGroundConfidence(qreal arg)
    {
        if (m_groundConfidence == arg)
            return;

        m_groundConfidence = arg;
        emit groundConfidenceChanged(arg);
    }

private:
    friend class QNiTETrackerRenderer;
    friend class QNiTEColorRenderer;

    openni::Device * m_device;
    openni::VideoStream * m_rgbStream;
    openni::VideoFrameRef m_rgbFrameRef;
    QMutex m_rgbFrameRefMutex;

    nite::UserTracker * m_userTracker;
    nite::UserTrackerFrameRef m_frameRef;
    QMutex m_frameRefMutex;
    bool m_initialized;

    int m_userCount;
    int m_frameIndex;
    bool m_rgbStreamEnabled;
    int m_skeletonCount;

    QMap<int, QNiTEUser *> m_users;

    bool m_shutdown;
    QVector3D m_groundPoint;
    QVector3D m_groundNormal;
    qreal m_groundConfidence;

    QElapsedTimer m_timer;
};

#endif // QNITE_H
