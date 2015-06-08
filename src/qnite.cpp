#include "qnite.h"
#include <QPainter>
#include <QtMath>

#include "qthread.h"
#include "QMetaMethod"

#include "qniteuser.h"

QNiTE::QNiTE(QObject *parent) : QObject(parent)
{
    m_initialized = false;

    m_device = 0;
    m_rgbStream = 0;
    m_userTracker = 0;

    m_userCount = m_skeletonCount = 0;
    m_frameIndex = 0;
    m_groundConfidence = 0.0;
    m_groundPoint = m_groundNormal = QVector3D();

    m_rgbStreamEnabled = true;

    m_shutdown = false;
}

void QNiTE::initialize()
{
    if(m_initialized) return;

    qDebug("[QNiTE] Initializing...");
    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK)
    {
        printf("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
        exit(-1);
    }

    m_device = new openni::Device();
    rc = m_device->open(0);
    if (rc != openni::STATUS_OK)
    {
        printf("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
        exit(-2);
    }
    m_device->setDepthColorSyncEnabled(true);

    m_rgbStream = new openni::VideoStream();
    openni::Status rcRGB = m_rgbStream->create(*m_device, openni::SENSOR_COLOR);
    if (rcRGB != openni::STATUS_OK)
    {
        printf("Failed to initialize RGB camera\n%s\n", openni::OpenNI::getExtendedError());
        exit(-1);
    }

    m_rgbStream->addNewFrameListener(this);

    if(m_rgbStreamEnabled)
        m_rgbStream->start();

    nite::NiTE::initialize();

    m_userTracker = new nite::UserTracker();
    if (m_userTracker->create(m_device) != nite::STATUS_OK)
    {
        qDebug("Failed to init user tracker");
    }

    m_userTracker->addNewFrameListener(this);



    setInitialized(true);

    QThread::currentThread()->setObjectName("Main Thread");
}

void QNiTE::processNewFrame()
{
    lockFrameRef();

    if(!m_frameRef.isValid())
    {
        qDebug("[QNiTE::processNewFrame] Frame is not valid.");
        unlockFrameRef();
        return;
    }

    setFrameIndex(m_frameRef.getFrameIndex());

    const nite::Array<nite::UserData>& users = m_frameRef.getUsers();
    setUserCount(users.getSize());
    int skeletons = 0;

    for (int i = 0; i < m_userCount; ++i)
    {
        const nite::UserData& user = users[i];
        QNiTEUser * userWrapper = 0;

        if (user.isLost())
        {
            if(m_users.contains(user.getId()))
            {
                emit userLost(user.getId());
                m_users.remove(user.getId());
                delete userWrapper;
                userWrapper = 0;
            }
        }
        else
        {

            if(user.getSkeleton().getState() == nite::SKELETON_TRACKED)
            {
                skeletons++;
            }

            userWrapper = getUser(user.getId());

            if(user.isNew())
                m_userTracker->startSkeletonTracking(user.getId());

            if(!userWrapper)
            {
                userWrapper = new QNiTEUser(user.getId(), this);
                m_users.insert(user.getId(), userWrapper);
                m_userTracker->startSkeletonTracking(user.getId());
                emit userFound(user.getId());
            }

            userWrapper->update(user);
        }

    }

    setSkeletonCount(skeletons);

    const nite::Plane & ground = m_frameRef.getFloor();

    setGroundNormal(QVector3D(ground.normal.x, ground.normal.y, ground.normal.z));
    setGroundPoint(QVector3D(ground.point.x, ground.point.y, ground.point.z));
    setGroundConfidence(m_frameRef.getFloorConfidence());

    m_frameRef.release();

    unlockFrameRef();

    emit newTrackerFrame();
}

void QNiTE::processNewRGBFrame()
{
    emit newRGBFrame();
}

QVector3D QNiTE::toScreenSpace(QVector3D point)
{
    float x, y;
    m_userTracker->convertJointCoordinatesToDepth(point.x(), point.y(), point.z(), &x, &y);
    return QVector3D(x, y, point.z());
}

QNiTE::~QNiTE()
{
    qDebug("[QNiTE] Cleaning house...");

    lockFrameRef();
    lockRGBFrameRef();

    if(m_userTracker) m_userTracker->removeNewFrameListener(this);
    if(m_rgbStream) m_rgbStream->removeNewFrameListener(this);

    m_shutdown = true;

    nite::NiTE::shutdown();
    openni::OpenNI::shutdown();

    unlockFrameRef();
    unlockRGBFrameRef();
}

// user tracker frame
void QNiTE::onNewFrame(nite::UserTracker & tracker)
{
    Q_UNUSED(tracker)

    lockFrameRef();

    if(m_shutdown) return;

    if(m_frameRef.isValid())
        qDebug("[QNiTE::onNewFrame] Overwriting unprocessed user tracker frame");

    nite::Status rc = m_userTracker->readFrame(&m_frameRef);
    if (rc != nite::STATUS_OK)
    {
        qDebug("[QNiTE::onNewFrame] Getting tracker frame failed");
        return;
    }

    unlockFrameRef();

    QMetaObject::invokeMethod(this, "processNewFrame", Qt::QueuedConnection);

}

// rgb frame
void QNiTE::onNewFrame(openni::VideoStream & stream)
{

    lockRGBFrameRef();

    openni::Status rc = m_rgbStream->readFrame(&m_rgbFrameRef);
    if (rc != openni::STATUS_OK)
    {
        printf("getting rgb frame failed on core\n");
        return;
    }

    unlockRGBFrameRef();

    QMetaObject::invokeMethod(this, "processNewRGBFrame", Qt::QueuedConnection);
}

#include <QtQml>
#include <QQmlEngine>

void QNiTE::utilTrimEngineComponentCache()
{
    qmlEngine(this)->trimComponentCache();
}

void QNiTE::utilStartTimer()
{
    m_timer.start();
}

quint64 QNiTE::utilGetElapsedNanos(bool restartAfter)
{
    quint64 ret = m_timer.nsecsElapsed();
    if(restartAfter) m_timer.restart();
    return ret;
}
