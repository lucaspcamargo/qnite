#ifndef QNITEUSER_H
#define QNITEUSER_H

#include <QObject>
#include <QVector3D>
#include <QMap>
#include <NiTE.h>

class QNiTEUser : public QObject
{
    Q_OBJECT
    Q_ENUMS(Joint)

    Q_PROPERTY(bool hasSkeleton READ hasSkeleton WRITE setHasSkeleton NOTIFY hasSkeletonChanged)
    Q_PROPERTY(int userId READ userId CONSTANT)
    Q_PROPERTY(QVector3D centerOfMass READ centerOfMass WRITE setCenterOfMass NOTIFY centerOfMassChanged)
    Q_PROPERTY(QVector3D boundingMin READ boundingMin WRITE setBoundingMin NOTIFY boundingMinChanged)
    Q_PROPERTY(QVector3D boundingMax READ boundingMax WRITE setBoundingMax NOTIFY boundingMaxChanged)

public:
    enum Joint {
        J_HEAD = nite::JOINT_HEAD,
        J_NECK = nite::JOINT_NECK,

        J_LEFT_SHOULDER = nite::JOINT_LEFT_SHOULDER,
        J_RIGHT_SHOULDER = nite::JOINT_RIGHT_SHOULDER,
        J_LEFT_ELBOW = nite::JOINT_LEFT_ELBOW,
        J_RIGHT_ELBOW = nite::JOINT_RIGHT_ELBOW,
        J_LEFT_HAND = nite::JOINT_LEFT_HAND,
        J_RIGHT_HAND = nite::JOINT_RIGHT_HAND,

        J_TORSO = nite::JOINT_TORSO,

        J_LEFT_HIP = nite::JOINT_LEFT_HIP,
        J_RIGHT_HIP = nite::JOINT_RIGHT_HIP,
        J_LEFT_KNEE = nite::JOINT_LEFT_KNEE,
        J_RIGHT_KNEE = nite::JOINT_RIGHT_KNEE,
        J_LEFT_FOOT = nite::JOINT_LEFT_FOOT,
        J_RIGHT_FOOT = nite::JOINT_RIGHT_FOOT
    };

    explicit QNiTEUser( nite::UserId id, QObject *parent = 0 );
    ~QNiTEUser();

    bool hasSkeleton() const
    {
        return m_hasSkeleton;
    }

    QVector3D centerOfMass() const
    {
        return m_centerOfMass;
    }

    QVector3D boundingMin() const
    {
        return m_boundingMin;
    }

    QVector3D boundingMax() const
    {
        return m_boundingMax;
    }

    int userId() const
    {
        return m_userId;
    }

signals:

    void hasSkeletonChanged(bool arg);

    void centerOfMassChanged(QVector3D arg);

    void boundingMinChanged(QVector3D arg);

    void boundingMaxChanged(QVector3D arg);

    void updated();

public slots:

    void update(const nite::UserData &data);

    QVector3D jointPosition(Joint joint)
    {
        return m_skeletonPositions.value(joint, QVector3D());
    }

    qreal jointConfidence(Joint joint)
    {
        return m_skeletonConfidences.value(joint, 0.0);
    }

    void setHasSkeleton(bool arg)
    {
        if (m_hasSkeleton == arg)
            return;

        m_hasSkeleton = arg;
        emit hasSkeletonChanged(arg);
    }

    void setCenterOfMass(QVector3D arg)
    {
        if (m_centerOfMass == arg)
            return;

        m_centerOfMass = arg;
        emit centerOfMassChanged(arg);
    }

    void setBoundingMin(QVector3D arg)
    {
        if (m_boundingMin == arg)
            return;

        m_boundingMin = arg;
        emit boundingMinChanged(arg);
    }

    void setBoundingMax(QVector3D arg)
    {
        if (m_boundingMax == arg)
            return;

        m_boundingMax = arg;
        emit boundingMaxChanged(arg);
    }

private:
    void storeJoint(const nite::Skeleton  &skeleton, Joint joint);

    bool m_hasSkeleton;
    QVector3D m_centerOfMass;
    QVector3D m_boundingMin;
    QVector3D m_boundingMax;
    int m_userId;

    QMap< Joint, QVector3D > m_skeletonPositions;
    QMap< Joint, qreal > m_skeletonConfidences;

};

#endif // QNITEUSER_H
