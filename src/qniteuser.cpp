#include "qniteuser.h"

QNiTEUser::QNiTEUser(nite::UserId id, QObject *parent) : QObject(parent)
{
    m_userId = id;
}

QNiTEUser::~QNiTEUser()
{

}

void QNiTEUser::update(const nite::UserData &data)
{
    const nite::Skeleton  &skeleton = data.getSkeleton();
    setHasSkeleton(skeleton.getState() == nite::SKELETON_TRACKED);

    const NitePoint3f c = data.getCenterOfMass();
    setCenterOfMass(QVector3D(c.x, c.y, c.z));

    const NitePoint3f min = data.getBoundingBox().min;
    const NitePoint3f max = data.getBoundingBox().max;
    setBoundingMin( QVector3D(min.x, min.y, min.z) );
    setBoundingMax( QVector3D(max.x, max.y, max.z) );

    storeJoint(skeleton, J_HEAD);
    storeJoint(skeleton, J_NECK);

    storeJoint(skeleton, J_LEFT_SHOULDER);
    storeJoint(skeleton, J_RIGHT_SHOULDER);
    storeJoint(skeleton, J_LEFT_ELBOW);
    storeJoint(skeleton, J_RIGHT_ELBOW);
    storeJoint(skeleton, J_LEFT_HAND);
    storeJoint(skeleton, J_RIGHT_HAND);

    storeJoint(skeleton, J_TORSO);

    storeJoint(skeleton, J_LEFT_HIP);
    storeJoint(skeleton, J_RIGHT_HIP);
    storeJoint(skeleton, J_LEFT_KNEE);
    storeJoint(skeleton, J_RIGHT_KNEE);
    storeJoint(skeleton, J_LEFT_FOOT);
    storeJoint(skeleton, J_RIGHT_FOOT);

    emit updated();

}

void QNiTEUser::storeJoint(const nite::Skeleton &skeleton, QNiTEUser::Joint jointId)
{
    const nite::SkeletonJoint & joint = skeleton.getJoint( static_cast<nite::JointType>(jointId) );
    nite::Point3f pos = joint.getPosition();

    m_skeletonPositions[jointId] = QVector3D(pos.x, pos.y, pos.z);
    m_skeletonConfidences[jointId] = joint.getPositionConfidence();

}

