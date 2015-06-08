#include "qnitetrackerrenderer.h"
#include <QPainter>
#include <QtMath>

#include "qnite.h"

QNiTETrackerRenderer::QNiTETrackerRenderer(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    m_initialized = false;

    m_device = 0;
    m_userTracker = 0;

    m_nTexMapX = m_nTexMapY = 0;
    m_pTexMap = 0;

    m_keepHistogram = m_hasHistogram = false;
}

void QNiTETrackerRenderer::initialize()
{
    if(m_initialized || !m_kinect) return;

    m_qnite = reinterpret_cast<QNiTE*>(m_kinect);
    m_userTracker = m_qnite->m_userTracker;

    connect(m_qnite, &QNiTE::newTrackerFrame, this, &QNiTETrackerRenderer::onNewFrame);

    setInitialized(true);

}

QNiTETrackerRenderer::~QNiTETrackerRenderer()
{
    qDebug("[QNiTETrackerRenderer] Cleaning house...");

    if( m_pTexMap ) delete[] m_pTexMap;
}

void QNiTETrackerRenderer::onNewFrame()
{
    emit newFrameAvailable();
}

void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
{
    const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
    int width = depthFrame.getWidth();
    int height = depthFrame.getHeight();

    memset(pHistogram, 0, histogramSize*sizeof(float));
    int restOfRow = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - width;

    unsigned int nNumberOfPoints = 0;
    for (int y = 0; y < height; ++y)
    {
        for (int x = 0; x < width; ++x, ++pDepth)
        {
            if (*pDepth != 0)
            {
                pHistogram[*pDepth]++;
                nNumberOfPoints++;
            }
        }
        pDepth += restOfRow;
    }
    for (int nIndex=1; nIndex<histogramSize; nIndex++)
    {
        pHistogram[nIndex] += pHistogram[nIndex-1];
    }
    if (nNumberOfPoints)
    {
        for (int nIndex=1; nIndex<histogramSize; nIndex++)
        {
            pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
        }
    }
}

void QNiTETrackerRenderer::DrawLimb(nite::UserTracker* pUserTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, QPainter *painter)
{
    float coordinates[6] = {0};
    pUserTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
    pUserTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

    coordinates[0] *= width()/(float)g_nXRes;
    coordinates[1] *= height()/(float)g_nYRes;
    coordinates[3] *= width()/(float)g_nXRes;
    coordinates[4] *= height()/(float)g_nYRes;

    if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
    {
        painter->setPen(QPen(Qt::yellow, 3, Qt::SolidLine, Qt::RoundCap));
    }
    else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
    {
        return;
    }
    else
    {
        painter->setPen(QPen(Qt::gray, 3, Qt::SolidLine, Qt::RoundCap));
    }

    painter->drawLine( QPointF(coordinates[0], coordinates[1]), QPointF(coordinates[3], coordinates[4]) );


    if (joint1.getPositionConfidence() == 1)
    {
        painter->setPen(QPen(Qt::yellow, 7, Qt::SolidLine, Qt::RoundCap));
    }
    else
    {
        painter->setPen(QPen(Qt::gray, 7, Qt::SolidLine, Qt::RoundCap));
    }

    painter->drawPoint(coordinates[0], coordinates[1]);

    if (joint2.getPositionConfidence() == 1)
    {
        painter->setPen(QPen(Qt::yellow, 7, Qt::SolidLine, Qt::RoundCap));
    }
    else
    {
        painter->setPen(QPen(Qt::gray, 7, Qt::SolidLine, Qt::RoundCap));
    }

    painter->drawPoint(coordinates[3], coordinates[4]);

}

void QNiTETrackerRenderer::DrawSkeleton(nite::UserTracker* pUserTracker, const nite::UserData& userData, QPainter *painter)
{
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP),  painter);


    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT),  painter);

    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE),  painter);
    DrawLimb(pUserTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT),  painter);
}

void QNiTETrackerRenderer::paint(QPainter *painter)
{
    if(!m_initialized)
        return;

    m_qnite->lockFrameRef();

    nite::UserTrackerFrameRef & userTrackerFrame = (*m_qnite->getFrameRef());
    openni::VideoFrameRef depthFrame;
    nite::Status rc = m_userTracker->readFrame(&userTrackerFrame);
    if (rc != nite::STATUS_OK)
    {
        printf("GetNextData failed\n");
        return;
    }

    depthFrame = userTrackerFrame.getDepthFrame();
    g_nXRes = depthFrame.getVideoMode().getResolutionX();
    g_nYRes = depthFrame.getVideoMode().getResolutionY();

    if (m_pTexMap == 0)
    {
        m_nTexMapX = depthFrame.getVideoMode().getResolutionX();
        m_nTexMapY = depthFrame.getVideoMode().getResolutionY();
        m_pTexMap = new openni::RGB888Pixel[m_nTexMapX * m_nTexMapY];
    }

    const nite::UserMap& userLabels = userTrackerFrame.getUserMap();

    painter->eraseRect(0,0,width(),height());

    if (depthFrame.isValid())
    {
        if(!m_hasHistogram || !m_keepHistogram)
        {
            calculateHistogram(m_pDepthHist, MAX_DEPTH, depthFrame);
            m_hasHistogram = true;
        }

    }

    memset(m_pTexMap, 0, m_nTexMapX*m_nTexMapY*sizeof(openni::RGB888Pixel));

    float factor[3] = {1, 1, 1};
    const float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
    const int colorCount = 3;

    // check if we need to draw depth frame to texture
    if (depthFrame.isValid())
    {
        const nite::UserId* pLabels = userLabels.getPixels();

        const openni::DepthPixel* pDepthRow = (const openni::DepthPixel*)depthFrame.getData();
        openni::RGB888Pixel* pTexRow = m_pTexMap + depthFrame.getCropOriginY() * m_nTexMapX;
        int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

        for (int y = 0; y < depthFrame.getHeight(); ++y)
        {
            const openni::DepthPixel* pDepth = pDepthRow;
            openni::RGB888Pixel* pTex = pTexRow + depthFrame.getCropOriginX();

            for (int x = 0; x < depthFrame.getWidth(); ++x, ++pDepth, ++pTex, ++pLabels)
            {
                if (*pDepth != 0)
                {
                    if (*pLabels == 0)
                    {
                        if (false)//!g_drawBackground)
                        {
                            factor[0] = factor[1] = factor[2] = 0;

                        }
                        else
                        {
                            factor[0] = Colors[colorCount][0];
                            factor[1] = Colors[colorCount][1];
                            factor[2] = Colors[colorCount][2];
                        }
                    }
                    else
                    {
                        factor[0] = Colors[*pLabels % colorCount][0];
                        factor[1] = Colors[*pLabels % colorCount][1];
                        factor[2] = Colors[*pLabels % colorCount][2];
                    }

                    int nHistValue = m_pDepthHist[*pDepth]; // (1.0 - (*pDepth)*0.00001525902189669642d)*256.0f;//(*pDepth)%256;//
                    pTex->r = nHistValue*factor[0];
                    pTex->g = nHistValue*factor[1];
                    pTex->b = nHistValue*factor[2];

                    factor[0] = factor[1] = factor[2] = 1;
                }
            }

            pDepthRow += rowSize;
            pTexRow += m_nTexMapX;
        }
    }

    QImage image( reinterpret_cast<uchar *>(m_pTexMap), m_nTexMapX, m_nTexMapY, QImage::Format_RGB888);
    painter->drawImage(QRect(0, 0, width(), height()), image);

    const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
    for (int i = 0; i < users.getSize(); ++i)
    {
        const nite::UserData& user = users[i];

        //updateUserState(user, userTrackerFrame.getTimestamp());
        if (user.isNew())
        {
            m_userTracker->startSkeletonTracking(user.getId());
        }
        else if (!user.isLost())
        {
            if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
            {
                DrawSkeleton(m_userTracker, user, painter);
            }
        }

    }

    m_qnite->unlockFrameRef();

}

