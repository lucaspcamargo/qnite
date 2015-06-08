#include "qnitecolorrenderer.h"
#include "qnite.h"
#include <QPainter>
#include <QImage>

QNiTEColorRenderer::QNiTEColorRenderer(QQuickItem *parent) : QQuickPaintedItem(parent)
{
    m_initialized = false;

}

QNiTEColorRenderer::~QNiTEColorRenderer()
{

}

void QNiTEColorRenderer::paint(QPainter * painter)
{
    if(!m_initialized) return;

    m_qnite->lockRGBFrameRef();
    openni::VideoFrameRef & frame = m_qnite->m_rgbFrameRef;

    if(!frame.isValid())
    {
        m_qnite->unlockRGBFrameRef();
        return;
    }

    QImage frameImage((const uchar *)frame.getData(), frame.getWidth(), frame.getHeight(), QImage::Format_RGB888);


    painter->drawImage(QRect(0, 0, width(), height()), frameImage);

    m_qnite->unlockRGBFrameRef();
}

void QNiTEColorRenderer::initialize()
{
    if(m_initialized || !m_kinect) return;

    m_qnite = reinterpret_cast<QNiTE*>(m_kinect);

    connect(m_qnite, &QNiTE::newRGBFrame, this, &QNiTEColorRenderer::onNewFrame);

    m_initialized = (true);
    emit initializedChanged(true);
}

void QNiTEColorRenderer::onNewFrame()
{
    update();
}

