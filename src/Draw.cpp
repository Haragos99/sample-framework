#include "MyViewer.h"



void MyViewer::draw() {

    for (auto object : objects)
    {
        object->draw(vis);
    }
    if (isAnimating_)
    {
        QImage frame = QOpenGLWidget::grabFramebuffer();
        render.addframe(frame);
    }

    if (axes.shown)
        drawAxes();
    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    drawText(10, int(1.5 * ((QApplication::font().pixelSize() > 0)
        ? QApplication::font().pixelSize()
        : QApplication::font().pointSize())),
        QString("Frame:") + QString(std::to_string(FrameSecond).c_str()));
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
}


void MyViewer::drawDelta()
{
    if (deltaMushFactor < 1)
    {

    }
}

void MyViewer::drawControlNet() const {
    glDisable(GL_LIGHTING);
    glLineWidth(3.0);
    glColor3d(0.3, 0.3, 1.0);
    size_t m = degree[1] + 1;
    for (size_t k = 0; k < 2; ++k)
        for (size_t i = 0; i <= degree[k]; ++i) {
            glBegin(GL_LINE_STRIP);
            for (size_t j = 0; j <= degree[1 - k]; ++j) {
                size_t const index = k ? j * m + i : i * m + j;
                const auto& p = control_points[index];
                glVertex3dv(p);
            }
            glEnd();
        }
    glLineWidth(1.0);
    glPointSize(8.0);
    glColor3d(1.0, 0.0, 1.0);
    glBegin(GL_POINTS);
    for (const auto& p : control_points)
        glVertex3dv(p);
    glEnd();
    glPointSize(1.0);
    glEnable(GL_LIGHTING);
}