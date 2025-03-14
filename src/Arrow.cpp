#include "MyViewer.h"


void MyViewer::drawAxes() const {
    const Vec& p = axes.position;
    glColor3d(1.0, 0.0, 0.0);
    drawArrow(p, p + Vec(axes.size, 0.0, 0.0), axes.size / 50.0);
    glColor3d(0.0, 1.0, 0.0);
    drawArrow(p, p + Vec(0.0, axes.size, 0.0), axes.size / 50.0);
    glColor3d(0.0, 0.0, 1.0);
    drawArrow(p, p + Vec(0.0, 0.0, axes.size), axes.size / 50.0);
    glEnd();
}



void MyViewer::drawJointAxes(std::vector<Axes>& a) const {
    for (auto& ax : a)
    {
        const Vec& p = ax.position;


        Vec4 x = Vec4(Vec(ax.size, 0.0, 0.0)) * ax.M;
        Vec4 y = Vec4(Vec(0.0, ax.size, 0.0)) * ax.M;
        Vec4 z = Vec4(Vec(0.0, 0.0, ax.size)) * ax.M;

        glColor3d(1.0, 0.0, 0.0);
        drawArrow(p, p + x.to_Vec(), ax.size / 50.0);
        glColor3d(0.0, 1.0, 0.0);
        drawArrow(p, p + y.to_Vec(), ax.size / 50.0);
        glColor3d(0.0, 0.0, 1.0);
        drawArrow(p, p + z.to_Vec(), ax.size / 50.0);
        glEnd();
    } 
}



void MyViewer::drawWithNames() {
    if (axes.shown)
        return drawAxesWithNames();

    for (size_t i = 0; i < objects.size(); ++i) {
        glPushName(i);
        objects[i]->drawWithNames(vis);
        glPopName();
    }
}

void MyViewer::drawAxesWithNames() const {
    const Vec& p = axes.position;
    glPushName(0);
    drawArrow(p, p + Vec(axes.size, 0.0, 0.0), axes.size / 50.0);
    glPopName();
    glPushName(1);
    drawArrow(p, p + Vec(0.0, axes.size, 0.0), axes.size / 50.0);
    glPopName();
    glPushName(2);
    drawArrow(p, p + Vec(0.0, 0.0, axes.size), axes.size / 50.0);
    glPopName();
}

void MyViewer::postSelection(const QPoint& p) {
    int sel = selectedName();
    if (sel == -1) {
        axes.shown = false;
        return;
    }

    if (axes.shown) {
        axes.selected_axis = sel;
        bool found;
        axes.grabbed_pos = camera()->pointUnderPixel(p, found);
        axes.original_pos = axes.position;
        if (!found)
            axes.shown = false;
        return;
    }

    selected_vertex = sel;
    ///New Version
    if (!objects.empty())
    {
        axes.position = objects[selected_object]->postSelection(sel);
    }

    double depth = camera()->projectedCoordinatesOf(axes.position)[2];
    Vec q1 = camera()->unprojectedCoordinatesOf(Vec(0.0, 0.0, depth));
    Vec q2 = camera()->unprojectedCoordinatesOf(Vec(width(), height(), depth));
    axes.size = (q1 - q2).norm() / 10.0;
    axes.shown = true;
    axes.selected_axis = -1;
}