#include "ControlPoint.h"



void ControlPoint::drawarrow()
{
    Vec const& p = position;
    glPushName(id);
    glRasterPos3fv(p);
    glPopName();

}
void ControlPoint::draw()
{
    glDisable(GL_LIGHTING);
    glColor3d(color.x, color.y, color.z);
    glPointSize(50.0);
    glBegin(GL_POINTS);
    glVertex3dv(position);
    glEnd();
    glEnable(GL_LIGHTING);
}

void ControlPoint::animate(float t)
{ 
    for (size_t i = 0; i < keyframes.size() - 1; ++i) {

        if (t >= keyframes[i].time() && t <= keyframes[i + 1].time()) {
            const Keyframe& startKeyframe = keyframes[i];
            const Keyframe& endKeyframe = keyframes[i + 1];
            Vec start = startKeyframe.position();
            Vec end = endKeyframe.position();
            float curent = (t - keyframes[i].time()) / (keyframes[i + 1].time() - keyframes[i].time());

            position = (qreal)(1.0f - curent) * start + (qreal)curent * end;
        }
    }
}

