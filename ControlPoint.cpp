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


