#include "Bone.h"

void BonePoly::draw()
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //glDisable(GL_LIGHTING);
    drawface(top, aP, cP);
    drawface(top, aP, dP);
    drawface(top, bP, cP);
    drawface(top, bP, dP);

    drawface(down, aP, cP);
    drawface(down, aP, dP);
    drawface(down, bP, cP);
    drawface(down, bP, dP);

    drawLines(top, aP);
    drawLines(top, bP);
    drawLines(top, cP);
    drawLines(top, dP);
    drawLines(cP, aP);
    drawLines(dP, aP);
    drawLines(cP, bP);
    drawLines(cP, dP);
    drawLines(bP, dP);
    drawLines(down, aP);
    drawLines(down, bP);
    drawLines(down, cP);
    drawLines(down, dP);
    glDisable(GL_BLEND);
    //glEnable(GL_LIGHTING);
}

void BonePoly::drawLines(Vec& a, Vec& b)
{
    glLineWidth(1.0);
    glBegin(GL_LINES);
    glColor3d(0,0,0);
    glVertex3dv(a);
    glVertex3dv(b);
    glEnd();
}
void BonePoly::drawface(Vec& a, Vec& b, Vec& c)
{
    
    glBegin(GL_POLYGON);
    glColor4d(color.x, color.y, color.z, 0.3);
    glVertex3dv(a);
    glVertex3dv(b);
    glVertex3dv(c);
    Vec ab = a - b;
    Vec ac = a - c;
    Vec normal = calcnormal(ab, ac);
    glNormal3dv(normal);
    glEnd();
   
}
void BonePoly::calculatepoly()
{
    double ox = (start->point.x + end->point.x) / 2;
    double oy = (start->point.y + end->point.y) / 2;
    double oz = (start->point.z + end->point.z) / 2;
    Vec origin = Vec(ox, oy, oz);
    Vec ir = end->point - start->point;
    origin = start->point + ir * 1.0 / 5;
    Vec v = end->point - start->point;
    Vec u = Vec(1, 0, 0);
    double factor = 8;

    Vec w = v ^ u;
    Vec wn = w * (v.norm() / (factor * w.norm()));

    Vec b = origin + wn;

    Vec a = origin - wn;

    u = Vec(0, 0, 1);

    w = v ^ u;
    wn = w * (v.norm() / (factor * w.norm()));
    Vec d = origin + wn;
    Vec c = origin - wn;
    down = end->point;
    top = start->point;
    aP = a;
    bP = b;
    cP = c;
    dP = d;

    /*
    glColor3d(1.0, 0.0, 1.0);
    glPointSize(50.0);
    glBegin(GL_POINTS);
    glVertex3dv(end->point);
    glVertex3dv(start->point);
    glVertex3dv(b);
    glVertex3dv(d);
    glVertex3dv(a);
    glVertex3dv(c);
    glEnd();
    glEnable(GL_LIGHTING);
    */
    draw();
    

}

void Bone::draw()
{
    if (start != nullptr)// Make it better
    {
        glLineWidth(200.0);
        glBegin(GL_LINES);
        glColor3d(color.x, color.y, color.z);
        glVertex3dv(start->point);
        glVertex3dv(end->point);
        glEnd();

        bp.calculatepoly();
    }
    


}


void Bone::manypoints()
{

    Vec ir = end->point - start->point;
    double len = sqrt(pow(end->point.x - start->point.x, 2) +
        pow(end->point.y - start->point.y, 2) + pow(end->point.z - start->point.z, 2));
    int res = 100;
    for (int i = 0; i < res; i++)
    {
        Vec t;
        t = start->point + ir * 1.0 / (res - 1) * i;
        points.push_back(t);
    }
}


bool Bone::isLastBone()
{
    return end->children.size() == 0;
}


