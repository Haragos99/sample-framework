#include "ControlPoint.h"

ControlPoint::ControlPoint(Vec _position)
{
    position_ = _position;
    color = Vec(1, 0, 0);

}
ControlPoint::ControlPoint(Vec _position, int _id)
{
    position_ = _position;
    color = Vec(1, 0, 0);
    id = _id;
}



void ControlPoint::datainfo()
{

}




ControlPoint::ControlPoint(Vec _position, int _id, std::shared_ptr<Skelton> skelton)
{
    position_ = _position;
    color = Vec(1, 0, 0);
    id = _id;
    IK = std::make_unique<InverseKinematics>(skelton, position_);
}


void ControlPoint::drawarrow()
{
    Vec const& p = position_;
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
    glVertex3dv(position_);
    glEnd();
    glEnable(GL_LIGHTING);
}

void ControlPoint::animate(float time)
{ 
    for (size_t i = 0; i < keyframes.size() - 1; ++i) {

        if (time >= keyframes[i].time() && time <= keyframes[i + 1].time()) {
            const Keyframe& startKeyframe = keyframes[i];
            const Keyframe& endKeyframe = keyframes[i + 1];
            Vec start = startKeyframe.position();
            Vec end = endKeyframe.position();
            float curent = (time - keyframes[i].time()) / (keyframes[i + 1].time() - keyframes[i].time());

            position_ = (qreal)(1.0f - curent) * start + (qreal)curent * end;
            IK->setPosition(position_);
            IK->execute(jointid);
        }
    }
}




void ControlPoint::reset()
{

}

void ControlPoint::drawWithNames(Vis::Visualization& vis) const
{
    Vec const& p = position_;
    glPushName(id);
    glRasterPos3fv(p);
    glPopName();
}
Vec ControlPoint::postSelection(const int p)
{
    return position_;
}
void ControlPoint::draw(Vis::Visualization& vis)
{
    glDisable(GL_LIGHTING);
    glColor3d(color.x, color.y, color.z);
    glPointSize(50.0);
    glBegin(GL_POINTS);
    glVertex3dv(position_);
    glEnd();
    glEnable(GL_LIGHTING);
}
void ControlPoint::movement(int selected, const Vector& position)
{
    position_ = Vec(position.data());
    IK->setPosition(position_);
    IK->execute(jointid);
}
void ControlPoint::rotate(int selected, Vec angel)
{

}

void ControlPoint::scale(float scale)
{

}

void ControlPoint::setCameraFocus(Vector& min, Vector& max)
{

}


void ControlPoint::inversekinematics(std::shared_ptr<Skelton> skelton)
{
    IK = std::make_unique<InverseKinematics>(skelton,position_);
}
void ControlPoint::addKeyframes(int selected,float timeline)
{
    Keyframe keyfram = Keyframe(timeline, position_, id);
    keyframes.push_back(keyfram);
}