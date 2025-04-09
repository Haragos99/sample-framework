#include "Skelton.h"
#include "../skinning/DualQuaternionSkinning.h"
void Skelton::scale(float scale)
{

}

void Skelton::drawWithNames(Vis::Visualization& vis) const
{
    for (int i = 0; i < points.size(); i++)
    {
        Joint* j = root->searchbyid(root, i);
        Vec const& p = j->point;
        glPushName(j->id);
        glRasterPos3fv(p);
        glPopName();

    }
}



void Skelton::setSkinning(std::shared_ptr<Skinning> skinning)
{
    skinningtechnic = skinning;
}

void Skelton::skinning(std::shared_ptr<BaseMesh> basemesh)
{
    mesh = basemesh;
    skinningtechnic->execute(mesh, bones);
}
void Skelton::setCameraFocus(Vector& min, Vector& max)
{
    for (auto v : points) {
        min.minimize(Vector(v.x, v.y, v.z));
        max.maximize(Vector(v.x, v.y, v.z));
    }
}

void Skelton::draw(Vis::Visualization& vis)
{
    for (auto b : bones)
    {
        b.draw();
    }
    root->draw(root);
}


void Skelton::addJoint(Joint* parent, Joint* child) {
    if (parent == nullptr)
    {
        root = child; // If no parent, child becomes root
    }
    else {
        parent->children.push_back(child);
        child->parent = parent;
    }
}

void Skelton::buildTree(std::vector<Joint*>& joints)
{
    for (int i = 0; i < childrenMatrix.size(); ++i)
    {
        for (int j = 0; j < childrenMatrix[i].size(); ++j)
        {
            int childId = childrenMatrix[i][j];
            if (childId != -1) // -1 indicates no child
            {
                Joint* parent = joints[i];
                Joint* child = joints[childId];
                addJoint(parent, child);
            }
        }
    }
}


Joint* Skelton::getSelectedJoint(int id)
{
    Joint* joint = root->searchbyid(root, id);
    return joint;
}




//TODO: Rewrite
void Skelton::calculateMatrix(std::vector<Vec>& ik, Joint* join)
{
    int n = ik.size();
    joint.clear();
    auto joints = getJointtoList(join);

    for (int i = 1; i < n; ++i)
    {
        Joint* joint = joints[i];
        Vec oldpoint = joints[i]->Tpose;
        Vec prevoldpoint = joints[i - 1]->Tpose;
        Vec old_diff = oldpoint - prevoldpoint;
        Vec new_diff = ik[i] - ik[i - 1];
        old_diff = old_diff.unit();
        new_diff = new_diff.unit();
        Vec axis = old_diff ^ new_diff;
        float dot = old_diff.x * new_diff.x + old_diff.y * new_diff.y + old_diff.z * new_diff.z;
        float rotAngle = std::atan2(axis.norm(), dot);
        Vec pivot = joint->parent->Tpose;
        Mat4 T1 = TranslateMatrix(-pivot);
        Mat4 T2 = TranslateMatrix(pivot);
        Mat4 R;
        if (axis.norm() > 1E-12) {
            axis = axis.unit();
            R = RotationMatrix(rotAngle, axis);
        }
        joint->R = R;
        Mat4 M = T1 * R * TranslateMatrix(joint->parent->point);  // * T2;
        Vec4 newpoint = Vec4(joint->Tpose) * M;
        joint->M = M;
        joint->point = Vec(newpoint.x, newpoint.y, newpoint.z);
    }
}

void Skelton::loadFile(const std::string& filename)
{
    std::ifstream file(filename); 
    std::vector<int> children;
    std::string line;
    while (getline(file, line)) {
        std::stringstream ss(line);
        char type;
        ss >> type;
        if (type == 'b') {
            char dummy;
            double x, y, z;
            ss >> dummy >> x >> dummy >> y >> dummy >> z >> dummy;
            points.push_back({ x, y, z });
        }
        else if (type == 'a') {
            char dummy;
            int idx1, idx2;
            ss >> dummy >> idx1 >> dummy >> idx2 >> dummy;
            indexes.push_back({ idx1, idx2 });
        }
        else if (type == 'i') {
            char dummy;
            int idx;
            int size;
            ss >> dummy >> size;
            for (int i = 0; i < size; i++)
            {
                ss >> dummy >> idx;
                children.push_back(idx);
            }
            childrenMatrix.push_back(children);
            children.clear();
        }
    }

    file.close();
}

void Skelton::build()
{
    std::vector<Joint*> joints;
    for (int i = 0; i < points.size(); i++)
    {
        joints.push_back(new Joint(points[i], i));
    }
    n_joint = joints.size();

    //Build tree by BFS
    root = joints[0];

    buildTree(joints);

    // build the bones
    int size = indexes.size();
    for (int i = 0; i < size; i++)
    {
        bones.push_back(Bone(joints[indexes[i].first], joints[indexes[i].second], i, colors_bone[i]));
    }

}

std::vector<Axes>Skelton::arrows()
{
    std::vector<Axes> result;
    for (int i = 0; i < n_joint; i++)
    {
        Joint* j = root->searchbyid(root, i);
        result.push_back(Axes(j->point, 0.1, j->R));
    }
    return result;
}


bool Skelton::hasMultipleChildren(Joint* j) {
    getList(j);
    for (auto j : joint)
    {
        if (j->children.size() > 1)
        {
            joint.clear();
            return false;
        }
    }
    joint.clear();
    return true;
}

void Skelton::setJointMatrix(int id, Vec& angle)
{
    Joint* j = root->searchbyid(root, id);
    j->setMatrix(angle);
}

bool Skelton::save(const std::string& filename)
{
    try {

        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        for (int i = 0; i < n_joint; i++)
        {
            Joint* j = root->searchbyid(root, i);
            Vec p = j->point;
            f << 'b' << ';' << p[0] << ';' << p[1] << ';' << p[2] << ';' << std::endl;
        }
        for (const auto& i : indexes)
        {
            f << 'a' << ';' << i.first << ';' << i.second << ';' << std::endl;
        }

        for (const auto& m : childrenMatrix)
        {
            std::string line = "i;";
            line = line + std::to_string(m.size()) + ";";
            for (int i : m)
            {
                line = line + std::to_string(i) + ";";
            }
            f << line << std::endl;
        }
    }
    catch (std::ifstream::failure&) {
        return false;
    }

    return true;
}

void Skelton::movement(int selected, const Vector& position)
{
   Joint* joint = root->searchbyid(root, selected);
   joint->point = Vec(position.data());
   animateMesh();
}

// Bool inv
void Skelton::animateMesh(bool inv)
{
    if (skinningtechnic != nullptr && mesh != nullptr)
    {
        skinningtechnic->animatemesh(mesh, bones, inv);
        /*
        DualQuaternionSkinning dq;
        joint.clear();
        getList(root);
        dq.animatemesh(mesh, bones, inv);
        */
    }
}


void Skelton::rotate(int selected, Vec angel)
{
    Joint* joint = root->searchbyid(root, selected);
    root->change_all_rotason(joint, joint->point, angel);
    // Toodo Animate the mesh 
    animateMesh();
    
}

void Skelton::addKeyframes(int selected, float timeline)
{
    Joint* joint = root->searchbyid(root, selected);
    Keyframe k = Keyframe(timeline, joint->id, joint->angel);
    root->addframe(joint,k);// TODO: Think about this solution
}


void Skelton::reset()
{
    root->reset_all(root);
}



void Skelton::getList(Joint* j)
{
    joint.push_back(j);
    for (int i = 0; i < j->children.size(); i++)
    {
        getList(j->children[i]);
    }
}


void Skelton::animate(float time)
{
    for (int i = 0; i < n_joint; i++)
    {
        Joint* joint = root->searchbyid(root, i);
        if (joint->keyframes.size() != 0 && joint->keyframes.back().time() >= time)
        {
            for (size_t j = 0; j < joint->keyframes.size() - 1; ++j) {
                Keyframe& startFrame = joint->keyframes[j];
                Keyframe& endFrame = joint->keyframes[j + 1];
                if (time >= startFrame.time() && time <= endFrame.time())
                {
                    Vec pivot = joint->point;
                    Vec rotated = (endFrame.angeles() - startFrame.angeles());
                    float timediff = (endFrame.time() - startFrame.time());
                    float step = 1;
                    float rate = timediff * step;
                    Vec angels = rotated / rate;
                    joint->calculateMatrecies(joint, pivot, angels);
                }
            }
        }

    }
    root->transform_point(root);
    animateMesh();
    set_deafult_matrix();
     
}

Vec Skelton::postSelection(const int p)
{
    Joint* joint = root->searchbyid(root, p);
    return joint->point;
}

void Skelton::datainfo()
{

}