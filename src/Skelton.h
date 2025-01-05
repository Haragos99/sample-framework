#pragma once
#include "Bone.h"
#include "Object3D.h"
#include "Skinning.h"


class Skelton : public Object3D {
private:
    std::vector<Vec> points;
    std::vector<Vec> Tpose;
    int n_joint;
    std::vector< std::pair<int, int>> indexes;
    std::vector<std::vector<int>> childrenMatrix;
    std::vector<Vec> colors_bone{
    Vec(0.0, 1.0, 1.0),
    Vec(1.0, 1.0, 0.0),
    Vec(1.0, 0.0, 1.0),
    Vec(0.5, 1.0, 0.5),
    Vec(1.0, 0.5, 0.5),
    Vec(0.5, 0.5, 1.0),
    Vec(0.1, 0.2, 0.2),
    Vec(0.7, 0.3, 0.0),
    Vec(0.0, 0.3, 0.7),
    Vec(0.0, 0.7, 0.3),
    Vec(0.7, 0.0, 0.3),
    Vec(0.3, 0.0, 0.7),
    Vec(0.3, 0.7, 0.0),
    Vec(0.7, 0.0, 0.0),
    Vec(0.0, 0.7 ,0.0),
    Vec(0.0, 0.0, 0.7),
    Vec(0.7, 0.7, 0.7),
    Vec(0.5, 1.0, 0.2),
    Vec(1.0, 0.6, 0.2),
    Vec(0.4, 0.5, 1.0),
    Vec(0.1, 0.2, 0.2),
    Vec(0.5, 0.3, 0.0),
    Vec(0.1, 0.3, 0.7),
    Vec(0.1, 0.7, 0.3),
    };
    MyMesh* mesh;
    std::unique_ptr<Skinning> skinningtechnic;
public:
    Joint* root;
    std::vector<Bone> bones;
    std::vector<Vec> po;
    std::vector<Joint*> joint;

    Skelton(std::vector<Vec> point, std::vector<std::vector<int>> _childrenMatrix, std::vector< std::pair<int, int>> _indexes) {
        points = point;
        Tpose = point;
        childrenMatrix = _childrenMatrix;
        indexes = _indexes;
        skinningtechnic = std::make_unique<Skinning>();
    }
    Skelton() {  }//delete

    void skinning(std::shared_ptr<BaseMesh> basemesh);

    void setSkinning(std::unique_ptr<Skinning> skinning);

    int getSize() { return bones.size(); }

    void set_deafult_matrix() { root->set_deafult_matrix(root); }

    void get_join_point(Joint* j)
    {
        po.push_back(j->Tpose);
        for (int i = 0; i < j->children.size(); i++)
        {
            get_join_point(j->children[i]);
        }

    }

    bool hasMultipleChildren(Joint* j);

    void loadFile(const std::string& filename);

    void getList(Joint* j);

    void setJointMatrix(int id, Vec& angle);

    std::vector<Vec> getPoints(Joint* j) { get_join_point(j); return po; }

    std::vector<Joint*> getJointtoList(Joint* j) { getList(j); return joint; }

    void animate(float current_time, MyMesh& mesh);

    void animate_mesh(MyMesh& mesh, bool isweight, bool inv = false);

    std::vector<Axes>arrows();

    void build();

    void calculateMatrix();

    std::vector<Vec> getPointlist() { return points; }

    void buildTree(std::vector<Joint*>& joints);

    void addJoint(Joint* parent, Joint* child);

    void drawarrow()
    {
        for (int i = 0; i < points.size(); i++)
        {
            Joint* j = root->searchbyid(root, i);
            Vec const& p = points[j->id];
            glPushName(j->id);
            glRasterPos3fv(p);
            glPopName();

        }
    }

    void draw()
    {
        for (auto b : bones)
        {
            b.draw();
        }
        root->draw(root);
    }

    void draw(Vis::Visualization& vis) override;

    void drawWithNames(Vis::Visualization& vis) const override;

    void movement(int selected, const Vector& position) override;

    void rotate(int selected, Vec angel) override;
     
    void setCameraFocus(Vector& min, Vector& max) override;
    
    void scale(float scale) override;

    void animate(float time) override;

    Vec postSelection(const int p) override;

    bool save(const std::string& filename);

    void reset() { root->reset_all(root); }
    ~Skelton(){}

};
