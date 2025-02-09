#pragma once
#include "Bone.h"
#include "../Object3D.h"
#include "src/skinning/Skinning.h"


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
    std::shared_ptr<Skinning> skinningtechnic;
    std::shared_ptr<BaseMesh> mesh;
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
        
    }
    Skelton() {  }//delete

    void skinning(std::shared_ptr<BaseMesh> basemesh);

    void setSkinning(std::shared_ptr<Skinning> skinning);

    int getSize() { return bones.size(); }

    void set_deafult_matrix() { root->set_deafult_matrix(root); }


    bool hasMultipleChildren(Joint* j);

    void loadFile(const std::string& filename);

    void getList(Joint* j);

    void setJointMatrix(int id, Vec& angle);


    std::vector<Joint*> getJointtoList(Joint* j) { getList(j); return joint; }

    std::vector<Axes>arrows();

    void build();

    void calculateMatrix(std::vector<Vec>& ik, Joint* joint);

    std::vector<Vec> getPointlist() { return points; }

    void buildTree(std::vector<Joint*>& joints);

    void addJoint(Joint* parent, Joint* child);

    void datainfo() override;

    void draw(Vis::Visualization& vis) override;

    void drawWithNames(Vis::Visualization& vis) const override;

    void movement(int selected, const Vector& position) override;

    void rotate(int selected, Vec angel) override;
     
    void setCameraFocus(Vector& min, Vector& max) override;
    
    void scale(float scale) override;

    void animate(float time) override;

    Vec postSelection(const int p) override;

    void addKeyframes(int selected,float timeline) override;

    void reset()override;

    bool save(const std::string& filename);


    Joint* getSelectedJoint(int id);

    void animateMesh(bool inv = false);

    ~Skelton(){}

};
