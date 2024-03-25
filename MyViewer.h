// -*- mode: c++ -*-
#pragma once

#include <string>
#include <Eigen/Eigen>
#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QGLViewer/quaternion.h>
#include"Bone.h"
#include "Openfiler.hpp"
#include <fstream>
#include <iostream>
#include <vector>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include <QtGui/QKeyEvent>
#include <QtWidgets>
#include <QGLViewer/quaternion.h>
#include <map>
#include <algorithm>

using qglviewer::Vec;


class MyViewer : public QGLViewer {
    Q_OBJECT

public:
    explicit MyViewer(QWidget* parent);
    virtual ~MyViewer();
    void keyframe_add();
    inline double getCutoffRatio() const;
    inline void setCutoffRatio(double ratio);
    inline double getMeanMin() const;
    inline void setMeanMin(double min);
    inline double getMeanMax() const;
    inline void setMeanMax(double max);
    inline const double* getSlicingDir() const;
    inline void setSlicingDir(double x, double y, double z);
    inline double getSlicingScaling() const;
    inline void setSlicingScaling(double scaling);
    bool openMesh(const std::string& filename, bool update_view = true);
    bool openSkelton(const std::string& filename, bool update_view = true);
    bool openBezier(const std::string& filename, bool update_view = true);
    bool saveBezier(const std::string& filename);
    bool saveBone(const std::string& filename);
    int getbone_size() { return b.size(); }
    void setMesh() { model_type = ModelType::MESH; }
    void setBone() { model_type = ModelType::SKELTON; }
    void selectedvert();
    void wierframe() {
        show_wireframe = !show_wireframe;
        update();
    }
    float bright = 0.5;
    void draw_smooth(); 
    void skining() { visualization = Visualization::WEIGH; }
    void Laplace() {
        smooth = mesh;
        bright = 0.1;
        transparent = true;
        createL_smooot(smooth); update(); 
    }
    void delta(){
        weigh();
        Delta_Mush(vec);
        delatamush = true;
        update();
    }
    void Boneheat()
    {
        auto dlg = std::make_unique<QDialog>(this);
        auto* hb1 = new QHBoxLayout;
        auto* vb = new QVBoxLayout;

        QLabel* text;
        if (points.size() != 0 && mesh.n_vertices() != 0)
        {

            Epsil();
            if (isweight == true && mehet == true)
            {
                Smooth();
                model_type = ModelType::SKELTON;

                text = new QLabel(tr("Success"));

            }
            else
            {
                text = new QLabel(tr("Error: No weight in the mesh"));
            }
        }
        else
        {
            text = new QLabel(tr("Error: No mesh or skellton"));
        }
        hb1->addWidget(text);
        vb->addLayout(hb1);
        dlg->setWindowTitle(tr("Message"));
        dlg->setLayout(vb);
        if (dlg->exec() == QDialog::Accepted) {
            update();
        }
    }

    void put_original(Tree& oldTree, Tree& newTree);

    void Epsil() {
        auto dlg = std::make_unique<QDialog>(this);
        auto* hb1 = new QHBoxLayout;
        auto* vb = new QVBoxLayout;
        auto* ok = new QPushButton(tr("Ok"));
        connect(ok, SIGNAL(pressed()), dlg.get(), SLOT(accept()));
        ok->setDefault(true);
        QLabel* text;
        auto* sb_H = new QDoubleSpinBox;
        sb_H->setDecimals(4);
        sb_H->setSingleStep(0.0001);
        sb_H->setRange(0.0001, 1);
        hb1->addWidget(sb_H);
        hb1->addWidget(ok);
        vb->addLayout(hb1);
        dlg->setWindowTitle(tr("Skalar"));
        dlg->setLayout(vb);
        if (dlg->exec() == QDialog::Accepted) {
            epsilon = sb_H->value();
            update();
        }


    }

    bool transparent = false;

    float& getFrameSecond() { return FrameSecond; }
    double epsilon = 0.001;
    void Reset();
    int wi = 2;
    void index_of_weight() {
        if (points.size() != 0 && mesh.n_vertices() != 0)
        {
            model_type = ModelType::SKELTON;
            visualization = Visualization::WEIGH2;
            wi++;
        }
        update();
    }
    void show() {
        show_solid = !show_solid;
        update();
    }

    void Invers();

    void Databone();

    void Frame();
    Vec angels;
    Vec ang;
    void Smooth();
    void weigh();
    void Rotate();
signals:
    void startComputation(QString message);
    void midComputation(int percent);
    void endComputation();
    void displayMessage(const QString& message);

protected:
    virtual void init() override;
    virtual void draw() override;
    virtual void animate()override;
    virtual void drawWithNames() override;
    virtual void postSelection(const QPoint& p) override;
    virtual void keyPressEvent(QKeyEvent* e) override;
    virtual void mouseMoveEvent(QMouseEvent* e) override;
    virtual QString helpString() const override;
private:
    struct MyTraits : public OpenMesh::DefaultTraits {
        using Point = OpenMesh::Vec3d; // the default would be Vec3f
        using Normal = OpenMesh::Vec3d;
        VertexTraits{
          OpenMesh::Vec3d original;
          double mean;              // approximated mean curvature
          std::vector<double> weigh;
          std::vector<double> distance;
          int idx_of_closest_bone;

        };
        VertexAttributes(OpenMesh::Attributes::Normal |
            OpenMesh::Attributes::Color | OpenMesh::Attributes::Status);
    };
    using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
    using Vector = OpenMesh::VectorT<double, 3>;

    // Mesh
    void updateMesh(bool update_mean_range = true);
    void updateVertexNormals();
#ifdef USE_JET_FITTING
    void updateWithJetFit(size_t neighbors);
#endif
    void localSystem(const Vector& normal, Vector& u, Vector& v);
    double voronoiWeight(MyMesh::HalfedgeHandle in_he);
    void updateMeanMinMax();
    void updateMeanCurvature();

    // Bezier
    static void bernsteinAll(size_t n, double u, std::vector<double>& coeff);
    void generateMesh(size_t resolution);

    // Visualization
    void setupCamera();
    Vec meanMapColor(double d) const;
    void drawControlNet() const;
    void drawSkleton();
    void drawAxes() const;
    void drawAxesWithNames() const;
    static Vec intersectLines(const Vec& ap, const Vec& ad, const Vec& bp, const Vec& bd);

    // Other
    void fairMesh();

    //////////////////////
    // Member variables //
    //////////////////////

    enum class ModelType { NONE, MESH, BEZIER_SURFACE, SKELTON, INVERZ } model_type;
    enum class SkelltonType { MAN, WRIST, ARM, FACE } skellton_type;
    // Mesh
    MyMesh mesh;

    MyMesh smooth;
    


    std::vector<int> used;
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



    double distance(Vec p, Vec p1)
    {
        double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));

        return len;
    }



    struct ControlPoint {
        Vec position;
        Vec color;
        ControlPoint() {}
        ControlPoint(Vec _position)
        {
            position = _position;
            color = Vec(1, 0, 0);

        }

        void drawarrow()
        {
            Vec const& p = position;
            glPushName(0);
            glRasterPos3fv(p);
            glPopName();

        }
        void draw()
        {
            glDisable(GL_LIGHTING);
            glColor3d(color.x, color.y, color.z);
            glPointSize(50.0);
            glBegin(GL_POINTS);
            glVertex3dv(position);
            glEnd();
            glEnable(GL_LIGHTING);
        }


    };

    ControlPoint target;

    void ininitSkelton();
    void createL(Eigen::SparseMatrix<double>& L);




    float FrameSecond = 0.0;
    QHBoxLayout* hb1 = new QHBoxLayout;
    QLabel* text_ = new QLabel;
    QVBoxLayout* vBox = new QVBoxLayout;

    //std::map<int, double> faceAreaMap;
    std::vector<std::pair<int, double>> sortedVector;
    std::vector<std::pair<int, double>> finalarea;
    std::map< MyMesh::FaceHandle, int> sortedMap;
    std::vector<Eigen::Vector4d> vec;
    // Custom comparator function to sort by values (double) in ascending order
    static bool sortByValue(const std::pair<int, double>& a, const std::pair<int, double>& b) {
        return a.second < b.second;
    }

    bool delatamush = false;
    void smoothvectors(std::vector<Vec>& smoothed);
    void smoothoriginal(std::vector<Vec>& smoothed);
    void Delta_Mush(std::vector<Eigen::Vector4d>& v);
    void Delta_Mush_two(std::vector<Eigen::Vector4d>& v);

    void getallpoints(Tree t);

    void get_change_points(Tree t);

    void armSkellton()
    {
        sk = Tree(points[0], 0);
        sk.child.push_back(Tree(points[1], 1));
        sk.child[0].child.push_back(Tree(points[2], 2));
        sk.child[0].child[0].child.push_back(Tree(points[3], 3));
    }

    void manSkellton()
    {
        sk = Tree(points[0], 0);
        sk.child.push_back(Tree(points[1], 1));// 
        sk.child.push_back(Tree(points[9], 9));// 
        sk.child.push_back(Tree(points[10], 10));// 

        sk.child[0].child.push_back(Tree(points[2], 2));
        sk.child[0].child.push_back(Tree(points[5], 5));
        sk.child[0].child[0].child.push_back(Tree(points[3], 3));
        sk.child[0].child[1].child.push_back(Tree(points[6], 6));
        sk.child[0].child[0].child[0].child.push_back(Tree(points[4], 4));
        sk.child[0].child[1].child[0].child.push_back(Tree(points[7], 7));
        sk.child[1].child.push_back(Tree(points[12], 12));
        sk.child[1].child[0].child.push_back(Tree(points[14], 14));
        sk.child[2].child.push_back(Tree(points[11], 11));
        sk.child[2].child[0].child.push_back(Tree(points[13], 13));
        sk.child.push_back(Tree(points[8], 8));
    }

    void csukloSkellton()
    {
        sk = Tree(points[0], 0);
        sk.child.push_back(Tree(points[1], 1));// 
        sk.child[0].child.push_back(Tree(points[2], 2));
        sk.child[0].child[0].child.push_back(Tree(points[3], 3));
        sk.child[0].child[0].child[0].child.push_back(Tree(points[4], 4));

        sk.child[0].child.push_back(Tree(points[5], 5));
        sk.child[0].child[1].child.push_back(Tree(points[6], 6));
        sk.child[0].child[1].child[0].child.push_back(Tree(points[7], 7));
        sk.child[0].child[1].child[0].child[0].child.push_back(Tree(points[8], 8));

        sk.child[0].child.push_back(Tree(points[9], 9));
        sk.child[0].child[2].child.push_back(Tree(points[10], 10));
        sk.child[0].child[2].child[0].child.push_back(Tree(points[11], 11));
        sk.child[0].child[2].child[0].child[0].child.push_back(Tree(points[12], 12));

        sk.child[0].child.push_back(Tree(points[13], 13));
        sk.child[0].child[3].child.push_back(Tree(points[14], 14));
        sk.child[0].child[3].child[0].child.push_back(Tree(points[15], 15));
        sk.child[0].child[3].child[0].child[0].child.push_back(Tree(points[16], 16));


        sk.child[0].child.push_back(Tree(points[17], 17));
        sk.child[0].child[4].child.push_back(Tree(points[18], 18));
        sk.child[0].child[4].child[0].child.push_back(Tree(points[19], 19));
        sk.child[0].child[4].child[0].child[0].child.push_back(Tree(points[20], 20));

    }


    void faceSkellton()
    {
        sk = Tree(points[0], 0);

        sk.child.push_back(Tree(points[1], 1));
        sk.child[0].child.push_back(Tree(points[2], 2));
        sk.child[0].child[0].child.push_back(Tree(points[3], 3));
        sk.child[0].child[0].child.push_back(Tree(points[4], 4));
        sk.child[0].child[0].child.push_back(Tree(points[5], 5));
        sk.child[0].child[0].child[1].child.push_back(Tree(points[6], 6));
        sk.child[0].child[0].child[1].child[0].child.push_back(Tree(points[7], 7));
        sk.child[0].child[0].child[1].child[0].child[0].child.push_back(Tree(points[8], 8));
        sk.child[0].child[0].child[1].child[0].child[0].child.push_back(Tree(points[9], 9));
        sk.child[0].child[0].child[1].child[0].child.push_back(Tree(points[10], 10));

        sk.child[0].child.push_back(Tree(points[11], 11));
        sk.child[0].child[1].child.push_back(Tree(points[12], 12));
        sk.child[0].child[1].child.push_back(Tree(points[13], 13));
        sk.child[0].child[1].child.push_back(Tree(points[14], 14));
        sk.child[0].child[1].child[1].child.push_back(Tree(points[15], 15));
        sk.child[0].child[1].child[1].child[0].child.push_back(Tree(points[16], 16));
        sk.child[0].child[1].child[1].child[0].child[0].child.push_back(Tree(points[17], 17));
        sk.child[0].child[1].child[1].child[0].child[0].child.push_back(Tree(points[18], 18));
        sk.child[0].child[1].child[1].child[0].child.push_back(Tree(points[19], 19));


    }


    void selectedjoin();

    void inverse_kinematics(ControlPoint t, Join* j);

    // this collect the bones
    std::vector<Bones> b;
    // this is the skeleton
    Tree sk;

    std::vector<Mat4> mteszt;

    void set_bone_matrix()
    {
        for (int i = 0; i < b.size(); i++)
        {
            b[i].M = Mat4();
        }
    }


    // for the animation api (it is simpal)
    Tree start;
    Tree end;

    Skelton skel;
    Skelton fab;
    int wx = 0;
    void animate_mesh();

    Tree FABRIK;

    std::vector<Vec>FABRIK_p;


    void createL_smooot(MyMesh& m);

    void move(std::vector<Vec> newp, std::vector<Vec> old);

    int elapsedTime;
    std::vector<int>indexes;
    std::vector<Vec> points;
    std::vector<Vec> selected_points_storage;
    float startAnimationTime_ = 0.0;
    float animationDuration_ = 1.0;
    std::vector<Keyframe> keyframes_;
    bool isAnimating_;
    Vec rotation;

    // Bezier
    size_t degree[2];
    std::vector<Vec> control_points;

    float currentTime() {
        auto now = std::chrono::high_resolution_clock::now();
        auto duration = now.time_since_epoch();
        return std::chrono::duration_cast<std::chrono::duration<float>>(duration).count();
    }

    void tree_to_array(Join* j);
    std::vector<Vec> ik;
    void IK_matrices();
    double sum_len();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="edgeHandle"></param>

    void setupCameraBone();
    bool mehet = false;
    bool isweight = false;
    std::vector<int> _used;
    // Visualization
    double mean_min, mean_max, cutoff_ratio;
    bool show_control_points, show_solid, show_wireframe, show_skelton;
    enum class Visualization { PLAIN, MEAN, SLICING, ISOPHOTES, WEIGH, WEIGH2 } visualization;
    GLuint isophote_texture, environment_texture, current_isophote_texture, slicing_texture;
    Vector slicing_dir;
    double slicing_scaling;
    int selected_vertex;
    struct ModificationAxes {
        bool shown;
        float size;
        int selected_axis;
        Vec position, grabbed_pos, original_pos;
    } axes;
    std::string last_filename;
    std::ofstream of;
};
// idó kivonva a másikból
#include "MyViewer.hpp"