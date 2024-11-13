// -*- mode: c++ -*-
#pragma once
#include <string>
#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QGLViewer/quaternion.h>
#include"Bone.h"
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
#include"HRBF.h"
#include "MarchingCubes.h"
#include "DeltaMush.h"
#include "mclccd\BVHTree.hpp"
#include "ControlPoint.h"
#include "Render.h"
#include <QOpenGLWidget>
#include "Collison.h"
#include "KinectSkelton.h"
#include <QTimer>

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
    float deltaMushFactor = 1.0;
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
        SetDistance();
        dm = DeltaMush(mesh);
        dm.setHelper(mesh);
        MushHelper = mesh;
        Helper = mesh;
        //dm.Delta_Mush();
        //DirectMush();
        Delta_Mush(vec);
        delatamush = true;
        mtransperent = mesh;
        update();
    }
    void poission() { seperateMesh(); }

    void CalculateImplicit();

    void Boneheat()
    {
        auto dlg = std::make_unique<QDialog>(this);
        auto* hb1 = new QHBoxLayout;
        auto* vb = new QVBoxLayout;

        QLabel* text;
        if (mesh.n_vertices() != 0)
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

    void setSlider(int value) {
        deltaMushFactor = (float)value/100.0f;
        dm.deltaMushFactor = deltaMushFactor;
        dm.setHelper(mesh);
        if (delatamush)
        {
            //TestDelta(vec);
            Delta_Mush_two(vec);
           //dm.Delta_Mush_two(mesh);
        }
        update();
    }
    
    std::set<MyMesh::VertexHandle> vert;
    std::vector<MyMesh::VertexHandle> verteces;
    std::set<MyMesh::VertexHandle> colliedverteces;
    std::set<MyMesh::FaceHandle> colliedfaces;
    std::set<MyMesh::EdgeHandle> colliededges;
    void smoothcollison(std::set<MyMesh::VertexHandle> verteces);

    bool Mydelta;

    void DeltaMush2(std::vector<Eigen::Vector4d> v);


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

    KinectSkelton kinect;
    QTimer* timer;
    bool transparent = false;
    bool transparent2 = true;
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

    void callKinekcnUpdate() {
        // Call the kinect's update function
        kinect.update();

        // Optionally refresh the viewer
        update();
    }


    void startTimer();

    void stopTimer() {
        if (timer->isActive()) {
            timer->stop();  // Stop the timer
        }
    }



    void Invers();

    void Databone();
    void smoothpoints();

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
    void setupCameraMC(MyMesh& _mesh);
    void setupCamera();
    Vec meanMapColor(double d) const;
    void drawControlNet() const;
    void drawSkleton();
    void drawAxes() const;
    void drawJointAxes(std::vector<Axes>& a) const;
    void drawAxesWithNames() const;
    static Vec intersectLines(const Vec& ap, const Vec& ad, const Vec& bp, const Vec& bd);
    Render render;
    // Other
    void fairMesh();


    void saveMeshToEigen(const MyMesh& _mesh, Eigen::MatrixXd& V);
    void saveMeshFaceToEigen(const MyMesh& _mesh, Eigen::MatrixXi& F);
    void collisonTest(std::vector<Eigen::Vector4d> v);

    //////////////////////
    // Member variables //
    //////////////////////

    enum class ModelType { NONE, MESH, BEZIER_SURFACE, SKELTON, INVERZ } model_type;
    enum class SkelltonType { MAN, WRIST, ARM, FACE } skellton_type;
    // Mesh
    MyMesh mesh;

    bool showSampels;
    bool showSmooth;

    std::vector<int> index;
    void TestDelta(std::vector<Eigen::Vector4d> v);
    void collisonTest2(std::vector<Eigen::Vector4d> v);

    MyMesh smooth;
    MyMesh MushHelper;
    MyMesh Helper;
    

    void SetDistance();

    DeltaMush dm;
    std::vector<float> tios;
    Eigen::SparseMatrix<double> A;
    std::vector<Vec4> delt;
    void DirectMush();
    void AnDirectMush();
    std::vector<std::vector<MyMesh::Point>> seprateSampels;
    std::vector<std::vector<MyMesh::Normal>> normalsofsampels;

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

    void drawMesh();
    Collison col;
    MyMesh mtransperent;

    void drawTransparent();
    ControlPoint target;
    struct SamplePoint {
        MyMesh::FaceHandle tri;
        int cell_id;
        MyMesh::Point pos;
        MyMesh::Normal normal;

        SamplePoint(MyMesh::Point _pos, MyMesh::Normal _normal) { pos = _pos; normal = _normal; }
    };
    
    float random_float(float maximum) { return float((double)rand() / (double)(RAND_MAX / maximum)); }

    void seperateMesh();


    Vec calcCentriod(MyMesh& _mesh);

    std::vector<MyMesh::Point> sampels;

    float generateSamples(int num_samples, MyMesh mesh_, std::vector<SamplePoint>& samples);

    std::vector<MyMesh> im;

    bool is_border_vertex(MyMesh::VertexHandle& vh);

    float f(const float x) { return x * x * x; }


    float li(Vec x, Vec pi) { return distance(x, pi); }
    
    MyMesh::Point e(Vec x, Vec pi){
        Vec d = x - pi;
        float l = li(x, pi);
        Vec result = d / l;
        result *= 3*l*l;// 3x^2

        return MyMesh::Point(result.x, result.y, result.z);

    }

    float c(Vec x, Vec pi) {
        float pow2 = li(x, pi)* li(x, pi);
        float first_part = 1 / pow2;
        float second_derivative = 6 * li(x, pi); // 6x
        float first_derivative = 3 * pow2;// 3x^2
        float second_part = second_derivative - (first_derivative / li(x, pi));

        return first_part * second_part;

    }



    void CalculateImplicitSkinning(std::vector<MyMesh::Point> s, MyMesh& mesh_);


    struct BBox {
        MyMesh::Point min, max;
        BBox()
        {
            min = MyMesh::Point(FLT_MAX, FLT_MAX, FLT_MAX);
            max = MyMesh::Point(-FLT_MAX, -FLT_MAX, -FLT_MAX);
        }

        void add_point(MyMesh::Point& p)
        {
            min[0] = fminf(p[0], min[0]);
            min[1] = fminf(p[1], min[1]);
            min[2] = fminf(p[2], min[2]);
            max[0] = fmaxf(p[0], max[0]);
            max[1] = fmaxf(p[1], max[1]);
            max[2] = fmaxf(p[2], max[2]);
        }
        MyMesh::Point lenghts() { return max - min; }
        MyMesh::Point index_grid_cell(MyMesh::Point res, MyMesh::Point p)
        {
            MyMesh::Point cell_l = lenghts() / res;

            MyMesh::Point lcl = p - min;
            MyMesh::Point idx = lcl / cell_l;
            MyMesh::Point int_idx = MyMesh::Point((int)floorf(idx[0]), (int)floorf(idx[1]), (int)floorf(idx[2]));
            return int_idx;
        }

    };

    struct poisson_sample {
        MyMesh::Point pos;
        MyMesh::Normal normal;
    };

    struct hash_data {
        // Resulting output sample points for this cell:
        std::vector<poisson_sample> poisson_samples;

        // Index into raw_samples:
        int first_sample_idx;
        int sample_cnt;
    };



    float approximate_geodesic_distance(MyMesh::Point p1, MyMesh::Point p2, MyMesh::Normal n1, MyMesh::Normal  n2);


    struct Idx3 {

        MyMesh::Point _size;
        int id;

        Idx3(const MyMesh::Point& size, int idx) : _size(size), id(idx) { }

        Idx3(const MyMesh::Point& size, const MyMesh::Point& pos) : _size(size) {
            set_3d(pos);
        }

        void set_3d(const MyMesh::Point& pos) { id = to_linear(_size, pos[0], pos[1], pos[2]); }


        int to_linear(const MyMesh::Point& size_, int x, int y, int z) {
            return x + size_[0] * (y + size_[1] * z);
        }

        int to_linear() const { return id; }
    };


    void createL(Eigen::SparseMatrix<double>& L);

    std::vector<MyMesh::Point> poissonDisk(float radius, std::vector<SamplePoint> raw, std::vector<MyMesh::Normal>& samples_nor);


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
    MyMesh smoothvectors(std::vector<Vec>& smoothed);
    void smoothoriginal(std::vector<Vec>& smoothed);
    void Delta_Mush(std::vector<Eigen::Vector4d>& v);
    void Delta_Mush_two(std::vector<Eigen::Vector4d> v);

    void drawDelta();
    std::vector<Eigen::Vector4d> setMushFactor(std::vector<Eigen::Vector4d> v);

    void selectedjoin();

    void inverse_kinematics(ControlPoint t, Joint* j);

    // this collect the bones
    std::vector<Bones> b;


    std::vector<Mat4> mteszt;

    void set_bone_matrix()
    {
        for (int i = 0; i < b.size(); i++)
        {
            b[i].M = Mat4();
        }
    }


    // for the animation api (it is simpal)

    Skelton skel;
    Skelton fab;
    int wx = 0;


    std::vector<Vec>FABRIK_p;


    void createL_smooot(MyMesh& m);

    void move(std::vector<Vec> newp, std::vector<Vec> old);

    int elapsedTime;
    std::vector<int>indexes;
    std::vector<Vec> points;
    std::vector<Vec> selected_points_storage;
    float startAnimationTime_ = 0.0;
    float endanimation = 0.0;
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

    void tree_to_array(Joint* j);
    std::vector<Vec> ik;

    std::vector<ControlPoint> cps;

    std::vector<HRBF> hrbf;
    void createControlPoins(Joint* j);
    
    MarchingCubes mc;


    void Error(MyMesh& m, HRBF& h);





    void createCP();

    void IK_matrices(Joint *j);
    double sum_len();
    /// <summary>
    /// 
    /// </summary>
    /// <param name="edgeHandle"></param>

    void setupCameraBone();
    bool mehet = false;
    bool isweight = false;
    bool showJA = false;
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