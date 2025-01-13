// -*- mode: c++ -*-
#pragma once
#include <string>
#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QGLViewer/quaternion.h>
#include "Skelton.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include <QtGui/QKeyEvent>
#include <QtWidgets>
#include <map>
#include <algorithm>
#include "HRBF.h"
#include "MarchingCubes.h"
#include "DeltaMush.h"
#include "mclccd\BVHTree.hpp"
#include "ControlPoint.h"
#include "Render.h"
#include <QOpenGLWidget>
#include "Collison.h"
#include "KinectSkelton.h"
#include <QTimer>
#include "ImplicitSkinning.h"
#include "Object3D.h"
//#include "BoneHeat.h"

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
    void skining();
    void Laplace() {
        smooth = mesh;
        bright = 0.1;
        transparent = true;
        createL_smooot(smooth); update(); 
    }
    void delta();
    void poission() {  }

    void Boneheat();

    void setSlider(int value) {
        deltaMushFactor = (float)value/100.0f;
        dm.deltaMushFactor = deltaMushFactor;
       
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
 
    bool Mydelta;

    void DeltaMush2(std::vector<Eigen::Vector4d> v);


    float  Epsil();

    KinectSkelton kinect;
    QTimer* timer;
    bool transparent = false;
    bool transparent2 = true;
    float& getFrameSecond() { return FrameSecond; }
    double epsilon = 0.001;
    void Reset();
    int wi = 2;
    void index_of_weight() {
        if (mesh.n_vertices() != 0)
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


    void improveDeltaMush();

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
    virtual void endSelection(const QPoint& point) override;
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
    void addObject(std::shared_ptr<Object3D> object);
    void addObjects(const std::vector<std::shared_ptr<Object3D>>& newObjects);

    Render render;
    // Other
    void fairMesh();

    //////////////////////
    // Member variables //
    //////////////////////

    std::vector<std::shared_ptr<Object3D>> objects;
    size_t selected_object;
    Vis::Visualization vis;
    enum class ModelType { NONE, MESH, BEZIER_SURFACE, SKELTON, INVERZ } model_type;
    enum class SkelltonType { MAN, WRIST, ARM, FACE } skellton_type;
    // Mesh
    MyMesh mesh;
    bool showSampels;
    bool showSmooth;
    int controlPointId = 0;
    std::vector<int> index;
    void TestDelta(std::vector<Eigen::Vector4d> v);
    MyMesh smooth;
    MyMesh MushHelper;
    MyMesh Helper;
    void SetDistance();
    DeltaMush dm;
    std::vector<float> tios;
    Eigen::SparseMatrix<double> A;
    std::vector<Vec4> delt;
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
    Vec calcCentriod(MyMesh& _mesh);
    std::vector<MyMesh::Point> sampels;
    std::vector<MyMesh> im;

    bool is_border_vertex(MyMesh::VertexHandle& vh);
    void createL(Eigen::SparseMatrix<double>& L);
    float FrameSecond = 0.0;
    QHBoxLayout* hb1 = new QHBoxLayout;
    QLabel* text_ = new QLabel;
    QVBoxLayout* vBox = new QVBoxLayout;
    std::vector<std::pair<int, double>> sortedVector;
    std::vector<std::pair<int, double>> finalarea;
    std::map< MyMesh::FaceHandle, int> sortedMap;
    std::vector<Eigen::Vector4d> vec;
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

    // for the animation api (it is simpal)
    Skelton skel;
    Skelton fab;
    int wx = 0;
    std::vector<Vec>FABRIK_p;
    void createL_smooot(MyMesh& m);
    int elapsedTime;

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
    std::vector<HRBF> hrbf;
    void createControlPoins(Joint* j);
    MarchingCubes mc;
    void Error(MyMesh& m, HRBF& h);
    void createCP();
    void IK_matrices(Joint *j);
    double sum_len();
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