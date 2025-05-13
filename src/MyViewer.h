// -*- mode: c++ -*-
#pragma once
#include <string>
#include <QGLViewer/qglviewer.h>
#include <OpenMesh/Core/Mesh/TriMesh_ArrayKernelT.hh>
#include <QFile>
#include <QDir>
#include <QTextStream>
#include <QGLViewer/quaternion.h>
#include "skeleton/Skelton.h"
#include <fstream>
#include <iostream>
#include <vector>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <QtGui/QKeyEvent>

#include <map>
#include <algorithm>
#include "HRBF.h"
#include "MarchingCubes.h"
#include "skinning/DeltaMush.h"
#include "ControlPoint.h"
#include "Render.h"
#include <QGLViewer/qglviewer.h>
#include "KinectSkelton.h"
#include <QTimer>
#include "skinning/ImplicitSkinning.h"
#include "Object3D.h"
#include "Sculpt.h"
#include "Blendshape.h"
#include <QtOpenGLWidgets/QOpenGLWidget>

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
    int getbone_size() { return 0; }
    void setMesh() { model_type = ModelType::MESH; }
    void setBone() { model_type = ModelType::SKELTON; }
    void selectedvert();
    void wierframe() {
        show_wireframe = !show_wireframe;
        update();
    }
    float bright = 0.5;
    float deltaMushFactor = 1.0;
    void skining();
    void Laplace() {
        bright = 0.1;
        update();
    }
    void delta();
    void poission() {  }

    void Boneheat();

    void setSlider(int value);

    float Epsil();


    

    KinectSkelton kinect;
    QTimer* timer;
    float& getFrameSecond() { return FrameSecond; }
    double epsilon = 0.001;
    void Reset();
    void index_of_weight();
    void show();

    void callKinekcnUpdate();
    void improveDeltaMush();
    void startTimer();
    void stopTimer();
    void Frame();
    Vec angels;
    Vec ang;
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
    void localSystem(const Vector& normal, Vector& u, Vector& v);
    void updateMeanMinMax();
    // Bezier
    static void bernsteinAll(size_t n, double u, std::vector<double>& coeff);
    void generateMesh(size_t resolution);

    // Visualization
    void setupCamera();
    Vec meanMapColor(double d) const;
    void drawControlNet() const;
    void drawAxes() const;
    void drawJointAxes(std::vector<Axes>& a) const;
    void drawAxesWithNames() const;
    static Vec intersectLines(const Vec& ap, const Vec& ad, const Vec& bp, const Vec& bd);
    void addObject(std::shared_ptr<Object3D> object);
    void addObjects(const std::vector<std::shared_ptr<Object3D>>& newObjects);
    void moveObject(std::shared_ptr<Object3D> elemt, int selected, Vector position);
    void createSculpt();
    void disconnectProcrecBar(std::shared_ptr<Skinning> skinning);
    void connectProcrecBar(std::shared_ptr<Skinning> skinning);
    void drawCircle(float centerX, float centerY, float radius);
    Render render;
    Vec fa;
    //////////////////////
    // Member variables //
    //////////////////////
    std::vector<std::shared_ptr<Object3D>> objects;
    std::vector<std::shared_ptr<Skelton>> skeltons;
    size_t selected_object;
    Vis::Visualization vis;
    enum class ModelType { NONE, MESH, BEZIER_SURFACE, SKELTON, INVERZ } model_type;
    // Mesh
    MyMesh mesh;
    int controlPointId = 0;
    float FrameSecond = 0.0;
    std::vector<Eigen::Vector4d> vec;
    void drawDelta();
    void selectedjoin();
    // for the animation api (it is simpal)
    int elapsedTime;
    float startAnimationTime_ = 0.0;
    float endanimation = 0.0;
    float animationDuration_ = 1.0;
    std::vector<Keyframe> keyframes_;
    bool isAnimating_;
    Vec rotation;
    // Bezier
    size_t degree[2];
    std::vector<Vec> control_points;
    float currentTime();
    Vec calcCentriod(MyMesh& _mesh);
    void createControlPoins(Joint* j);
    MarchingCubes mc;
    std::unique_ptr<Sculpt> sculpt;
    std::unique_ptr<Blendshape> blend;
    void createCP();
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
// id� kivonva a m�sikb�l
#include "MyViewer.hpp"