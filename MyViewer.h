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

using qglviewer::Vec;


class MyViewer : public QGLViewer {
  Q_OBJECT

public:
  explicit MyViewer(QWidget *parent);
  virtual ~MyViewer();

  inline double getCutoffRatio() const;
  inline void setCutoffRatio(double ratio);
  inline double getMeanMin() const;
  inline void setMeanMin(double min);
  inline double getMeanMax() const;
  inline void setMeanMax(double max);
  inline const double *getSlicingDir() const;
  inline void setSlicingDir(double x, double y, double z);
  inline double getSlicingScaling() const;
  inline void setSlicingScaling(double scaling);
  bool openMesh(const std::string &filename, bool update_view = true);
  bool openSkelton(const std::string& filename, bool update_view =true);
  bool openBezier(const std::string &filename, bool update_view = true);
  bool saveBezier(const std::string &filename);
  bool saveBone(const std::string& filename);

signals:
  void startComputation(QString message);
  void midComputation(int percent);
  void endComputation();
  void displayMessage(const QString& message);

protected:
  virtual void init() override;
  virtual void draw() override;
  virtual void drawWithNames() override;
  virtual void postSelection(const QPoint &p) override;
  virtual void keyPressEvent(QKeyEvent *e) override;
  virtual void mouseMoveEvent(QMouseEvent *e) override;
  virtual QString helpString() const override;

private:
  struct MyTraits : public OpenMesh::DefaultTraits {
    using Point  = OpenMesh::Vec3d; // the default would be Vec3f
    using Normal = OpenMesh::Vec3d;
    VertexTraits {
      double mean;              // approximated mean curvature
      std::vector<double> weigh;
      std::vector<double> distance;
      int idx_of_closest_bone;
    };
  };
  using MyMesh = OpenMesh::TriMesh_ArrayKernelT<MyTraits>;
  using Vector = OpenMesh::VectorT<double,3>;

  // Mesh
  void updateMesh(bool update_mean_range = true);
  void updateVertexNormals();
#ifdef USE_JET_FITTING
  void updateWithJetFit(size_t neighbors);
#endif
  void localSystem(const Vector &normal, Vector &u, Vector &v);
  double voronoiWeight(MyMesh::HalfedgeHandle in_he);
  void updateMeanMinMax();
  void updateMeanCurvature();

  // Bezier
  static void bernsteinAll(size_t n, double u, std::vector<double> &coeff);
  void generateMesh(size_t resolution);

  // Visualization
  void setupCamera();
  Vec meanMapColor(double d) const;
  void drawControlNet() const;
  void drawSkleton() ;
  void drawAxes() const;
  void drawAxesWithNames() const;
  static Vec intersectLines(const Vec &ap, const Vec &ad, const Vec &bp, const Vec &bd);

  // Other
  void fairMesh();

  //////////////////////
  // Member variables //
  //////////////////////

  enum class ModelType { NONE, MESH, BEZIER_SURFACE,SKELTON } model_type;
  enum class SkelltonType { MAN,WRIST,ARM } skellton_type;
  // Mesh
  MyMesh mesh;


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



  double tav(Vec p, Vec p1)
  {
      double len = sqrt(pow(p.x - p1.x, 2) + pow(p.y - p1.y, 2) + pow(p.z - p1.z, 2));

      return len;
  }

  void weigh();

  void ininitSkelton();
  void createL(Eigen::SparseMatrix<double>& L);

  void Smooth();
  

  void Rotate();

  void getallpoints(Tree t);

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


  // this collect the bones
  std::vector<Bones> b;
  // this is the skeleton
  Tree sk;

  std::vector<int>indexes;
  std::vector<Vec> points;
  std::vector<Vec> ve;
  // Bezier
  size_t degree[2];
  std::vector<Vec> control_points;
  int wi = 2;
  void setupCameraBone();
  bool mehet = false;
  bool isweight = false;
  // Visualization
  double mean_min, mean_max, cutoff_ratio;
  bool show_control_points, show_solid, show_wireframe,show_skelton;
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
};

#include "MyViewer.hpp"
