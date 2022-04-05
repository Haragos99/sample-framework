#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <map>
#include <vector>
#include "Bone.h"
#include <QtGui/QKeyEvent>
#include <QtWidgets>
#include <OpenMesh/Core/IO/MeshIO.hh>
#include <OpenMesh/Tools/Smoother/JacobiLaplaceSmootherT.hh>
#include <QGLViewer/quaternion.h>

#ifdef BETTER_MEAN_CURVATURE
#include "Eigen/Eigenvalues"
#include "Eigen/Geometry"
#include "Eigen/LU"
#include "Eigen/SVD"
#endif

#ifdef USE_JET_FITTING
#include "jet-wrapper.h"
#endif

#include "MyViewer.h"

#ifdef _WIN32
#define GL_CLAMP_TO_EDGE 0x812F
#define GL_BGRA 0x80E1
#endif

MyViewer::MyViewer(QWidget *parent) :
  QGLViewer(parent), model_type(ModelType::NONE),
  mean_min(0.0), mean_max(0.0), cutoff_ratio(0.05),
  show_control_points(true), show_solid(true), show_wireframe(false),
  visualization(Visualization::PLAIN), slicing_dir(0, 0, 1), slicing_scaling(1),
  last_filename("")
{
    QDir* logdir = new QDir();
    logdir->mkdir("logs");
    QString log_path("logs/log_");
    log_path += QDateTime::currentDateTime().toString("yyyy-MM-dd__hh-mm-ss-zzz") + ".txt";
    std::ofstream of = std::ofstream(log_path.toStdString());
    std::cerr.rdbuf(of.rdbuf());
    omerr().rdbuf(of.rdbuf());
  setSelectRegionWidth(10);
  setSelectRegionHeight(10);
  axes.shown = false;
}

MyViewer::~MyViewer() {
  glDeleteTextures(1, &isophote_texture);
  glDeleteTextures(1, &environment_texture);
  glDeleteTextures(1, &slicing_texture);
}

void MyViewer::updateMeanMinMax() {
  size_t n = mesh.n_vertices();
  if (n == 0)
    return;

  std::vector<double> mean;
  mean.reserve(n);
  for (auto v : mesh.vertices())
    mean.push_back(mesh.data(v).mean);

  std::sort(mean.begin(), mean.end());
  size_t k = (double)n * cutoff_ratio;
  mean_min = std::min(mean[k ? k-1 : 0], 0.0);
  mean_max = std::max(mean[k ? n-k : n-1], 0.0);
}

void MyViewer::localSystem(const MyViewer::Vector &normal,
                           MyViewer::Vector &u, MyViewer::Vector &v) {
  // Generates an orthogonal (u,v) coordinate system in the plane defined by `normal`.
  int maxi = 0, nexti = 1;
  double max = std::abs(normal[0]), next = std::abs(normal[1]);
  if (max < next) {
    std::swap(max, next);
    std::swap(maxi, nexti);
  }
  if (std::abs(normal[2]) > max) {
    nexti = maxi;
    maxi = 2;
  } else if (std::abs(normal[2]) > next)
    nexti = 2;

  u.vectorize(0.0);
  u[nexti] = -normal[maxi];
  u[maxi] = normal[nexti];
  u /= u.norm();
  v = normal % u;
}

double MyViewer::voronoiWeight(MyViewer::MyMesh::HalfedgeHandle in_he) {
  // Returns the area of the triangle bounded by in_he that is closest
  // to the vertex pointed to by in_he.
  if (mesh.is_boundary(in_he))
    return 0;
  auto next = mesh.next_halfedge_handle(in_he);
  auto prev = mesh.prev_halfedge_handle(in_he);
  double c2 = mesh.calc_edge_vector(in_he).sqrnorm();
  double b2 = mesh.calc_edge_vector(next).sqrnorm();
  double a2 = mesh.calc_edge_vector(prev).sqrnorm();
  double alpha = mesh.calc_sector_angle(in_he);

  if (a2 + b2 < c2)                // obtuse gamma
    return 0.125 * b2 * std::tan(alpha);
  if (a2 + c2 < b2)                // obtuse beta
    return 0.125 * c2 * std::tan(alpha);
  if (b2 + c2 < a2) {              // obtuse alpha
    double b = std::sqrt(b2), c = std::sqrt(c2);
    double total_area = 0.5 * b * c * std::sin(alpha);
    double beta  = mesh.calc_sector_angle(prev);
    double gamma = mesh.calc_sector_angle(next);
    return total_area - 0.125 * (b2 * std::tan(gamma) + c2 * std::tan(beta));
  }

  double r2 = 0.25 * a2 / std::pow(std::sin(alpha), 2); // squared circumradius
  auto area = [r2](double x2) {
    return 0.125 * std::sqrt(x2) * std::sqrt(std::max(4.0 * r2 - x2, 0.0));
  };
  return area(b2) + area(c2);
}

#ifndef BETTER_MEAN_CURVATURE
void MyViewer::updateMeanCurvature() {
  std::map<MyMesh::FaceHandle, double> face_area;
  std::map<MyMesh::VertexHandle, double> vertex_area;

  for (auto f : mesh.faces())
    face_area[f] = mesh.calc_sector_area(mesh.halfedge_handle(f));

  // Compute triangle strip areas
  for (auto v : mesh.vertices()) {
    vertex_area[v] = 0;
    mesh.data(v).mean = 0;
    for (auto f : mesh.vf_range(v))
      vertex_area[v] += face_area[f];
    vertex_area[v] /= 3.0;
  }

  // Compute mean values using dihedral angles
  for (auto v : mesh.vertices()) {
    for (auto h : mesh.vih_range(v)) {
      auto vec = mesh.calc_edge_vector(h);
      double angle = mesh.calc_dihedral_angle(h); // signed; returns 0 at the boundary
      mesh.data(v).mean += angle * vec.norm();
    }
    mesh.data(v).mean *= 0.25 / vertex_area[v];
  }
}
#else // BETTER_MEAN_CURVATURE
void MyViewer::updateMeanCurvature() {
  // As in the paper:
  //   S. Rusinkiewicz, Estimating curvatures and their derivatives on triangle meshes.
  //     3D Data Processing, Visualization and Transmission, IEEE, 2004.

  std::map<MyMesh::VertexHandle, Vector> efgp; // 2nd principal form
  std::map<MyMesh::VertexHandle, double> wp;   // accumulated weight

  // Initial setup
  for (auto v : mesh.vertices()) {
    efgp[v].vectorize(0.0);
    wp[v] = 0.0;
  }

  for (auto f : mesh.faces()) {
    // Setup local edges, vertices and normals
    auto h0 = mesh.halfedge_handle(f);
    auto h1 = mesh.next_halfedge_handle(h0);
    auto h2 = mesh.next_halfedge_handle(h1);
    auto e0 = mesh.calc_edge_vector(h0);
    auto e1 = mesh.calc_edge_vector(h1);
    auto e2 = mesh.calc_edge_vector(h2);
    auto n0 = mesh.normal(mesh.to_vertex_handle(h1));
    auto n1 = mesh.normal(mesh.to_vertex_handle(h2));
    auto n2 = mesh.normal(mesh.to_vertex_handle(h0));

    Vector n = mesh.normal(f), u, v;
    localSystem(n, u, v);

    // Solve a LSQ equation for (e,f,g) of the face
    Eigen::MatrixXd A(6, 3);
    A << (e0 | u), (e0 | v),    0.0,
            0.0,   (e0 | u), (e0 | v),
         (e1 | u), (e1 | v),    0.0,
            0.0,   (e1 | u), (e1 | v),
         (e2 | u), (e2 | v),    0.0,
            0.0,   (e2 | u), (e2 | v);
    Eigen::VectorXd b(6);
    b << ((n2 - n1) | u),
         ((n2 - n1) | v),
         ((n0 - n2) | u),
         ((n0 - n2) | v),
         ((n1 - n0) | u),
         ((n1 - n0) | v);
    Eigen::Vector3d x = A.fullPivLu().solve(b);

    Eigen::Matrix2d F;          // Fundamental matrix for the face
    F << x(0), x(1),
         x(1), x(2);

    for (auto h : mesh.fh_range(f)) {
      auto p = mesh.to_vertex_handle(h);

      // Rotate the (up,vp) local coordinate system to be coplanar with that of the face
      Vector np = mesh.normal(p), up, vp;
      localSystem(np, up, vp);
      auto axis = (np % n).normalize();
      double angle = std::acos(std::min(std::max(n | np, -1.0), 1.0));
      auto rotation = Eigen::AngleAxisd(angle, Eigen::Vector3d(axis.data()));
      Eigen::Vector3d up1(up.data()), vp1(vp.data());
      up1 = rotation * up1;    vp1 = rotation * vp1;
      up = Vector(up1.data()); vp = Vector(vp1.data());

      // Compute the vertex-local (e,f,g)
      double e, f, g;
      Eigen::Vector2d upf, vpf;
      upf << (up | u), (up | v);
      vpf << (vp | u), (vp | v);
      e = upf.transpose() * F * upf;
      f = upf.transpose() * F * vpf;
      g = vpf.transpose() * F * vpf;

      // Accumulate the results with Voronoi weights
      double w = voronoiWeight(h);
      efgp[p] += Vector(e, f, g) * w;
      wp[p] += w;
    }
  }

  // Compute the principal curvatures
  for (auto v : mesh.vertices()) {
    auto &efg = efgp[v];
    efg /= wp[v];
    Eigen::Matrix2d F;
    F << efg[0], efg[1],
         efg[1], efg[2];
    auto k = F.eigenvalues();   // always real, because F is a symmetric real matrix
    mesh.data(v).mean = (k(0).real() + k(1).real()) / 2.0;
  }
}
#endif

static Vec HSV2RGB(Vec hsv) {
  // As in Wikipedia
  double c = hsv[2] * hsv[1];
  double h = hsv[0] / 60;
  double x = c * (1 - std::abs(std::fmod(h, 2) - 1));
  double m = hsv[2] - c;
  Vec rgb(m, m, m);
  if (h <= 1)
    return rgb + Vec(c, x, 0);
  if (h <= 2)
    return rgb + Vec(x, c, 0);
  if (h <= 3)
    return rgb + Vec(0, c, x);
  if (h <= 4)
    return rgb + Vec(0, x, c);
  if (h <= 5)
    return rgb + Vec(x, 0, c);
  if (h <= 6)
    return rgb + Vec(c, 0, x);
  return rgb;
}

Vec MyViewer::meanMapColor(double d) const {
  double red = 0, green = 120, blue = 240; // Hue
  if (d < 0) {
    double alpha = mean_min ? std::min(d / mean_min, 1.0) : 1.0;
    return HSV2RGB({green * (1 - alpha) + blue * alpha, 1, 1});
  }
  double alpha = mean_max ? std::min(d / mean_max, 1.0) : 1.0;
  return HSV2RGB({green * (1 - alpha) + red * alpha, 1, 1});
}

void MyViewer::fairMesh() {
  if (model_type != ModelType::MESH)
    return;

  emit startComputation(tr("Fairing mesh..."));
  OpenMesh::Smoother::JacobiLaplaceSmootherT<MyMesh> smoother(mesh);
  smoother.initialize(OpenMesh::Smoother::SmootherT<MyMesh>::Normal, // or: Tangential_and_Normal
                      OpenMesh::Smoother::SmootherT<MyMesh>::C1);
  for (size_t i = 1; i <= 10; ++i) {
    smoother.smooth(10);
    emit midComputation(i * 10);
  }
  updateMesh(false);
  emit endComputation();
}

#ifdef USE_JET_FITTING

void MyViewer::updateWithJetFit(size_t neighbors) {
  std::vector<Vector> points;
  for (auto v : mesh.vertices())
    points.push_back(mesh.point(v));

  auto nearest = JetWrapper::Nearest(points, neighbors);

  for (auto v : mesh.vertices()) {
    auto jet = JetWrapper::fit(mesh.point(v), nearest, 2);
    if ((mesh.normal(v) | jet.normal) < 0) {
      mesh.set_normal(v, -jet.normal);
      mesh.data(v).mean = (jet.k_min + jet.k_max) / 2;
    } else {
      mesh.set_normal(v, jet.normal);
      mesh.data(v).mean = -(jet.k_min + jet.k_max) / 2;
    }
  }
}

#endif // USE_JET_FITTING

void MyViewer::updateVertexNormals() {
  // Weights according to:
  //   N. Max, Weights for computing vertex normals from facet normals.
  //     Journal of Graphics Tools, Vol. 4(2), 1999.
  for (auto v : mesh.vertices()) {
    Vector n(0.0, 0.0, 0.0);
    for (auto h : mesh.vih_range(v)) {
      if (mesh.is_boundary(h))
        continue;
      auto in_vec  = mesh.calc_edge_vector(h);
      auto out_vec = mesh.calc_edge_vector(mesh.next_halfedge_handle(h));
      double w = in_vec.sqrnorm() * out_vec.sqrnorm();
      n += (in_vec % out_vec) / (w == 0.0 ? 1.0 : w);
    }
    double len = n.length();
    if (len != 0.0)
      n /= len;
    mesh.set_normal(v, n);
  }
}

void MyViewer::updateMesh(bool update_mean_range) {
  if (model_type == ModelType::BEZIER_SURFACE)
    generateMesh(50);
  mesh.request_face_normals(); mesh.request_vertex_normals();
  mesh.update_face_normals();
#ifdef USE_JET_FITTING
  mesh.update_vertex_normals();
  updateWithJetFit(20);
#else // !USE_JET_FITTING
  updateVertexNormals();
  updateMeanCurvature();
#endif
  if (update_mean_range)
    updateMeanMinMax();
}


void MyViewer::setupCameraBone() {
    // Set camera on the model
    Vector box_min, box_max;
    box_min = box_max = Vector(points.front().x, points.front().y, points.front().z);
    for (auto v : points) {
        box_min.minimize(Vector(v.x, v.y, v.z));
        box_max.maximize(Vector(v.x, v.y, v.z));
    }
    camera()->setSceneBoundingBox(Vec(box_min.data()), Vec(box_max.data()));
    camera()->showEntireScene();

    slicing_scaling = 20 / (box_max - box_min).max();

    setSelectedName(-1);
    axes.shown = false;

    update();
}


void MyViewer::setupCamera() {
  // Set camera on the model
  Vector box_min, box_max;
  box_min = box_max = mesh.point(*mesh.vertices_begin());
  for (auto v : mesh.vertices()) {
    box_min.minimize(mesh.point(v));
    box_max.maximize(mesh.point(v));
  }
  camera()->setSceneBoundingBox(Vec(box_min.data()), Vec(box_max.data()));
  camera()->showEntireScene();

  slicing_scaling = 20 / (box_max - box_min).max();

  setSelectedName(-1);
  axes.shown = false;

  update();
}

bool MyViewer::openMesh(const std::string &filename, bool update_view) {
  if (!OpenMesh::IO::read_mesh(mesh, filename) || mesh.n_vertices() == 0)
    return false;
  model_type = ModelType::MESH;
  last_filename = filename;
  updateMesh(update_view);

  if (update_view)
    setupCamera();

  return true;
}

void MyViewer::ininitSkelton()
{

    int size = indexes.size();
    for (int i = 0; i < size; i+=2)
    {
        Bones bo;
        bo.start = points[indexes[i] -1];
        bo.End = points[indexes[i + 1]-1];
        b.push_back(bo);


    }

    for (int i = 0; i < b.size(); i++)
    {
        b[i].setColor(colors_bone[i].x, colors_bone[i].y, colors_bone[i].z);
        b[i].manypoints();
    }
        
    if (skellton_type == SkelltonType::MAN) {
        manSkellton();
    }
    if (skellton_type == SkelltonType::WRIST) {
        csukloSkellton();
    }
    if(skellton_type == SkelltonType::ARM)
    {
        armSkellton();
    }

}


void MyViewer::weigh()
{
      int indexof=0;
 

      for (auto v : mesh.vertices())
      {
          double min_val = std::numeric_limits<double>::infinity();
          mesh.data(v).weigh.clear();
          for (int i = 0; i < b.size(); i++)
          {
              mesh.data(v).weigh.push_back(0);
          }
          // az aktuális pont
          Vec d = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
          for (int i = 0; i < b.size(); i++)
          {
              double f = std::numeric_limits<double>::infinity(); ;
              for (int j = 0; j < b[i].points.size(); j++)
              {
                 // printf("%d %d", i, j);
                 double t = tav( b[i].points[j],d);
                
                 // legközelebbi súly
                 if (t < min_val)
                 {
                     min_val = t;
                     indexof = i;
                 }
                 // csonton lévõ legközelebbi távolság
                 if (t < f)
                 {
                     f = t;
                 }
                 
                 
              }
              mesh.data(v).tavolsag.push_back(f);
          }
          mesh.data(v).weigh[indexof] = 1;
      }


  
}


void MyViewer::Smooth()
{
    int n = mesh.n_vertices();
    // az aktuálos pont folyamatosan növeljük
    Eigen::SparseMatrix<double> L = createL();
    for (int k = 0; k < b.size(); k++)
    {
        int i = 0;
        Eigen::VectorXd w1(n);
        w1 = Eigen::VectorXd::Zero(n); // Feltöltjük 0-kkal
        Eigen::SparseMatrix<double> D(n, n);
        for (auto v : mesh.vertices()) {


                w1[i] = mesh.data(v).weigh[k];
                
                double er = 1 / pow(mesh.data(v).tavolsag[k],2);
                
                // a D mátrix átloja feltöltése
                D.coeffRef(i, i) = er;
                i++;
        }
        D.makeCompressed();
        Eigen::SparseMatrix<double> O = L + D;
        Eigen::VectorXd Sw1(n);
        Eigen::SparseLU< Eigen::SparseMatrix<double> > solver;
        Sw1 = D * w1;
        solver.compute(O); // M inverz mátrixának (valójában: "LU felbontásának") kiszámítása

        if (solver.info() == Eigen::Success) { // Teszteljük, hogy sikerült-e invertálni

            Eigen::VectorXd x = solver.solve(Sw1); // Megoldjuk az M*x = v egyenletrendszert
            
            if (solver.info() == Eigen::Success) { // Teszteljük, hogy sikerült-e megoldani

                int j = 0;
                for (auto v : mesh.vertices()) {
                    
                    mesh.data(v).weigh[k] = x[j];
                    //omerr() << "Ez egy szöveg, ez egy szám: " << x[j] << std::endl;
                    j++;
                }

            }
        }

    }
    /*
    for (auto v : mesh.vertices()) {
        double d = mesh.data(v).weigh[0] + mesh.data(v).weigh[1] + mesh.data(v).weigh[2];
        if (d != 1)
        {
            
        }
    }
    */
}

Eigen::SparseMatrix<double> MyViewer::createL()
{
    
    int n = mesh.n_vertices();
    Eigen::SparseMatrix<double> L(n, n);
    int i = 0;
    for (auto v : mesh.vertices()) {

        
        for (auto vi : mesh.vv_range(v)) { // v vertex-szel szomszédos vertexek


           auto eij = mesh.find_halfedge(v, vi);
          auto m1= mesh.next_halfedge_handle(eij);
          double theta1 = mesh.calc_sector_angle(m1);


           auto eji= mesh.find_halfedge(vi, v);
           auto m2= mesh.next_halfedge_handle(eji);
           double theta2 = mesh.calc_sector_angle(m2);

           double result = (tan(M_PI_2 - theta1) + tan(M_PI_2 - theta2)) / 2;
           if (i == vi.idx()) continue;
           L.coeffRef(i, vi.idx()) = result;
        }


        i++;
    }
    for (int i = 0; i < n; i++)
    {
       
        L.coeffRef(i, i) = -L.row(i).sum();
    }
    L.makeCompressed();

    return L;

}



bool MyViewer::openSkelton(const std::string& filename, bool update_view)
{
   
    if (filename.find("csuk") != std::string::npos) {
        skellton_type = SkelltonType::WRIST;

    }
    else if(filename.find("arm") != std::string::npos)
    {
        skellton_type = SkelltonType::ARM;
    }
    else if (filename.find("man") != std::string::npos)
    {
        skellton_type = SkelltonType::MAN;
    }
    else
    {
        return false;
    }
    std::string myText;
    std::vector<double> linepoint;
    // Read from the text file
    std::ifstream MyReadFile(filename);
    bool isindex = false;
    // Use a while loop together with the getline() function to read the file line by line
    while (std::getline(MyReadFile, myText)) {
        size_t pos = 0;
        std::string delimiter = ";";
        std::string token;

        if (myText != "#")
        {
            while ((pos = myText.find(delimiter)) != std::string::npos) {
                token = myText.substr(0, pos);
                std::cout << token << " ";
                if (isindex)
                {
                    indexes.push_back(std::stoi(token));
                }
                if (pos != 1 && !isindex)
                {
                    linepoint.push_back(std::stod(token));
                }
                myText.erase(0, pos + delimiter.length());
            }
            std::cout << '\n';
        }
        else {
            std::cout << myText << '\n';
            isindex = true;
        }
        if (!isindex)
        {
            points.push_back(Vec(linepoint[0], linepoint[1], linepoint[2]));
            linepoint.clear();
        }
    }

    // Close the file
    MyReadFile.close();
    model_type = ModelType::SKELTON;
    last_filename = filename;
    ininitSkelton();
    updateMesh(update_view);
    if (update_view)
        setupCameraBone();

    return true;
}

bool MyViewer::openBezier(const std::string &filename, bool update_view) {
  size_t n, m;
  try {
    std::ifstream f(filename.c_str());
    f.exceptions(std::ios::failbit | std::ios::badbit);
    f >> n >> m;
    degree[0] = n++; degree[1] = m++;
    control_points.resize(n * m);
    for (size_t i = 0, index = 0; i < n; ++i)
      for (size_t j = 0; j < m; ++j, ++index)
        f >> control_points[index][0] >> control_points[index][1] >> control_points[index][2];
  } catch(std::ifstream::failure &) {
    return false;
  }
  model_type = ModelType::BEZIER_SURFACE;
  last_filename = filename;
  updateMesh(update_view);
  if (update_view)
    setupCamera();
  return true;
}



bool MyViewer::saveBone(const std::string& filename) {
    if (model_type != ModelType::SKELTON)
        return false;

    try {
        getallpoints(sk);
        points = ve;
        ve.clear();
        std::ofstream f(filename.c_str());
        f.exceptions(std::ios::failbit | std::ios::badbit);
        for (const auto& p : points)
            f << 'b'<< ';' << p[0] << ';' << p[1] << ';' << p[2] << ';' << std::endl;
        f << '#' << std::endl;
        for (int i = 0; i < indexes.size(); i+=2)
        {
            f << indexes[i] << ';' << indexes[i + 1] << ';'<< std::endl;
        }
        f << '#' << std::endl;
    }
    catch (std::ifstream::failure&) {
        return false;
    }
    return true;
}


bool MyViewer::saveBezier(const std::string &filename) {
  if (model_type != ModelType::BEZIER_SURFACE)
    return false;

  try {
    std::ofstream f(filename.c_str());
    f.exceptions(std::ios::failbit | std::ios::badbit);
    f << degree[0] << ' ' << degree[1] << std::endl;
    for (const auto &p : control_points)
      f << p[0] << ' ' << p[1] << ' ' << p[2] << std::endl;
  } catch(std::ifstream::failure &) {
    return false;
  }
  return true;
}

void MyViewer::init() {
  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

  QImage img(":/isophotes.png");
  glGenTextures(1, &isophote_texture);
  glBindTexture(GL_TEXTURE_2D, isophote_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img.width(), img.height(), 0, GL_BGRA,
               GL_UNSIGNED_BYTE, img.convertToFormat(QImage::Format_ARGB32).bits());

  QImage img2(":/environment.png");
  glGenTextures(1, &environment_texture);
  glBindTexture(GL_TEXTURE_2D, environment_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, img2.width(), img2.height(), 0, GL_BGRA,
               GL_UNSIGNED_BYTE, img2.convertToFormat(QImage::Format_ARGB32).bits());

  glGenTextures(1, &slicing_texture);
  glBindTexture(GL_TEXTURE_1D, slicing_texture);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_1D, GL_TEXTURE_WRAP_S, GL_REPEAT);
  static const unsigned char slicing_img[] = { 0b11111111, 0b00011100 };
  glTexImage1D(GL_TEXTURE_1D, 0, GL_RGB, 2, 0, GL_RGB, GL_UNSIGNED_BYTE_3_3_2, &slicing_img);
}

void MyViewer::draw() {
  if (model_type == ModelType::BEZIER_SURFACE && show_control_points)
    drawControlNet();
  if (model_type == ModelType::SKELTON)
  {

  }
  if(show_skelton)
      drawSkleton();

  glPolygonMode(GL_FRONT_AND_BACK, !show_solid && show_wireframe ? GL_LINE : GL_FILL);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1, 1);

  if (show_solid || show_wireframe) {
    if (visualization == Visualization::PLAIN)
      glColor3d(1.0, 1.0, 1.0);
    else if (visualization == Visualization::ISOPHOTES) {
      glBindTexture(GL_TEXTURE_2D, current_isophote_texture);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
      glEnable(GL_TEXTURE_2D);
      glTexGeni(GL_S, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
      glTexGeni(GL_T, GL_TEXTURE_GEN_MODE, GL_SPHERE_MAP);
      glEnable(GL_TEXTURE_GEN_S);
      glEnable(GL_TEXTURE_GEN_T);
    } else if (visualization == Visualization::SLICING) {
      glBindTexture(GL_TEXTURE_1D, slicing_texture);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_DECAL);
      glEnable(GL_TEXTURE_1D);
    }
   
    for (auto f : mesh.faces()) {
      glBegin(GL_POLYGON);
      for (auto v : mesh.fv_range(f)) {
        if (visualization == Visualization::MEAN)
          glColor3dv(meanMapColor(mesh.data(v).mean));
        else if (visualization == Visualization::SLICING)
          glTexCoord1d(mesh.point(v) | slicing_dir * slicing_scaling);
        else if (visualization == Visualization::WEIGH) //Itt adjuk meg a súlyokat
        {

            Vec color = Vec(0,0,0);
            for (int i = 0; i < b.size(); i++)
            {
                if (mesh.data(v).weigh[i] != 0)
                {
                    color += mesh.data(v).weigh[i] * b[i].getColor();
   
                }
                


            }
            glColor3d(color.x, color.y, color.z);
        }
        else if (visualization == Visualization::WEIGH2)
        {
            if (wi == b.size())
            {
                wi = 0;
            }
           Vec color = mesh.data(v).weigh[wi] * b[wi].getColor();

            glColor3d(color.x, color.y, color.z);
        }
        glNormal3dv(mesh.normal(v).data());
        glVertex3dv(mesh.point(v).data());
        
       
      }
      glEnd();
    }
    if (visualization == Visualization::ISOPHOTES) {
      glDisable(GL_TEXTURE_GEN_S);
      glDisable(GL_TEXTURE_GEN_T);
      glDisable(GL_TEXTURE_2D);
      glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
    } else if (visualization == Visualization::SLICING) {
      glDisable(GL_TEXTURE_1D);
    }
  }

  if (show_solid && show_wireframe) {
    glPolygonMode(GL_FRONT, GL_LINE);
    glColor3d(0.0, 0.0, 0.0);
    glDisable(GL_LIGHTING);
    for (auto f : mesh.faces()) {
      glBegin(GL_POLYGON);
      for (auto v : mesh.fv_range(f))
        glVertex3dv(mesh.point(v).data());
      glEnd();
    }
    glEnable(GL_LIGHTING);
  }

  if (axes.shown)
    drawAxes();
}
/// <summary>
/// ezt rajzoljuk ku
/// </summary>
void MyViewer::drawSkleton()
{

    glPointSize(30.0);
    glColor3d(0.3, 0.0, 1.0);
    glLineWidth(200.0);
    int i = 0;
    
    for (const auto& p : b)
    {
        if(i<b.size())
        {
        glBegin(GL_LINES);
            glColor3d(p.x, p.y, p.z);
            glVertex3dv(p.start);
            glVertex3dv(p.End);
        glEnd();
        }
        i++;

    }
    
    sk.drawchild(sk);
   /*

    glColor3d(1.0, 0.0, 1.0);
    glPointSize(50.0);
    glBegin(GL_POINTS);
    
    for (const auto& p : points)
    {

        
        glVertex3dv(p);
        
    }

    glEnd();
    */
    //glColor3d(0.3, 0.0, 1.0);
    //glPointSize(60.0);
    //glBegin(GL_POINTS);
    //for (const auto& p : b)
    //{
    //    for (int i = 0; i < p.points.size(); i++)
    //    {
    //        glColor3d(p.x, p.y, p.z);
    //       // glVertex3dv(p.points[i]);
    //    }
    //    
    //}

    //glEnd();
}



void MyViewer::Rotate()
{

    
    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout,
        * hb2 = new QHBoxLayout,
        * hb3 = new QHBoxLayout,
        * hb4 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;

    auto* text_H = new QLabel(tr("X: "));
    auto* text_B = new QLabel(tr("Y: "));
    auto* text_P = new QLabel(tr("Z: "));
    auto* sb_H = new QDoubleSpinBox;
    auto* sb_B = new QDoubleSpinBox;
    auto* sb_P = new QDoubleSpinBox;
    auto* cancel = new QPushButton(tr("Cancel"));
    auto* ok = new QPushButton(tr("Ok"));
    connect(cancel, SIGNAL(pressed()), dlg.get(), SLOT(reject()));
    connect(ok, SIGNAL(pressed()), dlg.get(), SLOT(accept()));
    ok->setDefault(true);

    sb_H->setRange(-360, 360);
    sb_P->setRange(-360, 360);
    sb_B->setRange(-360, 360);
    hb1->addWidget(text_H);
    hb1->addWidget(sb_H);
    hb2->addWidget(text_P);
    hb2->addWidget(sb_P);
    hb3->addWidget(text_B);
    hb3->addWidget(sb_B);
    hb4->addWidget(cancel);
    hb4->addWidget(ok);
    vb->addLayout(hb1);
    vb->addLayout(hb2);
    vb->addLayout(hb3);
    vb->addLayout(hb4);

    dlg->setWindowTitle(tr("Rotate"));
    dlg->setLayout(vb);

    if(dlg->exec() == QDialog::Accepted) {
        
        double hr = sb_H->value();
        double pr = sb_P->value();
        double br = sb_B->value();
        Vec angles = Vec(hr,pr,br);
        Tree* to = sk.searchbyid(sk, selected_vertex);
   
    
        int des = -1;
        getallpoints(*to);
        std::vector<Vec> old = ve;
        ve.clear();
     
         // itt vátoztatjuk meg a kordinátát
     
        sk.change_all_rotason(*to, to->point, angles);
   
        getallpoints(*to);
        std::vector<Vec> newp = ve;
        ve.clear();
        for (int i = 0; i < b.size(); i++)
        {
            for (int j = 0; j < old.size(); j++)
            {
                if (b[i].start == old[j])
                {
                    b[i].start = newp[j];
                    des = i;
                }
                if (b[i].End == old[j])
                {
                    b[i].End = newp[j];
                   
                }
            }
            
            if (des != -1)
            {
                //Vec dif = newp[des] - old[des];
                qglviewer::Quaternion qx = qglviewer::Quaternion(Vec(1, 0, 0), angles.x / 180.0 * M_PI);
                qglviewer::Quaternion qy = qglviewer::Quaternion(Vec(0, 1, 0), angles.y / 180.0 * M_PI);
                qglviewer::Quaternion qz = qglviewer::Quaternion(Vec(0, 0, 1), angles.z / 180.0 * M_PI);
                /// <summary>
                /// EULER SZÖGEK MEG CSINÁLÁSA
                /// </summary>
                if (isweight)
                {
                    for (auto v : mesh.vertices())
                    {
                        if (mesh.data(v).weigh[des] != 0) 
                        {
                            Vec p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                            Vec result = to->point + qx.rotate(p- to->point);
                            OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(result.x, result.y, result.z);
                            mesh.point(v) = diffrents * mesh.data(v).weigh[des];

                      
                            p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                            Vec result2 = to->point + qy.rotate(p - to->point);
                            OpenMesh::Vec3d diffrents2 = OpenMesh::Vec3d(result2.x, result2.y, result2.z);      
                            mesh.point(v) = diffrents2 * mesh.data(v).weigh[des];

                        
                            p = Vec(mesh.point(v)[0], mesh.point(v)[1], mesh.point(v)[2]);
                            Vec result3 = to->point + qz.rotate(p - to->point);
                            OpenMesh::Vec3d diffrents3 = OpenMesh::Vec3d(result3.x, result3.y, result3.z);
                            mesh.point(v) = diffrents3 * mesh.data(v).weigh[des];

                        }

                    }
                    des = -1;
                }
            }
            

        }
        newp.clear();
        old.clear();
        update();
    }
     
}


void MyViewer::drawControlNet() const {
  glDisable(GL_LIGHTING);
  glLineWidth(3.0);
  glColor3d(0.3, 0.3, 1.0);
  size_t m = degree[1] + 1;
  for (size_t k = 0; k < 2; ++k)
    for (size_t i = 0; i <= degree[k]; ++i) {
      glBegin(GL_LINE_STRIP);
      for (size_t j = 0; j <= degree[1-k]; ++j) {
        size_t const index = k ? j * m + i : i * m + j;
        const auto &p = control_points[index];
        glVertex3dv(p);
      }
      glEnd();
    }
  glLineWidth(1.0);
  glPointSize(8.0);
  glColor3d(1.0, 0.0, 1.0);
  glBegin(GL_POINTS);
  for (const auto &p : control_points)
    glVertex3dv(p);
  glEnd();
  glPointSize(1.0);
  glEnable(GL_LIGHTING);
}

void MyViewer::drawAxes() const {
  const Vec &p = axes.position;
  glColor3d(1.0, 0.0, 0.0);
  drawArrow(p, p + Vec(axes.size, 0.0, 0.0), axes.size / 50.0);
  glColor3d(0.0, 1.0, 0.0);
  drawArrow(p, p + Vec(0.0, axes.size, 0.0), axes.size / 50.0);
  glColor3d(0.0, 0.0, 1.0);
  drawArrow(p, p + Vec(0.0, 0.0, axes.size), axes.size / 50.0);
  glEnd();
}

void MyViewer::drawWithNames() {
  if (axes.shown)
    return drawAxesWithNames();

  switch (model_type) {
  case ModelType::NONE: break;
  case ModelType::MESH:
    if (!show_wireframe)
      return;
    for (auto v : mesh.vertices()) {
      glPushName(v.idx());
      glRasterPos3dv(mesh.point(v).data());
      glPopName();
    }
    break;
  case ModelType::BEZIER_SURFACE:
    if (!show_control_points)
      return;
    for (size_t i = 0, ie = control_points.size(); i < ie; ++i) {
      Vec const &p = control_points[i];
      glPushName(i);
      glRasterPos3fv(p);
      glPopName();
    }
    break;
  case  ModelType::SKELTON:
      /*
      for (int i = 0; i < points.size(); i++)
      {
          Vec const& p = points[i];
          glPushName(i);
          glRasterPos3fv(p);
          glPopName();
      }
      */
      sk.drawarrow(sk);


      break;
  }
}

void MyViewer::drawAxesWithNames() const {
  const Vec &p = axes.position;
  glPushName(0);
  drawArrow(p, p + Vec(axes.size, 0.0, 0.0), axes.size / 50.0);
  glPopName();
  glPushName(1);
  drawArrow(p, p + Vec(0.0, axes.size, 0.0), axes.size / 50.0);
  glPopName();
  glPushName(2);
  drawArrow(p, p + Vec(0.0, 0.0, axes.size), axes.size / 50.0);
  glPopName();
}

void MyViewer::postSelection(const QPoint &p) {
  int sel = selectedName();
  if (sel == -1) {
    axes.shown = false;
    return;
  }

  if (axes.shown) {
    axes.selected_axis = sel;
    bool found;
    axes.grabbed_pos = camera()->pointUnderPixel(p, found);
    axes.original_pos = axes.position;
    if (!found)
      axes.shown = false;
    return;
  }

  selected_vertex = sel;
  if (model_type == ModelType::MESH)
    axes.position = Vec(mesh.point(MyMesh::VertexHandle(sel)).data());
  if (model_type == ModelType::BEZIER_SURFACE)
    axes.position = control_points[sel];
  if (model_type == ModelType::SKELTON)
  {
      Tree* t = sk.searchbyid(sk, sel);
      axes.position = t->point;
      //axes.position = points[sel];
      //sk.searchbyid(sk, 1);
  }
  double depth = camera()->projectedCoordinatesOf(axes.position)[2];
  Vec q1 = camera()->unprojectedCoordinatesOf(Vec(0.0, 0.0, depth));
  Vec q2 = camera()->unprojectedCoordinatesOf(Vec(width(), height(), depth));
  axes.size = (q1 - q2).norm() / 10.0;
  axes.shown = true;
  axes.selected_axis = -1;
}

void MyViewer::keyPressEvent(QKeyEvent *e) {

    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;
    QLabel* text;
  if (e->modifiers() == Qt::NoModifier)

    switch (e->key()) {
    case Qt::Key_R:
      if (model_type == ModelType::MESH)
        openMesh(last_filename, false);
      else if (model_type == ModelType::BEZIER_SURFACE)
        openBezier(last_filename, false);
      update();
      break;
    case Qt::Key_O:
      if (camera()->type() == qglviewer::Camera::PERSPECTIVE)
        camera()->setType(qglviewer::Camera::ORTHOGRAPHIC);
      else
        camera()->setType(qglviewer::Camera::PERSPECTIVE);
      update();
      break;
    case Qt::Key_P:
      visualization = Visualization::PLAIN;
      update();
      break;
    case Qt::Key_M:
      visualization = Visualization::MEAN;
      update();
      break;
    case Qt::Key_L:
      visualization = Visualization::SLICING;
      update();
      break;
    case Qt::Key_I:
      visualization = Visualization::ISOPHOTES;
      current_isophote_texture = isophote_texture;
      update();
      break;
    case Qt::Key_E:
      visualization = Visualization::ISOPHOTES;
      current_isophote_texture = environment_texture;
      update();
      break;
    case Qt::Key_1:
        if (points.size() != 0 && mesh.n_vertices() != 0)
        {
            model_type = ModelType::SKELTON;
            visualization = Visualization::WEIGH2;
            wi++;
        }
        update();
        break;

    case Qt::Key_4:
        if (axes.shown) {
            Rotate();
        }

        update();
        break;
    case Qt::Key_B:
      show_skelton = !show_skelton;
      update();
      break;

    case Qt::Key_T:
        if (points.size() != 0 && mesh.n_vertices() != 0)
        {
            visualization = Visualization::WEIGH;
            model_type = ModelType::SKELTON;
            mehet = true;
            isweight = true;
            weigh();
        }
        update();
        break;
    case Qt::Key_2:

        if (points.size() != 0 && mesh.n_vertices() != 0)
        {
            
            
            if (isweight == true && mehet == true)
            {
                visualization = Visualization::WEIGH;
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
            text = new QLabel(tr("Error: No mesh or skellton" ));
        }
        hb1->addWidget(text);
        vb->addLayout(hb1);
        dlg->setWindowTitle(tr("Message"));
        dlg->setLayout(vb);
        if (dlg->exec() == QDialog::Accepted) {
            update();
        }
        mehet = false;
        break;

    case Qt::Key_3:

        if (mesh.n_vertices() != 0)
        {
            for (auto v : mesh.vertices()) {
                mesh.data(v).weigh.clear();
            }
            model_type = ModelType::MESH;
            visualization = Visualization::PLAIN;
        }
        update();
        break;
    case Qt::Key_C:
      show_control_points = !show_control_points;
      update();
      break;
    case Qt::Key_S:
      show_solid = !show_solid;
      update();
      break;
    case Qt::Key_W:
      show_wireframe = !show_wireframe;
      update();
      break;
    case Qt::Key_F:
      fairMesh();
      update();
      break;
    default:
      QGLViewer::keyPressEvent(e);
    }
  else if (e->modifiers() == Qt::KeypadModifier)
    switch (e->key()) {
    case Qt::Key_Plus:
      slicing_scaling *= 2;
      update();
      break;
    case Qt::Key_Minus:
      slicing_scaling /= 2;
      update();
      break;
    case Qt::Key_Asterisk:
      slicing_dir = Vector(static_cast<double *>(camera()->viewDirection()));
      update();
      break;
    } else
    QGLViewer::keyPressEvent(e);
}

Vec MyViewer::intersectLines(const Vec &ap, const Vec &ad, const Vec &bp, const Vec &bd) {
  // always returns a point on the (ap, ad) line
  double a = ad * ad, b = ad * bd, c = bd * bd;
  double d = ad * (ap - bp), e = bd * (ap - bp);
  if (a * c - b * b < 1.0e-7)
    return ap;
  double s = (b * e - c * d) / (a * c - b * b);
  return ap + s * ad;
}

void MyViewer::bernsteinAll(size_t n, double u, std::vector<double> &coeff) {
  coeff.clear(); coeff.reserve(n + 1);
  coeff.push_back(1.0);
  double u1 = 1.0 - u;
  for (size_t j = 1; j <= n; ++j) {
    double saved = 0.0;
    for (size_t k = 0; k < j; ++k) {
      double tmp = coeff[k];
      coeff[k] = saved + tmp * u1;
      saved = tmp * u;
    }
    coeff.push_back(saved);
  }
}

void MyViewer::generateMesh(size_t resolution) {
  mesh.clear();
  std::vector<MyMesh::VertexHandle> handles, tri;
  size_t n = degree[0], m = degree[1];

  std::vector<double> coeff_u, coeff_v;
  for (size_t i = 0; i < resolution; ++i) {
    double u = (double)i / (double)(resolution - 1);
    bernsteinAll(n, u, coeff_u);
    for (size_t j = 0; j < resolution; ++j) {
      double v = (double)j / (double)(resolution - 1);
      bernsteinAll(m, v, coeff_v);
      Vec p(0.0, 0.0, 0.0);
      for (size_t k = 0, index = 0; k <= n; ++k)
        for (size_t l = 0; l <= m; ++l, ++index)
          p += control_points[index] * coeff_u[k] * coeff_v[l];
      handles.push_back(mesh.add_vertex(Vector(static_cast<double *>(p))));
    }
  }
  for (size_t i = 0; i < resolution - 1; ++i)
    for (size_t j = 0; j < resolution - 1; ++j) {
      tri.clear();
      tri.push_back(handles[i * resolution + j]);
      tri.push_back(handles[i * resolution + j + 1]);
      tri.push_back(handles[(i + 1) * resolution + j]);
      mesh.add_face(tri);
      tri.clear();
      tri.push_back(handles[(i + 1) * resolution + j]);
      tri.push_back(handles[i * resolution + j + 1]);
      tri.push_back(handles[(i + 1) * resolution + j + 1]);
      mesh.add_face(tri);
    }
}

void MyViewer::mouseMoveEvent(QMouseEvent *e) {
  if (!axes.shown ||
      (axes.selected_axis < 0 && !(e->modifiers() & Qt::ControlModifier)) ||
      !(e->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier)) ||
      !(e->buttons() & Qt::LeftButton))
    return QGLViewer::mouseMoveEvent(e);
  Vec p;
  float d;
  Vec axis(axes.selected_axis == 0, axes.selected_axis == 1, axes.selected_axis == 2);
  Vec old_pos = axes.position;
  if (e->modifiers() & Qt::ControlModifier) {
    // move in screen plane
    double depth = camera()->projectedCoordinatesOf(axes.position)[2];
    axes.position = camera()->unprojectedCoordinatesOf(Vec(e->pos().x(), e->pos().y(), depth));
  } else {
    Vec from, dir;
    camera()->convertClickToLine(e->pos(), from, dir);
    p = intersectLines(axes.grabbed_pos, axis, from, dir);
    d = (p - axes.grabbed_pos) * axis;
    axes.position[axes.selected_axis] = axes.original_pos[axes.selected_axis] + d;
  }

  if (model_type == ModelType::MESH)
    mesh.set_point(MyMesh::VertexHandle(selected_vertex),
                   Vector(static_cast<double *>(axes.position)));
  if (model_type == ModelType::BEZIER_SURFACE)
    control_points[selected_vertex] = axes.position;

  if (model_type == ModelType::SKELTON)
  {
     // Vec t = points[selected_vertex];

      /*
      * 
      * megkersük a kiválasztot ágakat
      */
      Tree* to = sk.searchbyid(sk, selected_vertex);
      int des = -1;
      getallpoints(*to);
      std::vector<Vec> old = ve;
      ve.clear();
     // points[selected_vertex] = axes.position;
      /*
      * itt vátoztatjuk meg a kordinátát
      */
      sk.change_all_position(*to, axes.position - old_pos);
      Vec dif = axes.position - old_pos;
      getallpoints(*to);
      std::vector<Vec> newp = ve;
      ve.clear();
      for (int i = 0; i < b.size(); i++)
      {
          for (int j = 0; j < old.size(); j++)
          {
              if (b[i].start == old[j])
              {
                  b[i].start = newp[j];
                  des = i;
              }
              if (b[i].End == old[j])
              {
                  b[i].End = newp[j];
                  des = i;
              }
          }
          if (des != -1)
          {

              OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(dif.x, dif.y, dif.z);
              if (isweight)
              {
                  for (auto v : mesh.vertices())
                  {
                      mesh.point(v) += diffrents * mesh.data(v).weigh[des];

                  }
                  des = -1;
              }
          }

      }
      newp.clear();
      old.clear();
      //Megnézük az összes csonton keresztül
      /*
      for (int i = 0; i < b.size(); i++)
      {
          if (t == b[i].start) {
              b[i].start = axes.position;
              des = i;
          }
          if (t == b[i].End) {
              b[i].End = axes.position;
              des = i;
          }
          if (des != -1)
          {
              Vec d = t - points[selected_vertex];
              OpenMesh::Vec3d diffrents = OpenMesh::Vec3d(d.x, d.y, d.z);
              if (isweight)
              {
                  for (auto v : mesh.vertices())
                  {
                      mesh.point(v) -= diffrents * mesh.data(v).weigh[des];

                  }
              }
          }
      }
      */
  }
  updateMesh();
  update();
}


void MyViewer::getallpoints(Tree t)
{
    ve.push_back(t.point);
    for (int i = 0; i < t.child.size(); i++)
    {
        getallpoints(t.child[i]);
    }
}

QString MyViewer::helpString() const {
  QString text("<h2>Sample Framework</h2>"
               "<p>This is a minimal framework for 3D mesh manipulation, which can be "
               "extended and used as a base for various projects, for example "
               "prototypes for fairing algorithms, or even displaying/modifying "
               "parametric surfaces, etc.</p>"
               "<p>The following hotkeys are available:</p>"
               "<ul>"
               "<li>&nbsp;R: Reload model</li>"
               "<li>&nbsp;O: Toggle orthographic projection</li>"
               "<li>&nbsp;P: Set plain map (no coloring)</li>"
               "<li>&nbsp;M: Set mean curvature map</li>"
               "<li>&nbsp;L: Set slicing map<ul>"
               "<li>&nbsp;+: Increase slicing density</li>"
               "<li>&nbsp;-: Decrease slicing density</li>"
               "<li>&nbsp;*: Set slicing direction to view</li></ul></li>"
               "<li>&nbsp;I: Set isophote line map</li>"
               "<li>&nbsp;E: Set environment texture</li>"
               "<li>&nbsp;C: Toggle control polygon visualization</li>"
               "<li>&nbsp;S: Toggle solid (filled polygon) visualization</li>"
               "<li>&nbsp;W: Toggle wireframe visualization</li>"
               "<li>&nbsp;F: Fair mesh</li>"
               "</ul>"
               "<p>There is also a simple selection and movement interface, enabled "
               "only when the wireframe/controlnet is displayed: a mesh vertex can be selected "
               "by shift-clicking, and it can be moved by shift-dragging one of the "
               "displayed axes. Pressing ctrl enables movement in the screen plane.</p>"
               "<p>Note that libQGLViewer is furnished with a lot of useful features, "
               "such as storing/loading view positions, or saving screenshots. "
               "OpenMesh also has a nice collection of tools for mesh manipulation: "
               "decimation, subdivision, smoothing, etc. These can provide "
               "good comparisons to the methods you implement.</p>"
               "<p>This software can be used as a sample GUI base for handling "
               "parametric or procedural surfaces, as well. The power of "
               "Qt and libQGLViewer makes it easy to set up a prototype application. "
               "Feel free to modify and explore!</p>"
               "<p align=\"right\">Peter Salvi</p>");
  return text;
}
