#include <algorithm>
#include <cmath>
#include <map>
#include "skeleton/Bone.h"
#include "skinning/BoneHeat.h"
#include "skinning/ImprovedDeltaMush.h"

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

MyViewer::MyViewer(QWidget* parent) :
    QGLViewer(parent), model_type(ModelType::NONE),
    mean_min(0.0), mean_max(0.0), cutoff_ratio(0.05),
    show_control_points(true), show_solid(true), show_wireframe(false), show_skelton(false),
    visualization(Visualization::PLAIN), slicing_dir(0, 0, 1), slicing_scaling(1),
    last_filename("")
{
    timer = new QTimer(this);

    of = std::ofstream("log.txt");
    std::cerr.rdbuf(of.rdbuf());
    omerr().rdbuf(of.rdbuf());
    setSelectRegionWidth(10);
    setSelectRegionHeight(10);
    axes.shown = false;
    isAnimating_ = false;
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
    mean_min = std::min(mean[k ? k - 1 : 0], 0.0);
    mean_max = std::max(mean[k ? n - k : n - 1], 0.0);
}

void MyViewer::localSystem(const MyViewer::Vector& normal,
    MyViewer::Vector& u, MyViewer::Vector& v) {
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
    }
    else if (std::abs(normal[2]) > next)
        nexti = 2;

    u.vectorize(0.0);
    u[nexti] = -normal[maxi];
    u[maxi] = normal[nexti];
    u /= u.norm();
    v = normal % u;
}

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




static Vec RGB2HSV(Vec rgb) {

    Vec hsv;
    double min, max, delta;
    double r, g, b;
    r = rgb[0];
    g = rgb[1];
    b = rgb[2];
    min = r < g ? r : g;
    min = min < b ? min : b;
    max = r > g ? r : g;
    max = max > b ? max : b;
    delta = max - min;
    hsv.z = max;
    if (delta < 0.00001)
    {
        hsv.x = 0;
        hsv.y = 0;
        return hsv;
    }
    if (max > 0.0)
    {
        hsv.y = (delta / max);
    }
    else
    {
        hsv.y = 0;
        hsv.x = NAN;
        return hsv;
    }
    if (r >= max)
    {
        hsv.x = (g - b) / delta;
    }
    else
        if (g >= max)
            hsv.x = 2.0 + (b - r) / delta;
        else
            hsv.x = 4.0 + (r - g) / delta;

    hsv.x *= 60.0;

    if (hsv.x < 0.0)
        hsv.x += 360.0;

    return hsv;
}


Vec MyViewer::meanMapColor(double d) const {
    double red = 0, green = 120, blue = 240; // Hue
    if (d < 0) {
        double alpha = mean_min ? std::min(d / mean_min, 1.0) : 1.0;
        return HSV2RGB({ green * (1 - alpha) + blue * alpha, 1, 1 });
    }
    double alpha = mean_max ? std::min(d / mean_max, 1.0) : 1.0;
    return HSV2RGB({ green * (1 - alpha) + red * alpha, 1, 1 });
}


void MyViewer::addObject(std::shared_ptr<Object3D> object) {
    objects.push_back(object);
}

void MyViewer::addObjects(const std::vector<std::shared_ptr<Object3D>>& newObjects) {
    objects.insert(objects.end(), newObjects.begin(), newObjects.end());
}

void MyViewer::stopTimer() {
    if (timer->isActive()) {
        timer->stop();  // Stop the timer
    }
}

void MyViewer::setupCamera() {
    // Set camera on the model
    double large = std::numeric_limits<double>::max();
    Vector box_min(large, large, large), box_max(-large, -large, -large);
    for(auto object : objects)
    {
        object->setCameraFocus(box_min, box_max);
    }
    camera()->setSceneBoundingBox(Vec(box_min.data()), Vec(box_max.data()));
    camera()->showEntireScene();

    slicing_scaling = 20 / (box_max - box_min).max();

    setSelectedName(-1);
    axes.shown = false;

    update();
}



void MyViewer::init() {
    glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);

    connect(timer, &QTimer::timeout, this, &MyViewer::callKinekcnUpdate);

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
    setAnimationPeriod(16);
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

    if (dlg->exec() == QDialog::Accepted) {
        double hr = sb_H->value();
        double pr = sb_P->value();
        double br = sb_B->value();
        Vec angles = Vec(hr, pr, br);
        angels = angles;
        rotation = angles;
        objects[selected_object]->rotate(selected_vertex, angels); 
        update();
    }
}

/// <summary>
/// TODO Refact to for the OO
/// </summary>
void MyViewer::selectedjoin()
{
    if (axes.shown)
    {
        auto dlg = std::make_unique<QDialog>(this);
        auto* hb1 = new QHBoxLayout,
            * hb2 = new QHBoxLayout,
            * hb3 = new QHBoxLayout;
        auto* vb = new QVBoxLayout;
        Joint* jo = nullptr;

        auto* text_H = new QLabel(tr("Matrix: "));
        auto* text_b = new QLabel(tr("Point: "));
        std::string s = jo->M.to_string();
        text_H->setText(s.c_str());
        std::string se = std::to_string(jo->point.x)+" " +std::to_string(jo->point.y) + " "+std::to_string(jo->point.z);
        text_b->setText(se.c_str());
        hb1->addWidget(text_H);
        hb2->addWidget(text_b);
        vb->addLayout(hb1);
        vb->addLayout(hb2);
        vb->addLayout(hb3);

        dlg->setWindowTitle(tr("Data of Join"));
        dlg->setLayout(vb);

        if (dlg->exec() == QDialog::Accepted) {


        }


    }
}



void MyViewer::callKinekcnUpdate() {
    // Call the kinect's update function
    kinect.update();

    // Optionally refresh the viewer
    update();
}


void MyViewer::show() {
    show_solid = !show_solid;
    update();
}

void MyViewer::index_of_weight()
{

}

/// <summary>
/// TODO Refact to for the OO
/// </summary>
void MyViewer::selectedvert()
{

    if (axes.shown && model_type == ModelType::MESH) {
        auto v = MyMesh::VertexHandle(selected_vertex);
        auto selcted_point = mesh.data(v);
        auto dlg = std::make_unique<QDialog>(this);
        auto* hb1 = new QHBoxLayout,
            * hb2 = new QHBoxLayout,
            * hb3 = new QHBoxLayout;
        auto* vb = new QVBoxLayout;


        for (int i = 0; i < selcted_point.weigh.size(); i++)
        {
            auto color = Vec() * 255;
            QColor rgb(color[0], color[1], color[2]);
            QString style = QString("QLabel { background-color : rgb(%1, %2, %3) }").arg(rgb.red()).arg(rgb.green()).arg(rgb.blue());

            auto* text_H = new QLabel(tr("X: "));
            auto* text_b = new QLabel(tr("X: "));
            text_H->setStyleSheet(style);
            std::string s = std::to_string(i) + " :" + std::to_string(selcted_point.distance[i]);
            text_H->setText(s.c_str());

            text_b->setStyleSheet(style);
            std::string se = std::to_string(i) + " :" + std::to_string(selcted_point.weigh[i]);
            text_b->setText(se.c_str());

            hb1->addWidget(text_H);

            hb2->addWidget(text_b);
        }

        vb->addLayout(hb1);
        vb->addLayout(hb2);
        vb->addLayout(hb3);

        dlg->setWindowTitle(tr("Data of vertex"));
        dlg->setLayout(vb);

        if (dlg->exec() == QDialog::Accepted) {


        }
    }
}

void MyViewer::skining()
{
    vis.type = Vis::VisualType::WEIGH;
    if (auto skeleton = std::dynamic_pointer_cast<Skelton>(objects[0]))
    {
        if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(objects[1]))
        {
            std::shared_ptr<Skinning> baseskinning = std::make_shared<Skinning>();
            skeleton->setSkinning(baseskinning);
            skeleton->skinning(mesh);
            update();
        }
    }
}

void MyViewer::createControlPoins(Joint* j)
{

}

void MyViewer::delta()
{
    if (auto skeleton = std::dynamic_pointer_cast<Skelton>(objects[0]))
    {
        vis.type = Vis::VisualType::WEIGH;
        std::shared_ptr<Skinning> delatmush = std::make_shared<DeltaMush>();
        if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(objects[1]))
        {
            skeleton->setSkinning(delatmush);
            skeleton->skinning(mesh);
            update();
        }
    }
}



/// <summary>
///  TODO: Refactor this for a clener code
/// </summary>
void MyViewer::createCP() {
    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;
    QLabel* text;
    bool IKok;
    if (auto skeleton = std::dynamic_pointer_cast<Skelton>(objects[selected_object]))
    {
        Joint* j = skeleton->getSelectedJoint(selected_vertex);
        IKok = skeleton->hasMultipleChildren(j);
        if (IKok)
        {
            Joint* leaf = j->getLeaf(j);
            std::shared_ptr<ControlPoint> cp = std::make_shared<ControlPoint>(leaf->point * 1.1, controlPointId, skeleton);
            cp->jointid = j->id;
            addObject(cp);
            setupCamera();
            controlPointId++;
        }
        else
        {
            text = new QLabel(tr("Error: No mesh or skellton"));
            hb1->addWidget(text);
            vb->addLayout(hb1);
            dlg->setWindowTitle(tr("Message"));
            dlg->setLayout(vb);
            if (dlg->exec() == QDialog::Accepted) {
            }
        }
    }
}



void MyViewer::improveDeltaMush()
{
    if (auto skeleton = std::dynamic_pointer_cast<Skelton>(objects[0]))
    {
        vis.type = Vis::VisualType::WEIGH;
        std::shared_ptr<Skinning> mymush = std::make_shared<ImprovedDeltaMush>();       
        if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(objects[1]))
        {
            skeleton->setSkinning(mymush);
            skeleton->skinning(mesh);
            update();
        }
    }
}



void MyViewer::setSlider(int value) {
    deltaMushFactor = (float)value / 100.0f;
    update();
}



void MyViewer::keyPressEvent(QKeyEvent* e) {

    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;
    QLabel* text;
    int sizek;  
    Vec ang = Vec(20, 0, 0);
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
           // visualization = Visualization::MEAN;
            createCP();

            update();
            break;
        case Qt::Key_L:
            visualization = Visualization::SLICING;

            update();
            break;
        case Qt::Key_V:
           
            update();
            break;

        case Qt::Key_I:
            visualization = Visualization::ISOPHOTES;
;
            current_isophote_texture = isophote_texture;
            update();
            break;
        case Qt::Key_E:
            visualization = Visualization::ISOPHOTES;
            current_isophote_texture = environment_texture;
            update();
            break;

        case Qt::Key_H:
            improveDeltaMush();
            update();
            break;

        case Qt::Key_G:
            createSculpt();       
            update();
            break;
        case Qt::Key_4:
            if (axes.shown) {
                selectedjoin();
            }

            update();
            break;
        case Qt::Key_B:
            show_skelton = !show_skelton;
            update();
            break;

        case Qt::Key_T:
            
            mc.showSampels = !mc.showSampels;
            update();
            break;
        case Qt::Key_2:

            startTimer();
            break;
        case Qt::Key_3:
            keyframe_add();
            update();
            break;
        case Qt::Key_C:
            show_control_points = !show_control_points;
            update();
            break;
        case Qt::Key_S:
            vis.show_solid = !vis.show_solid;
            update();
            break;
        case Qt::Key_W:
            vis.show_wireframe = !vis.show_wireframe;
            update();
            break;
        case Qt::Key_F:
            //fairMesh();
            
            kinect.CreateFirstConnected();
            update();
            break;

        case Qt::Key_Z:
            kinect.setSkeltonTracking();
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
            slicing_dir = Vector(static_cast<double*>(camera()->viewDirection()));
            update();
            break;
        }
    else
        QGLViewer::keyPressEvent(e);
}

void MyViewer::startTimer() {
    if (!timer->isActive()) {
        timer->start(10);  
    }
}



void MyViewer::createSculpt()
{

    bool isVaildIndex = selected_object <= objects.size();
    if (isVaildIndex)
    {
        if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(objects[selected_object]))
        {
            sculpt = std::make_unique<Sculpt>(mesh);
        }
    }
}


void MyViewer::endSelection(const QPoint& point) {
    glFlush();
    GLint nbHits = glRenderMode(GL_RENDER);
    if (nbHits <= 0)
        setSelectedName(-1);
    else {
        const GLuint* ptr = selectBuffer();
        GLuint zMin = std::numeric_limits<GLuint>::max();
        for (int i = 0; i < nbHits; ++i, ptr += 4) {
            GLuint names = ptr[0];
            if (ptr[1] < zMin) {
                zMin = ptr[1];
                if (names == 2) {
                    selected_object = ptr[3];
                    ptr++;
                }
                setSelectedName(ptr[3]);
            }
            else if (names == 2)
                ptr++;
        }
    }
}


Vec MyViewer::intersectLines(const Vec& ap, const Vec& ad, const Vec& bp, const Vec& bd) {
    // always returns a point on the (ap, ad) line
    double a = ad * ad, b = ad * bd, c = bd * bd;
    double d = ad * (ap - bp), e = bd * (ap - bp);
    if (a * c - b * b < 1.0e-7)
        return ap;
    double s = (b * e - c * d) / (a * c - b * b);
    return ap + s * ad;
}

void MyViewer::bernsteinAll(size_t n, double u, std::vector<double>& coeff) {
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

Vec MyViewer::calcCentriod(MyMesh& _mesh) {
    MyMesh::Point centroid(0.0, 0.0, 0.0);

    // Iterate over all vertices and sum their positions
    for (auto vh : _mesh.vertices()) {
        centroid += _mesh.point(vh);
    }

    // Divide by the number of vertices to get the centroid
    centroid /= _mesh.n_vertices();

    return Vec(centroid);
}

float  MyViewer::Epsil() {
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
    float epsilo = 0;
    if (dlg->exec() == QDialog::Accepted) {
        epsilo = sb_H->value();
        update();
    }

    return epsilo;
}

void MyViewer::Boneheat()
{
    auto dlg = std::make_unique<QDialog>(this);
    auto* hb1 = new QHBoxLayout;
    auto* vb = new QVBoxLayout;

    QLabel* text;


    Epsil();
    if (auto skeleton = std::dynamic_pointer_cast<Skelton>(objects[0]))
    {
        vis.type = Vis::VisualType::WEIGH;
        std::shared_ptr<Skinning> boneheat = std::make_shared<BoneHeat>();
        if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(objects[1]))
        {
            skeleton->setSkinning(boneheat);
            skeleton->skinning(mesh);
        }
        text = new QLabel(tr("Success"));
    }
    else
    {
        text = new QLabel(tr("Error: No weight in the mesh"));
    }
    

    hb1->addWidget(text);
    vb->addLayout(hb1);
    dlg->setWindowTitle(tr("Message"));
    dlg->setLayout(vb);
    if (dlg->exec() == QDialog::Accepted) {
        update();
    }
}

void MyViewer::drawCircle(float centerX, float centerY, float radius) {
  
    glColor3f(1.0, 0.0, 0.0);
    glBegin(GL_LINE_LOOP);
    const int numSegments = 50;

    for (int i = 0; i < numSegments; ++i) 
    {
        float theta = 2.0f * M_PI * float(i) / float(numSegments);
        float x = radius * cosf(theta);
        float y = radius * sinf(theta);
        glVertex3f(fa.x + x, fa.y + y, fa.z);
    }
    glEnd();
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
            handles.push_back(mesh.add_vertex(Vector(static_cast<double*>(p))));
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

void MyViewer::mouseMoveEvent(QMouseEvent* e) {
    

    if (!axes.shown ||
        (axes.selected_axis < 0 && !(e->modifiers() & Qt::ControlModifier)) ||
        !(e->modifiers() & (Qt::ShiftModifier | Qt::ControlModifier)) ||
        !(e->buttons() & Qt::LeftButton))
    {
        
         
 
        return QGLViewer::mouseMoveEvent(e);
    }
    Vec p;
    float d;
    Vec axis(axes.selected_axis == 0, axes.selected_axis == 1, axes.selected_axis == 2);
    Vec old_pos = axes.position;
    if (e->modifiers() & Qt::ControlModifier) {
        // move in screen plane
        double depth = camera()->projectedCoordinatesOf(axes.position)[2];
        axes.position = camera()->unprojectedCoordinatesOf(Vec(e->pos().x(), e->pos().y(), depth));
    }
    else {
        Vec from, dir;
        camera()->convertClickToLine(e->pos(), from, dir);
        p = intersectLines(axes.grabbed_pos, axis, from, dir);
        d = (p - axes.grabbed_pos) * axis;
        axes.position[axes.selected_axis] = axes.original_pos[axes.selected_axis] + d;
    }
    double de = camera()->projectedCoordinatesOf(axes.position)[2];
    fa = camera()->unprojectedCoordinatesOf(qglviewer::Vec(e->x(), e->y(), de));

    moveObject(objects[selected_object], selected_vertex, Vector(static_cast<double*>(axes.position)));
    update();
}



void MyViewer::moveObject(std::shared_ptr<Object3D> elemt, int selected, Vector position)
{
    if (auto mesh = std::dynamic_pointer_cast<BaseMesh>(elemt))
    {
        if (sculpt != nullptr)
        {
            sculpt->grab(selected_vertex, axes.position);
        }
    }
    else
    {
        elemt->movement(selected, position);
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
