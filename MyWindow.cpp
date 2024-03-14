#include <memory>

#include <QtWidgets>

#include "MyWindow.h"

MyWindow::MyWindow(QApplication *parent) :
  QMainWindow(), parent(parent), last_directory(".")
{
  setWindowTitle(tr("Sample 3D Framework"));
  setStatusBar(new QStatusBar);
  progress = new QProgressBar;
  progress->setMinimum(0); progress->setMaximum(100);
  progress->hide();
  statusBar()->addPermanentWidget(progress);

  viewer = new MyViewer(this);
  connect(viewer, SIGNAL(startComputation(QString)), this, SLOT(startComputation(QString)));
  connect(viewer, SIGNAL(midComputation(int)), this, SLOT(midComputation(int)));
  connect(viewer, SIGNAL(endComputation()), this, SLOT(endComputation()));
  connect(viewer, SIGNAL(displayMessage(const QString&)), this, SLOT(displayMessage(const QString&)));
  setCentralWidget(viewer);

  /////////////////////////
  // Setup actions/menus //
  /////////////////////////

  auto openAction = new QAction(tr("&Open"), this);
  openAction->setShortcut(tr("Ctrl+O"));
  openAction->setStatusTip(tr("Load a model from a file"));
  connect(openAction, SIGNAL(triggered()), this, SLOT(open()));

  auto saveAction = new QAction(tr("&Save as.."), this);
  saveAction->setStatusTip(tr("Save a Bézier surface to a file"));
  connect(saveAction, SIGNAL(triggered()), this, SLOT(save()));

  auto quitAction = new QAction(tr("&Quit"), this);
  quitAction->setShortcut(tr("Ctrl+Q"));
  quitAction->setStatusTip(tr("Quit the program"));
  connect(quitAction, SIGNAL(triggered()), this, SLOT(close()));

  auto cutoffAction = new QAction(tr("Set &cutoff ratio"), this);
  cutoffAction->setStatusTip(tr("Set mean map cutoff ratio"));
  connect(cutoffAction, SIGNAL(triggered()), this, SLOT(setCutoff()));

  auto rangeAction = new QAction(tr("Set &range"), this);
  rangeAction->setStatusTip(tr("Set mean map range"));
  connect(rangeAction, SIGNAL(triggered()), this, SLOT(setRange()));

  auto slicingAction = new QAction(tr("Set &slicing parameters"), this);
  rangeAction->setStatusTip(tr("Set contouring direction and scaling"));
  connect(slicingAction, SIGNAL(triggered()), this, SLOT(setSlicing()));

  auto fileMenu = menuBar()->addMenu(tr("&File"));
  fileMenu->addAction(openAction);
  fileMenu->addAction(saveAction);
  fileMenu->addAction(quitAction);

  auto visMenu = menuBar()->addMenu(tr("&Visualization"));
  visMenu->addAction(cutoffAction);
  visMenu->addAction(rangeAction);
  visMenu->addAction(slicingAction);

  auto hSplitter = new QSplitter(Qt::Horizontal, this);
  auto controlWidget = new QWidget(this);
  auto sa = new QScrollArea(this);


  controlWidget->setAutoFillBackground(true);
  QGridLayout* controlLayout = new QGridLayout();
  
  auto buttonOpen = new QPushButton("Open Mesh");
  controlLayout->addWidget(buttonOpen, 0, 0, 1, 1);
  connect(buttonOpen, SIGNAL(clicked()), this, SLOT(open()));


  auto buttonOpenB = new QPushButton("Open Bone");
  controlLayout->addWidget(buttonOpenB, 0, 1, 1, 1);
  connect(buttonOpenB, SIGNAL(clicked()), this, SLOT(open_bone()));

  auto buttonMesh = new QPushButton("Mesh");
  controlLayout->addWidget(buttonMesh, 1, 0, 1, 1);
  connect(buttonMesh, SIGNAL(clicked()), this, SLOT(set2mesh()));

  auto buttonBone = new QPushButton("Bone");
  controlLayout->addWidget(buttonBone, 1, 1, 1, 1);
  connect(buttonBone, SIGNAL(clicked()), this, SLOT(set2bone()));




  controlLayout->addWidget(layer, 3, 1, 1, 1);

  auto buttonSkining = new QPushButton("Skining");
  controlLayout->addWidget(buttonSkining, 2, 0, 1, 1);
  connect(buttonSkining, SIGNAL(clicked()), this, SLOT(skining()));


  auto buttonHeat= new QPushButton("Bone Heat");
  controlLayout->addWidget(buttonHeat, 2, 1, 1, 1);
  connect(buttonHeat, SIGNAL(clicked()), this, SLOT(boneheat()));


  auto buttonPoint = new QPushButton("Vertex");
  controlLayout->addWidget(buttonPoint, 3 ,0, 1, 1);
  connect(buttonPoint, SIGNAL(clicked()), this, SLOT(getpoint()));


  controlLayout->addWidget(wlayer, 5, 1, 1, 1);
  auto buttonW = new QPushButton("Weigh");
  controlLayout->addWidget(buttonW, 5, 0, 1, 1);
  connect(buttonW, SIGNAL(clicked()), this, SLOT(weightindex()));






  auto buttonwier = new QPushButton("Wierframe");
  controlLayout->addWidget(buttonwier, 4, 0, 1, 1);
  connect(buttonwier, SIGNAL(clicked()), this, SLOT(wierframeon()));


  auto buttonshow = new QPushButton("Show");
  controlLayout->addWidget(buttonshow, 4, 1, 1, 1);
  connect(buttonshow, SIGNAL(clicked()), this, SLOT(showm()));


  auto buttonrot = new QPushButton("Rotation");
  controlLayout->addWidget(buttonrot, 6, 1, 1, 1);
  connect(buttonrot, SIGNAL(clicked()), this, SLOT(rotation()));


  auto buttrest = new QPushButton("Reset");
  controlLayout->addWidget(buttrest, 6, 0, 1, 1);
  connect(buttrest, SIGNAL(clicked()), this, SLOT(resetall()));


  auto buttonstart = new QPushButton("Start");
  controlLayout->addWidget(buttonstart, 7, 0, 1, 1);
  connect(buttonstart, SIGNAL(clicked()), this, SLOT(start()));


  auto buttonend = new QPushButton("End");
  controlLayout->addWidget(buttonend, 7, 1, 1, 1);
  connect(buttonend, SIGNAL(clicked()), this, SLOT(end()));

  auto buttonskin = new QPushButton("Skin");
  controlLayout->addWidget(buttonskin, 8, 0, 1, 1);
  connect(buttonskin, SIGNAL(clicked()), this, SLOT(skin()));
  controlLayout->addWidget(flayer, 8, 1, 1, 1);

  auto buttonsmoth = new QPushButton("Smouth");
  controlLayout->addWidget(buttonsmoth, 9, 1, 1, 1);
  connect(buttonsmoth, SIGNAL(clicked()), this, SLOT(LaplanceSmooth()));


  auto buttonedelta = new QPushButton("DeltaMush");
  controlLayout->addWidget(buttonedelta, 9, 0, 1, 1);
  connect(buttonedelta, SIGNAL(clicked()), this, SLOT(Delta()));


  controlWidget->setLayout(controlLayout);


  sa->setWidget(controlWidget);
  sa->setFixedWidth(sa->sizeHint().width());

  hSplitter->insertWidget(0, sa);
  hSplitter->insertWidget(1, viewer);
  setCentralWidget(hSplitter);

}

MyWindow::~MyWindow() {
}


void MyWindow::open_bone() {
    auto filename = QFileDialog::getOpenFileName(this, tr("Open File"), last_directory,
        tr("Readable files ( *.bone);;"
            "Bone (*.bone);;"
            "All files (*.*)"));
    if (filename.isEmpty())
        return;
    last_directory = QFileInfo(filename).absolutePath();

    bool ok;
    if (filename.endsWith(".bone"))
        ok = viewer->openSkelton(filename.toUtf8().data());

    if (!ok)
        QMessageBox::warning(this, tr("Cannot open file"),
            tr("Could not open file: ") + filename + ".");

    int n = viewer->getbone_size();
    layer->setText(tr("Bones: ") + std::to_string(n).c_str());
}


void MyWindow::open() {
  auto filename =
    QFileDialog::getOpenFileName(this, tr("Open File"), last_directory,
                                 tr("Readable files (*.obj *.ply *.stl *.bzr );;"
                                    "Mesh (*.obj *.ply *.stl);;"
                                    "Bézier surface (*.bzr);;"
                                    "All files (*.*)"));
  if(filename.isEmpty())
    return;
  last_directory = QFileInfo(filename).absolutePath();

  bool ok;
  if (filename.endsWith(".bzr"))
      ok = viewer->openBezier(filename.toUtf8().data());
  else
    ok = viewer->openMesh(filename.toUtf8().data());

  if (!ok)
    QMessageBox::warning(this, tr("Cannot open file"),
                         tr("Could not open file: ") + filename + ".");
}

void MyWindow::save() {
  auto filename =
    QFileDialog::getSaveFileName(this, tr("Save File"), last_directory,
                                 tr("Bézier surface (*.bzr);;"
                                     "Bone (*.bone);;"));
  if(filename.isEmpty())
    return;
  last_directory = QFileInfo(filename).absolutePath();

  if (!viewer->saveBone(filename.toUtf8().data()))
      QMessageBox::warning(this, tr("Cannot save file"),
          tr("Could not save file: ") + filename + ".");

  /*if (!viewer->saveBezier(filename.toUtf8().data()))
    QMessageBox::warning(this, tr("Cannot save file"),
                         tr("Could not save file: ") + filename + ".");*/
}

void MyWindow::setCutoff() {
  // Memory management options for the dialog:
  // - on the stack (deleted at the end of the function)
  // - on the heap with manual delete or std::unique_ptr 
  // There is also a Qt-controlled automatic deletion by calling
  //     dlg->setAttribute(Qt::WA_DeleteOnClose);
  // ... but this could delete sub-widgets too early.

  auto dlg = std::make_unique<QDialog>(this);
  auto *hb1    = new QHBoxLayout,
       *hb2    = new QHBoxLayout;
  auto *vb     = new QVBoxLayout;
  auto *text   = new QLabel(tr("Cutoff ratio:"));
  auto *sb     = new QDoubleSpinBox;
  auto *cancel = new QPushButton(tr("Cancel"));
  auto *ok     = new QPushButton(tr("Ok"));

  sb->setDecimals(3);
  sb->setRange(0.001, 0.5);
  sb->setSingleStep(0.01);
  sb->setValue(viewer->getCutoffRatio());
  connect(cancel, SIGNAL(pressed()), dlg.get(), SLOT(reject()));
  connect(ok,     SIGNAL(pressed()), dlg.get(), SLOT(accept()));
  ok->setDefault(true);

  hb1->addWidget(text);
  hb1->addWidget(sb);
  hb2->addWidget(cancel);
  hb2->addWidget(ok);
  vb->addLayout(hb1);
  vb->addLayout(hb2);

  dlg->setWindowTitle(tr("Set ratio"));
  dlg->setLayout(vb);

  if(dlg->exec() == QDialog::Accepted) {
    viewer->setCutoffRatio(sb->value());
    viewer->update();
  }
}

void MyWindow::setRange() {
  QDialog dlg(this);
  auto *grid   = new QGridLayout;
  auto *text1  = new QLabel(tr("Min:")),
       *text2  = new QLabel(tr("Max:"));
  auto *sb1    = new QDoubleSpinBox,
       *sb2    = new QDoubleSpinBox;
  auto *cancel = new QPushButton(tr("Cancel"));
  auto *ok     = new QPushButton(tr("Ok"));

  // The range of the spinbox controls the number of displayable digits,
  // so setting it to a large value results in a very wide window.
  double max = 1000.0; // std::numeric_limits<double>::max();
  sb1->setDecimals(5);                 sb2->setDecimals(5);
  sb1->setRange(-max, 0.0);            sb2->setRange(0.0, max);
  sb1->setSingleStep(0.01);            sb2->setSingleStep(0.01);
  sb1->setValue(viewer->getMeanMin()); sb2->setValue(viewer->getMeanMax());
  connect(cancel, SIGNAL(pressed()), &dlg, SLOT(reject()));
  connect(ok,     SIGNAL(pressed()), &dlg, SLOT(accept()));
  ok->setDefault(true);

  grid->addWidget( text1, 1, 1, Qt::AlignRight);
  grid->addWidget(   sb1, 1, 2);
  grid->addWidget( text2, 2, 1, Qt::AlignRight);
  grid->addWidget(   sb2, 2, 2);
  grid->addWidget(cancel, 3, 1);
  grid->addWidget(    ok, 3, 2);

  dlg.setWindowTitle(tr("Set range"));
  dlg.setLayout(grid);

  if(dlg.exec() == QDialog::Accepted) {
    viewer->setMeanMin(sb1->value());
    viewer->setMeanMax(sb2->value());
    viewer->update();
  }
}

void MyWindow::setSlicing() {
  auto dlg = std::make_unique<QDialog>(this);
  auto *hb1    = new QHBoxLayout,
       *hb2    = new QHBoxLayout,
       *hb3    = new QHBoxLayout;
  auto *vb     = new QVBoxLayout;
  auto *text_v = new QLabel(tr("Slicing direction:"));
  QDoubleSpinBox *sb_v[3];
  auto *text_s = new QLabel(tr("Slicing scaling:"));
  auto *sb_s   = new QDoubleSpinBox;
  auto *cancel = new QPushButton(tr("Cancel"));
  auto *ok     = new QPushButton(tr("Ok"));

  for (int i = 0; i < 3; ++i) {
    sb_v[i] = new QDoubleSpinBox;
    sb_v[i]->setDecimals(3);
    sb_v[i]->setRange(-1, 1);
    sb_v[i]->setSingleStep(0.01);
    sb_v[i]->setValue(viewer->getSlicingDir()[i]);
  }
  sb_s->setDecimals(6);
  sb_s->setRange(0, 10000);
  sb_s->setSingleStep(1);
  sb_s->setValue(viewer->getSlicingScaling());
  connect(cancel, SIGNAL(pressed()), dlg.get(), SLOT(reject()));
  connect(ok,     SIGNAL(pressed()), dlg.get(), SLOT(accept()));
  ok->setDefault(true);

  hb1->addWidget(text_v);
  hb1->addWidget(sb_v[0]); hb1->addWidget(sb_v[1]); hb1->addWidget(sb_v[2]);
  hb2->addWidget(text_s);
  hb2->addWidget(sb_s);
  hb3->addWidget(cancel);
  hb3->addWidget(ok);
  vb->addLayout(hb1);
  vb->addLayout(hb2);
  vb->addLayout(hb3);

  dlg->setWindowTitle(tr("Set slicing"));
  dlg->setLayout(vb);

  if(dlg->exec() == QDialog::Accepted) {
    viewer->setSlicingDir(sb_v[0]->value(), sb_v[1]->value(), sb_v[2]->value());
    viewer->setSlicingScaling(sb_s->value());
    viewer->update();
  }
}

void MyWindow::startComputation(QString message) {
  statusBar()->showMessage(message);
  progress->setValue(0);
  progress->show();
  parent->processEvents(QEventLoop::ExcludeUserInputEvents);
}

void MyWindow::midComputation(int percent) {
  progress->setValue(percent);
  parent->processEvents(QEventLoop::ExcludeUserInputEvents);
}

void MyWindow::endComputation() {
  progress->hide();
  statusBar()->clearMessage();
}

void MyWindow::displayMessage(const QString& message) {
    statusBar()->showMessage(message);
    parent->processEvents(QEventLoop::ExcludeUserInputEvents);
}
