// -*- mode: c++ -*-
#pragma once

#include <QtWidgets/QMainWindow>

#include "MyViewer.h"

class QApplication;
class QProgressBar;

class MyWindow : public QMainWindow {
  Q_OBJECT

public:
  explicit MyWindow(QApplication *parent);
  ~MyWindow();

private slots:

  void set2bone() { viewer->setBone(); viewer->update(); }
  void set2mesh() { viewer->setMesh(); viewer->update(); }
  void skining()  { viewer->skining(); viewer->update(); }
  void boneheat() { viewer->Boneheat(); }
  void getpoint() { viewer->selectedvert(); viewer->update(); }
  void wierframeon() { viewer->wierframe(); }
  void weightindex() { viewer->index_of_weight(); std::string s = "weight: " + std::to_string(0); wlayer->setText(s.c_str()); }
  void frame() { }
  void start() { viewer->Frame(); std::string s = "Frame: " + std::to_string(viewer->getFrameSecond()); flayer->setText(s.c_str());
  }
  void end() { viewer->stopAnimation(); }
  void rotation() { viewer->Rotate(); }
  void keyframe() { viewer->keyframe_add(); }
  void showm() { viewer->show(); }
  void resetall() { viewer->Reset(); }
  void skin() { viewer->skining(); viewer->update();}
  void open_bone();
  void open();
  void save();
  void setCutoff();
  void setRange();
  void setSlicing();
  void startComputation(QString message);
  void midComputation(int percent);
  void endComputation();
  void LaplanceSmooth() { viewer->Laplace(); }
  void Delta() { viewer->delta(); }
  void displayMessage(const QString& message);
  void Sample() { viewer->poission(); }
  void Implicit() { }
  void Slider(int value) { viewer->setSlider(value); Slayer->setText("Value: " + QString::number(value)); }

private:
  QApplication *parent;
  MyViewer *viewer;
  QProgressBar *progress;
  QString last_directory;
  QLabel* wlayer = new QLabel(tr("weight: "));
  QLabel* flayer = new QLabel(tr("Frame: "));
  QLabel* layer = new QLabel(tr("Bones: "));
  QLabel* Slayer = new QLabel(tr("Value: "));
  QSlider* slider = new QSlider(Qt::Horizontal, this);
};
