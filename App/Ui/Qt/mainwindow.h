#pragma once

#include <App/Applied/Drawer/drawer.h>
#include <App/Applied/Facade/facade.h>

#include <QGraphicsScene>
#include <QKeyEvent>
#include <QMainWindow>
#include <QMouseEvent>
#include <QThread>
#include <QTimer>
#include <QWheelEvent>
#include <cstddef>
#include <memory>

#include "global_types.h"

QT_BEGIN_NAMESPACE
namespace Ui {
class MainWindow;
}
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget* parent = nullptr);

  ~MainWindow() override;

  void closeEvent(QCloseEvent* event) override {
    emit finishAppSignal();

    event->accept();
  }

 public slots:
  void start();

  // from MainWorker
  void getFpsSlot(int FPS, size_t np);
  void simRunnedSlot();
  void simStoppedSlot();

  // mouse and keyboard handle
  void mousePressSlot(QMouseEvent* event);
  void mouseReleaseSlot(QMouseEvent* event);
  void mouseMoveSlot(QMouseEvent* event);
  void wheelSlot(QWheelEvent* event);
  void keyPressSlot(QKeyEvent* event);
  void keyReleaseSlot(QKeyEvent* event);

  // buttons handle
  void on_loadPolygonalModelBtn_clicked();
  void on_runSimBtn_clicked();

  // set camera
  void on_topCamBtn_clicked();
  void on_bottomCamBtn_clicked();
  void on_frontCamBtn_clicked();
  void on_backCamBtn_clicked();
  void on_rightCamBtn_clicked();
  void on_leftCamBtn_clicked();

  // zone size
  void on_lengthSpinBox_valueChanged(double arg1);
  void on_widthSpinBox_valueChanged(double arg1);
  void on_heightSpinBox_valueChanged(double arg1);

  // model
  void on_delModelBtn_clicked();
  void on_zoneViewCheckBox_stateChanged(int arg1);

  // change modeling parameters
  void on_emit_rate_valueChanged(int value);
  void on_max_speed_valueChanged(int value);
  void on_viscosity_valueChanged(int value);
  void on_int_stiff_valueChanged(int value);
  void on_ext_stiff_valueChanged(int value);
  void on__start_speed_valueChanged(int value);

 public:
  void connectDevicesSlots();
  void setupQScene();

  void _simRunnedDisableInterface();
  void _simStoppedEnableInterface();

 public:
  Ui::MainWindow* _ui;

  //  std::thread _t_sim;

  QTimer _timer;

  QGraphicsScene* _qScene;

 signals:
  void loadModelSignal(const QString&);
  void sendQSceneSignal(QGraphicsScene*);
  void runSimulationSignal(int);

  // set camera
  void setCamToTopSignal();
  void setCamToBottomSignal();
  void setCamToLeftSignal();
  void setCamToRightSignal();
  void setCamToBackSignal();
  void setCamToFrontSignal();

  // parameters changed
  void emitRateChangedSignal(int);
  void maxSpeedChangedSignal(int);
  void viscosityChangedSignal(int);
  void intStiffChangedSignal(int);
  void extStiffChangedSignal(int);
  void startSpeedChangedSignal(int);

  // zone size
  void lengthChangedSignal(double);
  void widthChangedSignal(double);
  void heightChangedSignal(double);

  // model
  void delModelSignal();

  // devices
  void mousePressSignal(QMouseEvent*);
  void mouseReleaseSignal(QMouseEvent*);
  void mouseMoveSignal(QMouseEvent*);
  void wheelSignal(QWheelEvent*);
  void keyPressSignal(QKeyEvent*);
  void keyReleaseSignal(QKeyEvent*);

  void setModelMouseObjectSignal();
  void setCameraMouseObjectSignal();

  // app
  void finishAppSignal();

  // model view radio buttons
  void setWireframeViewSignal();
  void setSimpleViewSignal();
  void setGourandViewSignal();

  // zone view
  void setZoneViewSignal(bool value);
};
