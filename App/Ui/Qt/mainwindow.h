#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <cstddef>
#include <memory>

#include <QMainWindow>
#include <QGraphicsScene>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QWheelEvent>
#include <QTimer>

#include <Domains/Applied/Facade/facade.h>
#include <Domains/Applied/Drawer/drawer.h>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow {
  Q_OBJECT

public:
  MainWindow(QApplication* app, QWidget* parent = nullptr);

  ~MainWindow();

private slots:
  void start();

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

  void on_topCamBtn_clicked();

  void on_bottomCamBtn_clicked();

  void on_frontCamBtn_clicked();

  void on_backCamBtn_clicked();

  void on_rightCamBtn_clicked();

  void on_leftCamBtn_clicked();

  //
  void on_lengthSpinBox_valueChanged(double arg1);

  void on_widthSpinBox_valueChanged(double arg1);

  void on_heightSpinBox_valueChanged(double arg1);

  void on_delModelBtn_clicked();

  void on_zoneViewCheckBox_stateChanged(int arg1);

private:
  void _updateScene();
  void _setupDrawing();
  void _buildSimZone();
  void _addAxis();
  void _removeParticleSystem();
  void _addDefaultCamera();
  void _addDefaultLight();

  void _connectSlots();
  void _simRunnedDisableInterface();
  void _simStoppedEnableInterface();

private:
  Ui::MainWindow* _ui;
  QApplication* _app;

  QTimer _timer;

  std::shared_ptr<Facade> _facade = std::make_shared<Facade>();

  std::shared_ptr<QGraphicsScene> _qScene;

  // objects
  std::shared_ptr<Scene> _scene;
  std::shared_ptr<BaseDrawer> _drawer;
  shared_ptr<BaseObject> _model;
  shared_ptr<Camera> _camera;
  std::shared_ptr<ParticleSystem> psys;

  QPoint _prevMousePos;
  bool _mouseRightButtonPressed = false;
  bool _mouseLeftButtonPressed = false;

  bool _xActive = false;
  bool _yActive = false;

  bool _f1 = false;
  bool _f2 = false;

  bool _simRunned = false;

  size_t _nObjects = 0;
  int _cameraId;
  int _modelId;

  // Particle System
  int _psysId = -1;

  // Simulation Zone
  float _xMax, _yMax, _zMax, _xMin, _yMin, _zMin;
  int _zoneId = -1;

protected:
  // model tranform
  void moveModel(double dx, double dy, double dz);
  void rotateModel(double dx, double dy, double dz);
  void scaleModel(double kx, double ky, double kz);

  // camera tranform
  void rotateCamera(double dx, double dy, double dz);
};

#endif  // MAINWINDOW_H
