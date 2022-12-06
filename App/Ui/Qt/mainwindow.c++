#include "mainwindow.h"

#include <App/Applied/Commands/CameraCommands/add_camera_command.h>
#include <App/Applied/Commands/CameraCommands/load_camera_command.h>
#include <App/Applied/Commands/CameraCommands/rotate_camera_command.h>
#include <App/Applied/Commands/CameraCommands/set_cur_camera_command.h>
#include <App/Applied/Commands/ModelCommands/add_model_command.h>
#include <App/Applied/Commands/ModelCommands/load_model_command.h>
#include <App/Applied/Commands/ModelCommands/load_polygonal_model_command.h>
#include <App/Applied/Commands/ModelCommands/move_model_command.h>
#include <App/Applied/Commands/ModelCommands/remove_model_command.h>
#include <App/Applied/Commands/ModelCommands/rotate_model_command.h>
#include <App/Applied/Commands/ModelCommands/scale_model_command.h>
#include <App/Applied/Commands/SceneCommands/draw_scene_command.h>
#include <App/Applied/Commands/SceneCommands/get_cur_camera_command.h>
#include <App/Applied/Commands/SceneCommands/get_object_command.h>
#include <App/Applied/Commands/SceneCommands/get_scene_command.h>
#include <App/Applied/Drawer/solution_drawer_factory.h>
#include <App/Applied/Exceptions/base_exception.h>
#include <App/Applied/SceneObjects/Model/polygonal_model.h>
#include <App/Applied/SceneObjects/Model/wireframe_model.h>
#include <App/Applied/SceneObjects/ParticleSystem/particle_system.h>
#include <App/Ui/Qt/Drawer/qt_drawer_factory.h>

#include <App/Applied/Commands/LightCommands/add_light_command.hpp>
#include <App/Applied/Commands/ParticleSystemCommands/add_particle_system_command.hpp>
#include <App/Applied/Commands/ParticleSystemCommands/remove_particle_system_command.hpp>
#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <QDebug>
#include <QFileDialog>
#include <QMainWindow>
#include <QMessageBox>
#include <QThread>
#include <thread>

#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget* parent) : QMainWindow(parent), _ui(new Ui::MainWindow) {
  // setup ui
  _ui->setupUi(this);
  _ui->ViewParticleGroupBox->setVisible(false);
  _ui->lightRadioBtn->setVisible(false);
  _ui->modelPressureCheckBox->setVisible(false);

  // setup timer
  _timer.setSingleShot(true);
  connect(&_timer, SIGNAL(timeout()), this, SLOT(start()));
  _timer.start(1000);  // delay for normal screen size initialization
}

MainWindow::~MainWindow() {
  std::cout << "[DBG]: ~MainWindow()\n";
//  delete _qScene;
  delete _ui;
}

void MainWindow::start() {
  std::cout << "[DBG]: MainWindow::start()\n";
  _timer.stop();
  _timer.disconnect();

  // setup
  connectDevicesSlots();
  setupQScene();
  connect(_ui->modelRadioBtn, &QRadioButton::pressed, this, &MainWindow::setModelMouseObjectSignal);
  connect(_ui->cameraRadioBtn, &QRadioButton::pressed, this, &MainWindow::setCameraMouseObjectSignal);

  //  _t_sim = std::thread([&] { simulation_thread_func(); });
  //    qtThr = Thread("Sim Worker", this);
  //  _t_sim = std::thread(&MainWindow::simulation_thread_func, this);
  //  _t_sim = std::thread(MainWindow::simulation_thread_func);
  //  _t_sim.detach();

  // test calls
  //  on_loadPolygonalModelBtn_clicked();
  //
  //  while (true) {  // custom event loop
  //    _app->processEvents();
  //
  //    if (_simRunned) {
  //      psys->run();
  //      _ui->cur_n_particles->setText(QString::number(psys->_particles.size()));
  //      _updateScene();
  //    }
  //  }
}

// mouse and keyboard handle
void MainWindow::mousePressSlot(QMouseEvent* event) {
  emit mousePressSignal(event);
}

void MainWindow::mouseReleaseSlot(QMouseEvent* event) {
  emit mouseReleaseSignal(event);
}

void MainWindow::mouseMoveSlot(QMouseEvent* event) {
  emit mouseMoveSignal(event);
}

void MainWindow::wheelSlot(QWheelEvent* event) {
  emit wheelSignal(event);
}

void MainWindow::keyPressSlot(QKeyEvent* event) {
  emit keyPressSignal(event);
}

void MainWindow::keyReleaseSlot(QKeyEvent* event) {
  emit keyReleaseSignal(event);
}

void MainWindow::setupQScene() {
  // create and setup QScene
  _qScene = new QGraphicsScene(this);
  _qScene->setBackgroundBrush(Qt::black);
  _ui->customGraphicsView->setScene(_qScene);
  _ui->customGraphicsView->setAlignment(Qt::AlignTop | Qt::AlignLeft);
  _qScene->setSceneRect(0, 0, _ui->customGraphicsView->width(), _ui->customGraphicsView->height());

  // test
  //  _qScene->addLine(20, 20, 40, 60, QPen(Qt::yellow));
  //  _qScene->addLine(100, 10, 60, 80, QPen(Qt::yellow));

  emit sendQSceneSignal(_qScene);
}

void MainWindow::connectDevicesSlots() {
  // mouse connect
  QObject::connect(_ui->customGraphicsView, SIGNAL(mousePressSignal(QMouseEvent*)), this,
                   SLOT(mousePressSlot(QMouseEvent*)));
  QObject::connect(_ui->customGraphicsView, SIGNAL(mouseReleaseSignal(QMouseEvent*)), this,
                   SLOT(mouseReleaseSlot(QMouseEvent*)));
  QObject::connect(_ui->customGraphicsView, SIGNAL(mouseMoveSignal(QMouseEvent*)), this,
                   SLOT(mouseMoveSlot(QMouseEvent*)));
  // wheel connect
  QObject::connect(_ui->customGraphicsView, SIGNAL(wheelSignal(QWheelEvent*)), this, SLOT(wheelSlot(QWheelEvent*)));
  // key connect
  QObject::connect(_ui->customGraphicsView, SIGNAL(keyPressSigna(QKeyEvent*)), this, SLOT(keyPressSlot(QKeyEvent*)));
  QObject::connect(_ui->customGraphicsView, SIGNAL(keyReleaseSignal(QKeyEvent*)), this,
                   SLOT(keyReleaseSlot(QKeyEvent*)));

  std::cout << "[DBG]: Slots for mouse and keyboard connected\n";
}

void MainWindow::_simRunnedDisableInterface() {
  _ui->delModelBtn->setDisabled(true);
  _ui->simZoneGroupBox->setDisabled(true);
  _ui->nParticlesSpinBox->setDisabled(true);
  _ui->loadPolygonalModelBtn->setDisabled(true);

  if (_ui->modelRadioBtn->isChecked())
    _ui->cameraRadioBtn->setChecked(true);
  _ui->modelRadioBtn->setDisabled(true);
}

void MainWindow::_simStoppedEnableInterface() {
  _ui->delModelBtn->setEnabled(true);
  _ui->simZoneGroupBox->setEnabled(true);
  _ui->nParticlesSpinBox->setEnabled(true);
  _ui->loadPolygonalModelBtn->setEnabled(true);
  _ui->modelRadioBtn->setEnabled(true);
}

// Get file name and send it to GuiWorker
void MainWindow::on_loadPolygonalModelBtn_clicked() {
  //  QString fileName = "Data/Models/Polygonal//Triangulated/african_head.obj";
  //  QString fileName = "Data/Models/Polygonal/Triangulated/heart.obj";
  QString fileName = "Data/Models/Polygonal/Triangulated/cube.obj";
  //  QString fileName = "Data/Models/Polygonal/Triangulated/wing.obj";
  //  QString fileName = "Data/Models/Polygonal/Triangulated/s_57.obj";

  //  auto fileName = QFileDialog::getOpenFileName();

  if (fileName.isNull())
    return;

  emit loadModelSignal(fileName);
}

void MainWindow::on_runSimBtn_clicked() {
  emit runSimulationSignal(_ui->nParticlesSpinBox->value());
}

void MainWindow::on_topCamBtn_clicked() {
  emit setCamToTopSignal();
}

void MainWindow::on_bottomCamBtn_clicked() {
  emit setCamToBottomSignal();
}

void MainWindow::on_frontCamBtn_clicked() {
  emit setCamToFrontSignal();
}

void MainWindow::on_backCamBtn_clicked() {
  emit setCamToBackSignal();
}

void MainWindow::on_rightCamBtn_clicked() {
  emit setCamToRightSignal();
}

void MainWindow::on_leftCamBtn_clicked() {
  emit setCamToLeftSignal();
}

void MainWindow::on_lengthSpinBox_valueChanged(double arg1) {
  //  if (_psysId != -1) {
  //    _removeParticleSystem();
  //  }
  //
  //  _buildSimZone();
}

void MainWindow::on_widthSpinBox_valueChanged(double arg1) {
  //  if (_psysId != -1) {
  //    _removeParticleSystem();
  //  }
  //
  //  _buildSimZone();
}

void MainWindow::on_heightSpinBox_valueChanged(double arg1) {
  //  if (_psysId != -1) {
  //    _removeParticleSystem();
  //  }
  //
  //  _buildSimZone();
}

void MainWindow::on_delModelBtn_clicked() {
  emit delModelSignal();
}

void MainWindow::on_zoneViewCheckBox_stateChanged(int arg1) {
}

// modeling parameters
void MainWindow::on_emit_rate_valueChanged(int value) {
  emit emitRateChangedSignal(value);
}

void MainWindow::on_max_speed_valueChanged(int value) {
  emit maxSpeedChangedSignal(value);
}

void MainWindow::on_viscosity_valueChanged(int value) {
  emit viscosityChangedSignal(value);
}

void MainWindow::on_int_stiff_valueChanged(int value) {
  emit intStiffChangedSignal(value);
}

void MainWindow::on_ext_stiff_valueChanged(int value) {
  emit extStiffChangedSignal(value);
}

void MainWindow::on__start_speed_valueChanged(int value) {
  emit startSpeedChangedSignal(value);
}
