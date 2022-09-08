#include "main_window.h"
#include "ui_main_window.h"

#include <QDebug>
#include <QFileDialog>
#include <QMessageBox>

#include <Domains/Applied/Commands/CameraCommands/add_camera_command.h>
#include <Domains/Applied/Commands/CameraCommands/load_camera_command.h>
#include <Domains/Applied/Commands/CameraCommands/rotate_camera_command.h>
#include <Domains/Applied/Commands/CameraCommands/set_cur_camera_command.h>
#include <Domains/Applied/Commands/ModelCommands/add_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/load_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/load_polygonal_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/move_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/remove_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/rotate_model_command.h>
#include <Domains/Applied/Commands/ModelCommands/scale_model_command.h>
#include <Domains/Applied/Commands/SceneCommands/draw_scene_command.h>
#include <Domains/Applied/Commands/SceneCommands/get_cur_camera_command.h>
#include <Domains/Applied/Commands/SceneCommands/get_object_command.h>
#include <Domains/Applied/Commands/SceneCommands/get_scene_command.h>
#include <Domains/Applied/Commands/SceneCommands/get_scene_command.h>
#include <Domains/Applied/Commands/LightCommands/add_light_command.hpp>

#include <Domains/Applied/Commands/ParticleSystemCommands/add_particle_system_command.hpp>
#include <Domains/Applied/Commands/ParticleSystemCommands/remove_particle_system_command.hpp>
#include <Domains/Applied/Drawer/solution_drawer_factory.h>
#include <Domains/Applied/Exceptions/base_exception.h>
#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <Domains/Applied/SceneObjects/Model/polygonal_model.h>
#include <Domains/Applied/SceneObjects/Model/wireframe_model.h>
#include <Domains/Applied/SceneObjects/ParticleSystem/particle_system.h>

#include <Domains/Ui/Qt/Drawer/qt_drawer_factory.h>

MainWindow::MainWindow(QApplication* app, QWidget* parent)
    : QMainWindow(parent), _ui(new Ui::MainWindow), _app(app) {
  _ui->setupUi(this);
  _ui->ViewParticleGroupBox->setVisible(false);  // TODO: implement
  _ui->lightRadioBtn->setVisible(false);

  _timer.setSingleShot(true);
  connect(&_timer, SIGNAL(timeout()), this, SLOT(start()));
  _timer.start(1000);  // delay for normal screen size initialization
}

MainWindow::~MainWindow() { delete _ui; }

void MainWindow::start() {
  qDebug() << "start()";
  _timer.stop();
  _timer.disconnect();

  // setup
  _connectSlots();
  _setupDrawing();
  _addDefaultCamera();
  _addDefaultLight();
  _addAxis();

  // test calls
  on_loadPolygonalModelBtn_clicked();

  while (true) {  // custom event loop
    _app->processEvents();

    if (_simRunned) {
      psys->run();
      _updateScene();
    }
  }
}

// mouse and keyboard handle
void MainWindow::mousePressSlot(QMouseEvent* event) {
  //    qDebug() << "Mouse pressed: x = " << event.x() << "; y = " <<
  //    event.y();

  _prevMousePos = event->pos();

  if (event->button() == Qt::LeftButton)
    _mouseLeftButtonPressed = true;

  if (event->button() == Qt::RightButton)
    _mouseRightButtonPressed = true;

  event->accept();
}

void MainWindow::mouseReleaseSlot(QMouseEvent* event) {
  _mouseLeftButtonPressed = false;
  _mouseRightButtonPressed = false;

  event->accept();
}

void MainWindow::mouseMoveSlot(QMouseEvent* event) {
  float dx = event->x() - _prevMousePos.x();
  float dy = event->y() - _prevMousePos.y();

  //      qDebug() << "Mouse moved: dx = " << dx << "; dy = " << dy;

  _prevMousePos = event->pos();

  if (_mouseLeftButtonPressed) {
    if (_ui->cameraRadioBtn->isChecked()) {
      // TODO: Move camera
    } else if (_ui->modelRadioBtn->isChecked()) {
      if (_psysId != -1) {
        _removeParticleSystem();
      }
      moveModel(dx * 0.001, dy * 0.001, 0);
    } else {
    }
    _updateScene();
  } else if (_mouseRightButtonPressed) {
    if (_ui->cameraRadioBtn->isChecked()) {
      rotateCamera(dy, dx, 0);
    } else if (_ui->modelRadioBtn->isChecked()) {
      if (_psysId != -1) {
        _removeParticleSystem();
      }
      rotateModel(-dy, -dx, 0);
    } else {
    }
    _updateScene();
  }

  event->accept();
}

void MainWindow::wheelSlot(QWheelEvent* event) {
  //    qDebug() << "wheelSlot(): " << event->angleDelta();

  float factor = 1 + event->angleDelta().y() / 120 * 0.05;

  if (_ui->cameraRadioBtn->isChecked()) {
    // TODO: zoom camera
  } else if (_ui->modelRadioBtn->isChecked()) {
    if (_psysId != -1) {
      _removeParticleSystem();
    }
    scaleModel(factor, factor, factor);
    _updateScene();
  } else {
  }

  event->accept();
}

void MainWindow::keyPressSlot(QKeyEvent* event) {
  Q_UNUSED(event)
  //  if (event->key() == Qt::Key_Shift) {
  //  }
}

void MainWindow::keyReleaseSlot(QKeyEvent* event) {
  Q_UNUSED(event)
  //  if (event->key() == Qt::Key_Shift) {
  //  }
}

// transform model methods
void MainWindow::moveModel(double dx, double dy, double dz) {
  auto moveModelCommand = MoveModelCommand(_model, dx, dy, dz);

  try {
    _facade->executeCommand(moveModelCommand);
    _updateScene();
  } catch (const BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

void MainWindow::rotateModel(double dx, double dy, double dz) {
  auto rotateModelComand =
  RotateModelCommand(_model, dx * 0.01, dy * 0.01, dz * 0.01);

  try {
    _facade->executeCommand(rotateModelComand);
    _updateScene();
  } catch (const BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

void MainWindow::scaleModel(double kx, double ky, double kz) {
  auto scaleModelComand = ScaleModelCommand(_model, kx, ky, kz);

  try {
    _facade->executeCommand(scaleModelComand);
    _updateScene();
  } catch (const BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

// transform camera methods
void MainWindow::rotateCamera(double dx, double dy, double dz) {
  auto rotateCameraComand =
  RotateCameraCommand(_camera, dx * 0.01, dy * 0.01, dz * 0.01);

  try {
    _facade->executeCommand(rotateCameraComand);
    _updateScene();
  } catch (const BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

//
void MainWindow::_addDefaultCamera() {
  Vec3f center = { 0, 0, 0 };
  Vec3f eye = { 3, 2, 2 };
  Vec3f up = { 0, 0, 1 };

  auto addCameraCommand = AddCameraCommand(center, eye, up);
  try {
    _facade->executeCommand(addCameraCommand);
  } catch (BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
    return;
  }

  _nObjects++;

  auto setCurCameraCommand = SetCurCameraCommand(0);
  _facade->executeCommand(setCurCameraCommand);

  auto getCurCameraCommand = GetCurCameraCommand(_camera);
  _facade->executeCommand(getCurCameraCommand);
}

void MainWindow::_addDefaultLight() {
  //  Vec3f dir = { -2, -2, -1 };
  Vec3f dir = { 3, 2, 2 };

  auto addLightCommand = AddLightCommand(dir);

  try {
    _facade->executeCommand(addLightCommand);
  } catch (BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
    return;
  }

  _nObjects++;
}

void MainWindow::_setupDrawing() {
  // create and setup QScene
  _qScene = std::make_shared<QGraphicsScene>(this);
  _qScene->setBackgroundBrush(Qt::black);

  _ui->customGraphicsView->setScene(_qScene.get());
  _ui->customGraphicsView->setAlignment(Qt::AlignTop | Qt::AlignLeft);

  int minSize =
  min(_ui->customGraphicsView->width(), _ui->customGraphicsView->height());

  _qScene->setSceneRect(0, 0, minSize, minSize);

  // create drawer
  auto solution = std::make_shared<SolutionDrawerFactory>();
  solution->registration<QtDrawerFactory>("Qt", _qScene);

  auto factory = solution->createFactory("Qt");
  _drawer = factory->createDrawer();

  // get Scene
  auto getSceneCommand = GetSceneCommand(_scene);
  try {
    _facade->executeCommand(getSceneCommand);
  } catch (BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

void MainWindow::_connectSlots() {
  // mouse connect
  QObject::connect(_ui->customGraphicsView,
                   SIGNAL(mousePressSignal(QMouseEvent*)), this,
                   SLOT(mousePressSlot(QMouseEvent*)));
  QObject::connect(_ui->customGraphicsView,
                   SIGNAL(mouseReleaseSignal(QMouseEvent*)), this,
                   SLOT(mouseReleaseSlot(QMouseEvent*)));
  QObject::connect(_ui->customGraphicsView,
                   SIGNAL(mouseMoveSignal(QMouseEvent*)), this,
                   SLOT(mouseMoveSlot(QMouseEvent*)));
  // wheel connect
  QObject::connect(_ui->customGraphicsView, SIGNAL(wheelSignal(QWheelEvent*)),
                   this, SLOT(wheelSlot(QWheelEvent*)));
  // key connect
  QObject::connect(_ui->customGraphicsView, SIGNAL(keyPressSigna(QKeyEvent*)),
                   this, SLOT(keyPressSlot(QKeyEvent*)));
  QObject::connect(_ui->customGraphicsView,
                   SIGNAL(keyReleaseSignal(QKeyEvent*)), this,
                   SLOT(keyReleaseSlot(QKeyEvent*)));

  qDebug() << "Slots connected";
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

void MainWindow::_buildSimZone() {
  if (_zoneId != -1) {
    auto removeModelCommand = RemoveModelCommand(_zoneId);
    _facade->executeCommand(removeModelCommand);
    _nObjects--;
    _zoneId = -1;
  }

  // 1. find min, max points
  const auto pmodel = std::dynamic_pointer_cast<PolygonalModel>(_model);
  const auto& vertices = pmodel->components()->vertices();

  _xMax = _xMin = vertices[0].x();
  _yMax = _yMin = vertices[0].y();
  _zMax = _zMin = vertices[0].z();

  for (const auto& vertex : vertices) {
    if (vertex.x() > _xMax)
      _xMax = vertex.x();
    if (vertex.y() > _yMax)
      _yMax = vertex.y();
    if (vertex.z() > _zMax)
      _zMax = vertex.z();
    if (vertex.x() < _xMin)
      _xMin = vertex.x();
    if (vertex.y() < _yMin)
      _yMin = vertex.y();
    if (vertex.z() < _zMin)
      _zMin = vertex.z();
  }

  float dx = _xMax - _xMin;
  float dy = _yMax - _yMin;
  float dz = _zMax - _zMin;

  // 2. find bound limits
  float length = (_ui->lengthSpinBox->value() - 1) / 2;
  float width = (_ui->widthSpinBox->value() - 1) / 2;
  float height = (_ui->heightSpinBox->value() - 1) / 2;

  _xMax += length * dx;
  _xMin -= length * dx;
  _yMax += width * dy;
  _yMin -= width * dy;
  _zMax += height * dz;
  _zMin -= height * dz;

  // 3. create model by points
  if (_ui->zoneViewCheckBox->isChecked()) {
    std::vector<Vec3f> boundVerts;
    std::vector<Edge> boundEdges;

    boundVerts.push_back(Vec3f(_xMax, _yMax, _zMax));
    boundVerts.push_back(Vec3f(_xMax, _yMax, _zMin));
    boundVerts.push_back(Vec3f(_xMax, _yMin, _zMax));
    boundVerts.push_back(Vec3f(_xMax, _yMin, _zMin));

    boundVerts.push_back(Vec3f(_xMin, _yMax, _zMax));
    boundVerts.push_back(Vec3f(_xMin, _yMax, _zMin));
    boundVerts.push_back(Vec3f(_xMin, _yMin, _zMax));
    boundVerts.push_back(Vec3f(_xMin, _yMin, _zMin));

    boundEdges.push_back(Edge(0, 1));
    boundEdges.push_back(Edge(0, 2));
    boundEdges.push_back(Edge(3, 1));
    boundEdges.push_back(Edge(3, 2));

    boundEdges.push_back(Edge(4, 5));
    boundEdges.push_back(Edge(4, 6));
    boundEdges.push_back(Edge(7, 5));
    boundEdges.push_back(Edge(7, 6));

    boundEdges.push_back(Edge(0, 4));
    boundEdges.push_back(Edge(1, 5));
    boundEdges.push_back(Edge(2, 6));
    boundEdges.push_back(Edge(3, 7));

    shared_ptr<WireframeModelComponents> comp =
    std::make_shared<WireframeModelComponents>(boundVerts, boundEdges);

    shared_ptr<BaseObject> bound = std::make_shared<WireframeModel>(
    comp, Vec3f(0, 0, 0), Color(255, 255, 0, 255));

    auto addModelCommand = AddModelCommand(bound);
    _facade->executeCommand(addModelCommand);

    _zoneId = _nObjects;
    _nObjects++;
  }
}

void MainWindow::_addAxis() {
  const Vec3f OX_beg = { 0, 0, 0 };
  const Vec3f OY_beg = { 0, 0, 0 };
  const Vec3f OZ_beg = { 0, 0, 0 };

  const Vec3f OX_end = { 3, 0, 0 };
  const Vec3f OY_end = { 0, 3, 0 };
  const Vec3f OZ_end = { 0, 0, 3 };

  // OX axis
  std::vector<Vec3f> vertices;
  vertices.push_back(OX_beg);
  vertices.push_back(OX_end);
  std::vector<Edge> edges;
  edges.push_back(Edge(0, 1));
  shared_ptr<WireframeModelComponents> comp =
  std::make_shared<WireframeModelComponents>(vertices, edges);
  shared_ptr<BaseObject> axisModel =
  std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(255, 0, 0, 255));
  auto addModelCommand = AddModelCommand(axisModel);
  _facade->executeCommand(addModelCommand);
  _nObjects++;

  // OY axis
  vertices.clear();
  vertices.push_back(OY_beg);
  vertices.push_back(OY_end);
  edges.clear();
  edges.push_back(Edge(0, 1));
  comp = std::make_shared<WireframeModelComponents>(vertices, edges);
  axisModel =
  std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(0, 255, 0, 255));
  addModelCommand = AddModelCommand(axisModel);
  _facade->executeCommand(addModelCommand);
  _nObjects++;

  // OZ axis
  vertices.clear();
  vertices.push_back(OZ_beg);
  vertices.push_back(OZ_end);
  edges.clear();
  edges.push_back(Edge(0, 1));
  comp = std::make_shared<WireframeModelComponents>(vertices, edges);
  axisModel =
  std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(0, 0, 255, 255));
  addModelCommand = AddModelCommand(axisModel);
  _facade->executeCommand(addModelCommand);
  _nObjects++;
}

void MainWindow::_removeParticleSystem() {
  auto removeParticleSystemCommand = RemoveParticleSystemCommand(_psysId);
  _facade->executeCommand(removeParticleSystemCommand);
  _psysId = -1;
  _nObjects--;
}

void MainWindow::_updateScene() {
  // draw scene
  DrawMode mode;
  if (_ui->wireframeRadioBtn->isChecked()) {
    mode = WIREFRAME;
  } else if (_ui->simpleRadioBtn->isChecked()) {
    mode = SIMPLE;
  } else {
    mode = GOURAND;
  }
  ParticleMode particleMode;
  //  if (_ui->circleParticleRadioBtn->isChecked()) {
  //    particleMode = SPHERE;
  //  } else {
  particleMode = VECTOR;
  //  }

  auto drawSceneCommand =
  DrawSceneCommand(_scene, _drawer, _camera, mode, particleMode);
  try {
    _facade->executeCommand(drawSceneCommand);
  } catch (BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
  }
}

// buttons handle
void MainWindow::on_loadPolygonalModelBtn_clicked() {
  //  QString fileName = "Data/Models/Polygonal//Triangulated/african_head.obj";
  //  QString fileName = "Data/Models/Polygonal/Triangulated/heart.obj";
  QString fileName = "Data/Models/Polygonal/Triangulated/cube.obj";

  //  auto fileName = QFileDialog::getOpenFileName();

  if (fileName.isNull())
    return;

  auto loadModelCommand =
  LoadPolygonalModelCommand(_model, fileName.toUtf8().data());
  try {
    _facade->executeCommand(loadModelCommand);
  } catch (const BaseException& ex) {
    QMessageBox::warning(this, "Error", QString(ex.what()));
    return;
  }

  auto addModelCommand = AddModelCommand(_model);
  _facade->executeCommand(addModelCommand);

  scaleModel(0.2, 0.2, 0.2);

  _modelId = _nObjects;
  _nObjects++;

  _buildSimZone();

  _ui->delModelBtn->setEnabled(true);
  //  _ui->loadPolygonalModelBtn->setDisabled(true);
  _ui->modelRadioBtn->setEnabled(true);
  _ui->runSimBtn->setEnabled(true);

  _updateScene();
}

void MainWindow::on_runSimBtn_clicked() {
  if (!_simRunned) {
    if (_psysId == -1) {
      psys = std::make_shared<ParticleSystem>(_model);

      auto addParticleSystemCommand = AddParticleSystemCommand(psys);
      _facade->executeCommand(addParticleSystemCommand);

      _psysId = _nObjects;
      _nObjects++;

      const int nParticles = _ui->nParticlesSpinBox->value();
      psys->initialize(nParticles);
      psys->createExample(_xMin * 30, _xMax * 30, _yMin * 30, _yMax * 30,
                          _zMin * 30, _zMax * 30);
    }

    _simRunnedDisableInterface();

    _ui->runSimBtn->setText("Остановить симуляцию");
  } else {
    _simStoppedEnableInterface();

    _ui->runSimBtn->setText("Запустить симуляцию");
  }

  _simRunned = !_simRunned;
}

void MainWindow::on_topCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 1, 0));
  _camera->setEye(Vec3f(0, 0, 3));
}

void MainWindow::on_bottomCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 1, 0));
  _camera->setEye(Vec3f(0, 0, -3));
}

void MainWindow::on_frontCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 0, 1));
  _camera->setEye(Vec3f(3, 0, 0));
}

void MainWindow::on_backCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 0, 1));
  _camera->setEye(Vec3f(-3, 0, 0));
}

void MainWindow::on_rightCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 0, 1));
  _camera->setEye(Vec3f(0, -3, 0));
}

void MainWindow::on_leftCamBtn_clicked() {
  _camera->setCenter(Vec3f(0, 0, 0));
  _camera->setUp(Vec3f(0, 0, 1));
  _camera->setEye(Vec3f(0, 3, 0));
}

void MainWindow::on_lengthSpinBox_valueChanged(double arg1) {
  if (_psysId != -1) {
    _removeParticleSystem();
  }

  _buildSimZone();
}

void MainWindow::on_widthSpinBox_valueChanged(double arg1) {
  if (_psysId != -1) {
    _removeParticleSystem();
  }

  _buildSimZone();
}

void MainWindow::on_heightSpinBox_valueChanged(double arg1) {
  if (_psysId != -1) {
    _removeParticleSystem();
  }

  _buildSimZone();
}

void MainWindow::on_delModelBtn_clicked() {
  if (_psysId != -1) {
    _removeParticleSystem();
  }

  if (_zoneId != -1) {
    auto removeModelCommand = RemoveModelCommand(_zoneId);
    _facade->executeCommand(removeModelCommand);
    _nObjects--;
    _zoneId = -1;
  }

  auto removeModelCommand = RemoveModelCommand(_modelId);
  _facade->executeCommand(removeModelCommand);

  _model.reset();
  _modelId = -1;
  _nObjects--;

  _ui->delModelBtn->setDisabled(true);
  _ui->loadPolygonalModelBtn->setEnabled(true);

  if (_ui->modelRadioBtn->isChecked())
    _ui->cameraRadioBtn->setChecked(true);
  _ui->modelRadioBtn->setDisabled(true);
  _ui->runSimBtn->setDisabled(true);

  //  _updateScene();
}

void MainWindow::on_zoneViewCheckBox_stateChanged(int arg1) {}
