#pragma once

#include <QApplication>
#include <QGraphicsScene>
#include <QObject>
#include <chrono>

// scene objects
#include "particle_system.h"
#include "wireframe_model.h"

// commands
#include "LightCommands/add_light_command.hpp"
#include "ParticleSystemCommands/add_particle_system_command.hpp"
#include "ParticleSystemCommands/remove_particle_system_command.hpp"
#include "add_camera_command.h"
#include "add_model_command.h"
#include "base_exception.h"
#include "draw_scene_command.h"
#include "facade.h"
#include "get_cur_camera_command.h"
#include "get_scene_command.h"
#include "load_polygonal_model_command.h"
#include "move_model_command.h"
#include "qt_drawer_factory.h"
#include "remove_model_command.h"
#include "rotate_camera_command.h"
#include "rotate_model_command.h"
#include "scale_model_command.h"
#include "set_cur_camera_command.h"
#include "solution_drawer_factory.h"

// workers
#include "gui_worker.h"
#include "sim_worker.h"

class MainWorker : public QObject {
  Q_OBJECT

  enum MouseObject { CAMERA, MODEL, LIGHT };

 public:
  MainWorker(QApplication& app) : _simWorker(), _app(app) {
    std::cout << "[DBG]: MainWorker()\n";
    connectUiWorker();

    while (!_finishApp) {  // custom event loop
      _app.processEvents();
      if (_simRunned) {
        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - _prev_time).count() >
            1) {
          // send FPC
          emit sendFpsSignal(_FPS, _psys->_particles.size());
          _FPS = 0;
          _prev_time = std::chrono::steady_clock::now();
//          std::cout << "[DBG]: Simulation working...\n";
        }
        _psys->run();
        updateScene();
        ++_FPS;
      }
    }
    std::cout << "[DBG]: Event loop finished\n";
  }
  ~MainWorker() override {
    std::cout << "[DBG]: ~MainWorker()\n";
  }

  void updateScene() {
    // draw scene
    ParticleMode particleMode;
    //  if (_ui->circleParticleRadioBtn->isChecked()) {
    //    particleMode = SPHERE;
    //  } else {
    particleMode = VECTOR;
    //  }

    auto drawSceneCommand = DrawSceneCommand(_scene, _drawer, _camera, _viewMode, particleMode);
    try {
      _facade->executeCommand(drawSceneCommand);
    } catch (BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute DrawSceneCommand\n";
    }
  }

  void connectUiWorker() {
    //
    connect(&_guiWorker, &GuiWorker::loadModelSignal, this, &MainWorker::loadModelSlot);
    connect(&_guiWorker, &GuiWorker::sendQSceneSignal, this, &MainWorker::getQSceneSlot);
    connect(&_guiWorker, &GuiWorker::runSimulationSignal, this, &MainWorker::runSimulationSlot);

    // set camera
    connect(&_guiWorker, &GuiWorker::setCamToTopSignal, this, &MainWorker::setCamToTopSlot);
    connect(&_guiWorker, &GuiWorker::setCamToBottomSignal, this, &MainWorker::setCamToBottomSlot);
    connect(&_guiWorker, &GuiWorker::setCamToBackSignal, this, &MainWorker::setCamToBackSlot);
    connect(&_guiWorker, &GuiWorker::setCamToFrontSignal, this, &MainWorker::setCamToFrontSlot);
    connect(&_guiWorker, &GuiWorker::setCamToLeftSignal, this, &MainWorker::setCamToLeftSlot);
    connect(&_guiWorker, &GuiWorker::setCamToRightSignal, this, &MainWorker::setCamToRightSlot);

    // parameters changed
    connect(&_guiWorker, &GuiWorker::emitRateChangedSignal, this, &MainWorker::emitRateChangedSlot);
    connect(&_guiWorker, &GuiWorker::maxSpeedChangedSignal, this, &MainWorker::maxSpeedChangedSlot);
    connect(&_guiWorker, &GuiWorker::viscosityChangedSignal, this, &MainWorker::viscosityChangedSlot);
    connect(&_guiWorker, &GuiWorker::intStiffChangedSignal, this, &MainWorker::intStiffChangedSlot);
    connect(&_guiWorker, &GuiWorker::extStiffChangedSignal, this, &MainWorker::extStiffChangedSlot);
    connect(&_guiWorker, &GuiWorker::startSpeedChangedSignal, this, &MainWorker::startSpeedChangedSlot);

    // model
    connect(&_guiWorker, &GuiWorker::delModelSignal, this, &MainWorker::delModelSlot);

    // devices
    connect(&_guiWorker, &GuiWorker::mousePressSignal, this, &MainWorker::mousePressSlot);
    connect(&_guiWorker, &GuiWorker::mouseReleaseSignal, this, &MainWorker::mouseReleaseSlot);
    connect(&_guiWorker, &GuiWorker::mouseMoveSignal, this, &MainWorker::mouseMoveSlot);
    connect(&_guiWorker, &GuiWorker::wheelSignal, this, &MainWorker::wheelSlot);
    connect(&_guiWorker, &GuiWorker::keyPressSignal, this, &MainWorker::keyPressSlot);
    connect(&_guiWorker, &GuiWorker::keyReleaseSignal, this, &MainWorker::keyReleaseSlot);

    connect(&_guiWorker, &GuiWorker::setModelMouseObjectSignal, this, &MainWorker::setModelMouseObjectSlot);
    connect(&_guiWorker, &GuiWorker::setCameraMouseObjectSignal, this, &MainWorker::setCameraMouseObjectSlot);

    connect(&_guiWorker, &GuiWorker::finishAppSignal, this, &MainWorker::finishAppSlot);

    // zone size
    connect(&_guiWorker, &GuiWorker::lengthChangedSignal, this, &MainWorker::lengthChangedSlot);
    connect(&_guiWorker, &GuiWorker::widthChangedSignal, this, &MainWorker::widthChangedSlot);
    connect(&_guiWorker, &GuiWorker::heightChangedSignal, this, &MainWorker::heightChangedSlot);

    // to Gui
    connect(this, &MainWorker::sendFpsSignal, &_guiWorker, &GuiWorker::getFpsSlot);
    connect(this, &MainWorker::simRunned, &_guiWorker, &GuiWorker::simRunnedSignal);
    connect(this, &MainWorker::simStopped, &_guiWorker, &GuiWorker::simStoppedSignal);

    // model view radio buttons
    connect(&_guiWorker, &GuiWorker::setWireframeViewSignal, this, &MainWorker::setWireframeViewSlot);
    connect(&_guiWorker, &GuiWorker::setSimpleViewSignal, this, &MainWorker::setSimpleViewSlot);
    connect(&_guiWorker, &GuiWorker::setGourandViewSignal, this, &MainWorker::setGourandViewSlot);

    connect(&_guiWorker, &GuiWorker::setZoneViewSignal, this, &MainWorker::setZoneViewSlot);

    connect(&_guiWorker, &GuiWorker::nParticlesChangedSignal, this, &MainWorker::nParticlesChangedSlot);
  }

  void scaleModel(double kx, double ky, double kz) {
    auto scaleModelComand = ScaleModelCommand(_model, kx, ky, kz);

    try {
      _facade->executeCommand(scaleModelComand);
      //      _updateScene();
    } catch (const BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute ScaleModelCommand\n";
    }

    updateScene();
  }

  void addDefaultCamera() {
    Vec3f center = {0, 0, 0};
    Vec3f eye = {3, 2, 2};
    Vec3f up = {0, 0, 1};

    auto addCameraCommand = AddCameraCommand(center, eye, up);
    try {
      _facade->executeCommand(addCameraCommand);
    } catch (BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute AddCameraCommand\n";
      return;
    }

    _nObjects++;

    auto setCurCameraCommand = SetCurCameraCommand(0);
    _facade->executeCommand(setCurCameraCommand);

    auto getCurCameraCommand = GetCurCameraCommand(_camera);
    _facade->executeCommand(getCurCameraCommand);
  }

  void addDefaultLight() {
    //  Vec3f dir = { -2, -2, -1 };
    Vec3f dir = {3, 2, 2};

    auto addLightCommand = AddLightCommand(dir);

    try {
      _facade->executeCommand(addLightCommand);
    } catch (BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute AddLightCommand\n";
      return;
    }

    _nObjects++;
  }

  void addAxis() {
    const Vec3f OX_beg = {0, 0, 0};
    const Vec3f OY_beg = {0, 0, 0};
    const Vec3f OZ_beg = {0, 0, 0};

    const Vec3f OX_end = {3, 0, 0};
    const Vec3f OY_end = {0, 3, 0};
    const Vec3f OZ_end = {0, 0, 3};

    // OX axis
    std::vector<Vec3f> vertices;
    vertices.push_back(OX_beg);
    vertices.push_back(OX_end);
    std::vector<Edge> edges;
    edges.push_back(Edge(0, 1));
    shared_ptr<WireframeModelComponents> comp = std::make_shared<WireframeModelComponents>(vertices, edges);
    shared_ptr<BaseObject> axisModel = std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(255, 0, 0, 255));
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
    axisModel = std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(0, 255, 0, 255));
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
    axisModel = std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(0, 0, 255, 255));
    addModelCommand = AddModelCommand(axisModel);
    _facade->executeCommand(addModelCommand);
    _nObjects++;
  }

  void rebuildSimZone() {
    std::cout << "[DBG]: rebuildSimZone()\n";
    std::cout << "[DBG]: Before: zoneId=" << _zoneId << "; nObjects=" << _nObjects << std::endl;
    if (_zoneId != -1) {
      auto removeModelCommand = RemoveModelCommand(_zoneId);
      _facade->executeCommand(removeModelCommand);
      --_nObjects;
      if (_modelId > _zoneId) {
        --_modelId;
      }
      _zoneId = -1;
    }

    if (_model) {
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
    } else {
      _xMin = -0.2;
      _xMax = 0.2;
      _yMin = -0.2;
      _yMax = 0.2;
      _zMin = -0.2;
      _zMax = 0.2;
    }

    model_bound.xMin = _xMin * 30;
    model_bound.xMax = _xMax * 30;
    model_bound.yMin = _yMin * 30;
    model_bound.yMax = _yMax * 30;
    model_bound.zMin = _zMin * 30;
    model_bound.zMax = _zMax * 30;

    float dx = _xMax - _xMin;
    float dy = _yMax - _yMin;
    float dz = _zMax - _zMin;

    // 2. find bound limits
    float length = (_length - 1) / 2.0f;
    float width = (_width - 1) / 2.0f;
    float height = (_height - 1) / 2.0f;

    _xMax += length * dx;
    _xMin -= length * dx;
    _yMax += width * dy;
    _yMin -= width * dy;
    _zMax += height * dz;
    _zMin -= height * dz;

    // 3. create model by points
    if (_isZoneVisible) {
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

      shared_ptr<WireframeModelComponents> comp = std::make_shared<WireframeModelComponents>(boundVerts, boundEdges);

      shared_ptr<BaseObject> bound = std::make_shared<WireframeModel>(comp, Vec3f(0, 0, 0), Color(255, 255, 0, 255));

      auto addModelCommand = AddModelCommand(bound);
      _facade->executeCommand(addModelCommand);

      _zoneId = _nObjects;
      _nObjects++;
    }
    std::cout << "[DBG]: After: zoneId=" << _zoneId << "; nObjects=" << _nObjects << std::endl;
    std::cout << "[DBG]: After: model id=" << _modelId << std::endl;
    updateScene();
  }

 public slots:
  void loadModelSlot(const QString& fileName) {
    std::cout << "[DBG]: MainWorker::loadModelSlot()\n";
    std::cout << "[DBG]: Before: modelId=" << _modelId << "; nObjects=" << _nObjects << std::endl;

    _simWorker.setRunning(false);

    auto loadModelCommand = LoadPolygonalModelCommand(_model, fileName.toUtf8().data());
    try {
      _facade->executeCommand(loadModelCommand);
    } catch (const BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to load model\n";
      return;
    }

    auto addModelCommand = AddModelCommand(_model);
    _facade->executeCommand(addModelCommand);

    scaleModel(0.2, 0.2, 0.2);

    _modelId = _nObjects;
    _nObjects++;

    //    rebuildSimZone();
    std::cout << "[DBG]: After: modelId=" << _modelId << "; nObjects=" << _nObjects << std::endl;
    rebuildSimZone();
    updateScene();
  }

  void getQSceneSlot(QGraphicsScene* qScene) {
    _qScene = qScene;

    //    std::cout << "[DBG]: MainWorker::getQSceneSlot()\n";
    //    qScene->addLine(20, 20, 40, 60, QPen(Qt::blue));
    //    qScene->addLine(100, 10, 60, 80, QPen(Qt::yellow));

    // create drawer
    auto solution = std::make_shared<SolutionDrawerFactory>();
    solution->registration<QtDrawerFactory>("Qt", std::shared_ptr<QGraphicsScene>(qScene));

    auto factory = solution->createFactory("Qt");
    _drawer = factory->createDrawer();

    // get Scene
    auto getSceneCommand = GetSceneCommand(_scene);
    try {
      _facade->executeCommand(getSceneCommand);
    } catch (BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute GetSceneCommand\n";
    }

    addDefaultCamera();
    addDefaultLight();
    addAxis();
    setCamToLeftSlot();
    //    _viewMode = WIREFRAME;
    updateScene();
  }

  void runSimulationSlot(int nParticles) {
    std::cout << "[DBG]: runSimulationSlot()\n";
    if (!_simRunned) {
      if (_psysId == -1) {
        std::cout << "[DBG]: Create ParticleSystem\n";
        _psys = std::make_shared<ParticleSystem>(_model, model_bound);

        auto addParticleSystemCommand = AddParticleSystemCommand(_psys);
        _facade->executeCommand(addParticleSystemCommand);

        _psysId = _nObjects;
        _nObjects++;

        _psys->initialize(_nParticles);
        _psys->createExample(_xMin * 30, _xMax * 30, _yMin * 30, _yMax * 30, _zMin * 30, _zMax * 30);

        _psys->_emit_rate = 0.2f * ((float)_emit_rate / 100.0f);
        _psys->_limit = 600.0f * ((float)_max_speed / 100.0f);
        _psys->_viscosity = 0.4f * ((float)_viscosity / 100.0f);
        _psys->start_speed = 10 * ((float) _start_speed / 100.0f);
        std::cout << "[DBG]: start speed = " << _psys->start_speed << std::endl;
      }
      //      _simWorker.setFields(_scene, _drawer, _camera, _psys, _facade, _qScene);
      _FPS = 0;
      _prev_time = std::chrono::steady_clock::now();
      //      connect(&qThread, &QThread::started, &_simWorker, &SimWorker::run);
      //      connect(&_simWorker, &SimWorker::finished, &qThread, &QThread::terminate);
      //      _simWorker.moveToThread(&qThread);
      //      _simWorker.setRunning(true);
      //      qThread.start();
      emit simRunned();
      extStiffChangedSlot(80);
      _mouseObject = CAMERA;
      _simRunned = true;
    } else {
      //      _simWorker.setRunning(false);
      //      qThread.;
      //      _simWorker.stop();
      emit sendFpsSignal(0, 0);
      emit simStopped();
      _simRunned = false;
    }
  }

  void finishAppSlot() {
    _finishApp = true;
  };

  // set camera
  void setCamToTopSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 1, 0));
    _camera->setEye(Vec3f(0, 0, 3));
    updateScene();
  }

  void setCamToBottomSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 1, 0));
    _camera->setEye(Vec3f(0, 0, -3));
    updateScene();
  }

  void setCamToFrontSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 0, 1));
    _camera->setEye(Vec3f(3, 0, 0));
    updateScene();
  }

  void setCamToBackSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 0, 1));
    _camera->setEye(Vec3f(-3, 0, 0));
    updateScene();
  }

  void setCamToRightSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 0, 1));
    _camera->setEye(Vec3f(0, -3, 0));
    updateScene();
  }

  void setCamToLeftSlot() {
    _camera->setCenter(Vec3f(0, 0, 0));
    _camera->setUp(Vec3f(0, 0, 1));
    _camera->setEye(Vec3f(0, 3, 0));
    updateScene();
  }

  // modeling parameters
  void emitRateChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->_emit_rate = 0.2f * ((float)(100 - value) / 100.0f);
      _emit_rate = 100 - value;
    }
  }

  void maxSpeedChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->_limit = 600.0f * ((float)value / 100.0f);
      _max_speed = value;
    }
  }

  void viscosityChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->_viscosity = 0.4f * ((float)value / 100.0f);
      _viscosity = value;
    }
  }

  void intStiffChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->_ext_stiff = 1.0f * ((float)value / 100.0f);
      _int_stiff = value;
    }
  }

  void extStiffChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->_ext_stiff = 20000.0f * ((float)value / 100.0f);
      _ext_stiff = value;
    }
  }

  void startSpeedChangedSlot(int value) {
    if (_psysId != -1) {
      _psys->start_speed = 10 * ((float)value / 100.0f);
      _start_speed = value;
    }
  }

  // zone size
  void lengthChangedSlot(double value) {
    _length = (float)value;
    if (_psysId != -1) {
      removeParticleSystem();
    }
    rebuildSimZone();
  }

  void widthChangedSlot(double value) {
    _width = (float)value;
    if (_psysId != -1) {
      removeParticleSystem();
    }
    rebuildSimZone();
  }

  void heightChangedSlot(double value) {
    _height = (float)value;
    if (_psysId != -1) {
      removeParticleSystem();
    }
    rebuildSimZone();
  }

  //
  void removeParticleSystem() {
    auto removeParticleSystemCommand = RemoveParticleSystemCommand(_psysId);
    _facade->executeCommand(removeParticleSystemCommand);
    _psysId = -1;
    _nObjects--;
  }

  void delModelSlot() {
    if (_psysId != -1) {
      removeParticleSystem();
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

    updateScene();
  }

  // transform model methods
  void moveModel(double dx, double dy, double dz) {
    auto moveModelCommand = MoveModelCommand(_model, dx, dy, dz);

    try {
      _facade->executeCommand(moveModelCommand);
      updateScene();
    } catch (const BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
    }
  }

  void rotateModel(double dx, double dy, double dz) {
    auto rotateModelComand = RotateModelCommand(_model, dx * 0.01, dy * 0.01, dz * 0.01);

    try {
      _facade->executeCommand(rotateModelComand);
      updateScene();
    } catch (const BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
    }
  }

  // transform camera methods
  void rotateCamera(double dx, double dy, double dz) {
    auto rotateCameraComand = RotateCameraCommand(_camera, dx * 0.01, dy * 0.01, dz * 0.01);

    try {
      _facade->executeCommand(rotateCameraComand);
      updateScene();
    } catch (const BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
    }
  }

  // mouse and keyboard handle
  void mousePressSlot(QMouseEvent* event) {
    //    std::cout << "[DBG]: MainWorker::mousePressSlot(): x = " << event->x() << "; y = " << event->y() << std::endl;

    _prevMousePos = event->pos();

    if (event->button() == Qt::LeftButton)
      _mouseLeftButtonPressed = true;

    if (event->button() == Qt::RightButton)
      _mouseRightButtonPressed = true;

    event->accept();
  }

  void mouseReleaseSlot(QMouseEvent* event) {
    //    std::cout << "[DBG]: MainWorker::mouseReleaseSlot(): x = " << event->x() << "; y = " << event->y() <<
    //    std::endl;

    _mouseLeftButtonPressed = false;
    _mouseRightButtonPressed = false;

    event->accept();
  }

  void mouseMoveSlot(QMouseEvent* event) {
    float dx = event->x() - _prevMousePos.x();
    float dy = event->y() - _prevMousePos.y();

    if (fabs(dx) > fabs(dy))
      dy = 0;
    else
      dx = 0;

    //    std::cout << "[DBG]: MainWorker::mousePressSlot(): dx = " << dx << "; dy = " << dy;

    _prevMousePos = event->pos();

    if (_mouseLeftButtonPressed) {
      if (_mouseObject == CAMERA) {
        // TODO: Move camera
      } else if (_mouseObject == MODEL) {
        if (_psysId != -1) {
          removeParticleSystem();
        }
        moveModel(dx * 0.001, dy * 0.001, 0);
      } else {
      }
    } else if (_mouseRightButtonPressed) {
      if (_mouseObject == CAMERA) {
        rotateCamera(dy, dx, 0);
      } else if (_mouseObject == MODEL) {
        if (_psysId != -1) {
          removeParticleSystem();
        }
        rotateModel(-dy, -dx, 0);
      } else {
      }
    }

    event->accept();
  }

  void wheelSlot(QWheelEvent* event) {
    //    std::cout << "[DBG]: MainWorker::wheelSlot(): x = " << event->angleDelta().x()
    //              << "; y = " << event->angleDelta().y() << std::endl;

    float factor = 1 + event->angleDelta().y() / 120 * 0.05;

    if (_mouseObject == CAMERA) {
      // TODO: zoom camera
    } else if (_mouseObject == MODEL) {
      if (_psysId != -1) {
        removeParticleSystem();
      }
      scaleModel(factor, factor, factor);
    } else {
    }

    event->accept();
  }

  void keyPressSlot(QKeyEvent* event) {
    event->accept();
  }

  void keyReleaseSlot(QKeyEvent* event) {
    event->accept();
  }

  void setModelMouseObjectSlot() {
    _mouseObject = MODEL;
  }

  void setCameraMouseObjectSlot() {
    _mouseObject = CAMERA;
  }

  // model view radio buttons
  void setWireframeViewSlot() {
    //    std::cout << "[DBG]: setWireframeViewSlot()\n";
    _viewMode = WIREFRAME;
    updateScene();
  }

  void setSimpleViewSlot() {
    //    std::cout << "[DBG]: setSimpleViewSlot()\n";
    _viewMode = SIMPLE;
    updateScene();
  }

  void setGourandViewSlot() {
    //    std::cout << "[DBG]: setGourandViewSlot()\n";
    _viewMode = GOURAND;
    updateScene();
  }

  // zone view
  void setZoneViewSlot(bool value) {
    _isZoneVisible = value;
    rebuildSimZone();
    updateScene();
  }

  // n particles
  void nParticlesChangedSlot(int value) {
    std::cout << "[DBG]: nParticlesChangedSlot(): value = " << value << std::endl;
    _nParticles = value;
    if (_psysId != -1) {
      removeParticleSystem();
    }
    updateScene();
  }

 signals:
  void sendFpsSignal(int, size_t);
  void simRunned();
  void simStopped();

 private:
  // objects
  std::shared_ptr<Facade> _facade = std::make_shared<Facade>();
  std::shared_ptr<Scene> _scene;
  std::shared_ptr<BaseDrawer> _drawer;
  shared_ptr<BaseObject> _model;
  shared_ptr<Camera> _camera;
  std::shared_ptr<ParticleSystem> _psys;

  // id
  size_t _nObjects = 0;
  int _cameraId = -1;
  int _modelId = -1;
  int _zoneId = -1;
  int _psysId = -1;

  QGraphicsScene* _qScene;

  // zone
  float _xMax, _yMax, _zMax, _xMin, _yMin, _zMin;
  Bound3D model_bound;

  bool _simRunned = false;

  // modeling parameters
  int _viscosity = 50;
  int _max_speed = 50;
  int _emit_rate = 50;
  int _start_speed = 50;
  int _int_stiff = 50;
  int _ext_stiff = 50;

  // mouse
  QPoint _prevMousePos;
  bool _mouseRightButtonPressed = false;
  bool _mouseLeftButtonPressed = false;

  // enum
  MouseObject _mouseObject = CAMERA;
  DrawMode _viewMode = SIMPLE;

  // zone
  float _length = 3.0;
  float _width = 0.8;
  float _height = 2.0;
  bool _isZoneVisible = true;

  bool _finishApp = false;

  int _nParticles = 2000;

 private:
  // workers
  GuiWorker _guiWorker;
  SimWorker _simWorker;
  QThread qThread;
  //  DrawWorker _drawWorker;
  //  NetWorker _netWorker;
  //  LoadWorker _loadWorker;

  int _FPS = 0;
  std::chrono::time_point<std::chrono::steady_clock> _prev_time;

  // app
  QApplication& _app;
};
