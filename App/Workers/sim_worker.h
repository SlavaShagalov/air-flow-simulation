#pragma once

#include <QGraphicsScene>
#include <QObject>
#include <thread>

#include "base_exception.h"
#include "draw_scene_command.h"
#include "facade.h"
#include "particle_system.h"

class SimWorker : public QObject {
  Q_OBJECT
 public:
  explicit SimWorker(QObject* parent = nullptr) : QObject(parent) {
    std::cout << "SimWorker()\n";
  }

  void setFields(std::shared_ptr<Scene> scene, std::shared_ptr<BaseDrawer> drawer, shared_ptr<Camera> camera,
                 std::shared_ptr<ParticleSystem> psys, std::shared_ptr<Facade> facade, QGraphicsScene* qScene) {
    _scene = scene;
    _drawer = drawer;
    _camera = camera;
    _psys = psys;
    _facade = facade;
    _qScene = qScene;
  }

  ~SimWorker() override {
    std::cout << "[DBG]: ~SimWorker()\n";
  }

  void connectToUi() {
  }

  void updateScene() {
    // draw scene
    DrawMode mode = SIMPLE;
    //    if (_ui->wireframeRadioBtn->isChecked()) {
    //      mode = WIREFRAME;
    //    } else if (_ui->simpleRadioBtn->isChecked()) {
    //      mode = SIMPLE;
    //    } else {
    //      mode = GOURAND;
    //    }
    ParticleMode particleMode;
    //  if (_ui->circleParticleRadioBtn->isChecked()) {
    //    particleMode = SPHERE;
    //  } else {
    particleMode = VECTOR;
    //  }

    auto drawSceneCommand = DrawSceneCommand(_scene, _drawer, _camera, mode, particleMode);
    try {
      _facade->executeCommand(drawSceneCommand);
    } catch (BaseException& ex) {
      //      QMessageBox::warning(this, "Error", QString(ex.what()));
      std::cout << "[DBG]: Failed to execute DrawSceneCommand\n";
    }
  }

 signals:
  void finished();
  void runningChanged(bool running);

 public slots:
  void run() {
    std::cout << "SimWorker runned\n";
    // Переменная m_running отвечает за работу объекта в потоке.
    // При значении false работа завершается
    while (m_running) {
      // test
//      _qScene->addLine(20, 20, 40, 60, QPen(Qt::red));
//      _qScene->addLine(100, 10, 60, 80, QPen(Qt::green));
      _psys->run();
      //      _ui->cur_n_particles->setText(QString::number(_psys->_particles.size()));
      updateScene();
    }
    std::cout << "SimWorker finished\n";
    emit finished();
  }

  bool running() const {
    return m_running;
  }

  void setRunning(bool running) {
    if (m_running == running)
      return;

    m_running = running;
    emit runningChanged(running);
  }

  //  void stop() {
  //    isStopped = true;
  //  }

 private:
  // objects
  std::shared_ptr<Scene> _scene;
  QGraphicsScene* _qScene;
  std::shared_ptr<BaseDrawer> _drawer;
  shared_ptr<BaseObject> _model;
  shared_ptr<Camera> _camera;
  std::shared_ptr<ParticleSystem> _psys;
  std::shared_ptr<Facade> _facade = std::make_shared<Facade>();

  // Свойство, управляющее работой потока
  //  Q_PROPERTY(bool running READ running WRITE setRunning NOTIFY runningChanged)
  // Первое сообщение в объекте
  //  Q_PROPERTY(QString message READ message WRITE setMessage NOTIFY messageChanged)
  //  // Второе сообщение, которое будем передавать через сигнал/слот во второй объект
  //  Q_PROPERTY(QString message_2 READ message_2 WRITE setMessage_2 NOTIFY message_2Changed)

  bool m_running;
};
