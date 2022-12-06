#pragma once

#include <QObject>

#include "facade.h"
#include "mainwindow.h"
#include "particle_system.h"

class GuiWorker : public QObject {
  Q_OBJECT
 public:
  explicit GuiWorker(QObject *parent = nullptr) : QObject(parent) {
    connectMainWindow();
    _mainWindow.showMaximized();
  }
  GuiWorker(const GuiWorker &) = delete;
  GuiWorker(GuiWorker &&) = delete;
  GuiWorker &operator=(const GuiWorker &) = delete;
  GuiWorker &operator=(GuiWorker &&) = delete;

  ~GuiWorker() override {
    std::cout << "[DBG]: ~GuiWorker()\n";
    _mainWindow.close();
  }

  void connectMainWindow() {
    // retransmit filename from MainWindow to MainWorker
    connect(&_mainWindow, &MainWindow::loadModelSignal, this, &GuiWorker::loadModelSignal);
    connect(&_mainWindow, &MainWindow::sendQSceneSignal, this, &GuiWorker::sendQSceneSignal);
    connect(&_mainWindow, &MainWindow::runSimulationSignal, this, &GuiWorker::runSimulationSignal);

    // set camera
    connect(&_mainWindow, &MainWindow::setCamToTopSignal, this, &GuiWorker::setCamToTopSignal);
    connect(&_mainWindow, &MainWindow::setCamToBottomSignal, this, &GuiWorker::setCamToBottomSignal);
    connect(&_mainWindow, &MainWindow::setCamToFrontSignal, this, &GuiWorker::setCamToFrontSignal);
    connect(&_mainWindow, &MainWindow::setCamToBackSignal, this, &GuiWorker::setCamToBackSignal);
    connect(&_mainWindow, &MainWindow::setCamToLeftSignal, this, &GuiWorker::setCamToLeftSignal);
    connect(&_mainWindow, &MainWindow::setCamToRightSignal, this, &GuiWorker::setCamToRightSignal);

    // parameters changed
    connect(&_mainWindow, &MainWindow::emitRateChangedSignal, this, &GuiWorker::emitRateChangedSignal);
    connect(&_mainWindow, &MainWindow::maxSpeedChangedSignal, this, &GuiWorker::maxSpeedChangedSignal);
    connect(&_mainWindow, &MainWindow::viscosityChangedSignal, this, &GuiWorker::viscosityChangedSignal);
    connect(&_mainWindow, &MainWindow::intStiffChangedSignal, this, &GuiWorker::intStiffChangedSignal);
    connect(&_mainWindow, &MainWindow::extStiffChangedSignal, this, &GuiWorker::extStiffChangedSignal);
    connect(&_mainWindow, &MainWindow::startSpeedChangedSignal, this, &GuiWorker::startSpeedChangedSignal);

    // model
    connect(&_mainWindow, &MainWindow::delModelSignal, this, &GuiWorker::delModelSignal);

    // devices
    connect(&_mainWindow, &MainWindow::mousePressSignal, this, &GuiWorker::mousePressSignal);
    connect(&_mainWindow, &MainWindow::mouseReleaseSignal, this, &GuiWorker::mouseReleaseSignal);
    connect(&_mainWindow, &MainWindow::mouseMoveSignal, this, &GuiWorker::mouseMoveSignal);
    connect(&_mainWindow, &MainWindow::wheelSignal, this, &GuiWorker::wheelSignal);
    connect(&_mainWindow, &MainWindow::keyPressSignal, this, &GuiWorker::keyPressSignal);
    connect(&_mainWindow, &MainWindow::keyReleaseSignal, this, &GuiWorker::keyReleaseSignal);

    //
    connect(&_mainWindow, &MainWindow::setModelMouseObjectSignal, this, &GuiWorker::setModelMouseObjectSignal);
    connect(&_mainWindow, &MainWindow::setCameraMouseObjectSignal, this, &GuiWorker::setCameraMouseObjectSignal);
  }

 signals:
  void loadModelSignal(const QString &);
  void sendQSceneSignal(QGraphicsScene *);
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

  // model
  void delModelSignal();

  // devices
  void mousePressSignal(QMouseEvent *);
  void mouseReleaseSignal(QMouseEvent *);
  void mouseMoveSignal(QMouseEvent *);
  void wheelSignal(QWheelEvent *);
  void keyPressSignal(QKeyEvent *);
  void keyReleaseSignal(QKeyEvent *);

  //
  void setModelMouseObjectSignal();
  void setCameraMouseObjectSignal();

 private:
  MainWindow _mainWindow;
};
