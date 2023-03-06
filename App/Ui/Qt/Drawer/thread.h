#pragma once

#include <QThread>
#include <iostream>

#include "particle_system.h"
#include "mainwindow.h"

//class MainWindow;


class Thread : public QThread {
 public:
  explicit Thread(QString threadName, MainWindow* mw, bool* sr)
      : name(threadName), simRunned(sr), m(mw) {
  }

  void run() {
    std::cout << "run()\n";
    for (;;) {
      if (*simRunned) {
        //        _psys->run();
        //        m->_ui->cur_n_particles->setText(
        //            QString::number(_psys->_particles.size()));
        m->_updateScene();
      }
    }
  }

  void setPS(std::shared_ptr<ParticleSystem> ps) {
    psys = ps;
  }

 private:
  QString name;  // Имя потока
  bool* simRunned;
  std::shared_ptr<ParticleSystem> psys;
  MainWindow* m;
};
