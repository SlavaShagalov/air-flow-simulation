#include <QApplication>
#include <QThread>

#include "App/Workers/main_worker.h"

int main(int argc, char* argv[]) {
  setbuf(stdout, nullptr);
  qputenv("QT_ASSUME_STDERR_HAS_CONSOLE", "1");
  srand(time(nullptr));

  QApplication app(argc, argv);

  MainWorker mainWorker;
//  QThread qThread;
//  mainWorker.moveToThread(&qThread);
//  qThread.start();

  QApplication::exec();

//  qThread.quit();
//  qThread.wait();

  return 0;
}
