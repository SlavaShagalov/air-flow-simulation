#include <QApplication>

#include <Domains/Ui/Qt/main_window.h>

int main(int argc, char* argv[]) {
  qputenv("QT_ASSUME_STDERR_HAS_CONSOLE", "1");
  srand(time(nullptr));

  QApplication app(argc, argv);

  MainWindow w(&app);
  w.showMaximized();

  return QApplication::exec();
}
