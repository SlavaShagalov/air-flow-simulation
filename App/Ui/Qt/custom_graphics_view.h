#ifndef CUSTOMGRAPHICSVIEW_H
#define CUSTOMGRAPHICSVIEW_H

#include <QGraphicsView>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QWheelEvent>

class CustomGraphicsView : public QGraphicsView {
  Q_OBJECT
public:
  CustomGraphicsView(QWidget* parent = nullptr) : QGraphicsView(parent){};

  // events
  void keyPressEvent(QKeyEvent* event) override { emit keyPressSigna(event); }

  void keyReleaseEvent(QKeyEvent* event) override {
    emit keyReleaseSignal(event);
  }
  void mousePressEvent(QMouseEvent* event) override {
    emit mousePressSignal(event);
  }
  void mouseReleaseEvent(QMouseEvent* event) override {
    emit mouseReleaseSignal(event);
  }
  void mouseMoveEvent(QMouseEvent* event) override {
    emit mouseMoveSignal(event);
  }
  void wheelEvent(QWheelEvent* event) override { emit wheelSignal(event); }

signals:
  void mousePressSignal(QMouseEvent* event);
  void mouseReleaseSignal(QMouseEvent* event);
  void mouseMoveSignal(QMouseEvent* event);
  void wheelSignal(QWheelEvent* event);
  void keyPressSigna(QKeyEvent* event);
  void keyReleaseSignal(QKeyEvent* event);
};

#endif  // CUSTOMGRAPHICSVIEW_H
