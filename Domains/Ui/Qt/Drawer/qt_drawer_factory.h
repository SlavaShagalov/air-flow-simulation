#ifndef QT_DRAWER_FACTORY_H
#define QT_DRAWER_FACTORY_H

#include <memory>
#include <QGraphicsScene>

#include <Domains/Applied/Drawer/drawer.h>

class QtDrawerFactory : public DrawerFactory {
public:
  explicit QtDrawerFactory(std::shared_ptr<QGraphicsScene>& scene);

  std::unique_ptr<BaseDrawer> createDrawer() override;

private:
  std::shared_ptr<QGraphicsScene> _scene;
};

#endif  // QT_DRAWER_FACTORY_H
