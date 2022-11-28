#include "qt_drawer.h"
#include "qt_drawer_factory.h"

QtDrawerFactory::QtDrawerFactory(std::shared_ptr<QGraphicsScene>& scene)
    : _scene(scene)
{
}

std::unique_ptr<BaseDrawer> QtDrawerFactory::createDrawer()
{
  return std::make_unique<QtDrawer>(_scene);
}
