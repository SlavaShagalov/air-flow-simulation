#ifndef TRANSFORM_MANAGER_H
#define TRANSFORM_MANAGER_H

#include "base_manager.h"

#include <App/Applied/SceneObjects/base_object.h>

#include <App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>

class TransformManager : public BaseManager {
public:
  TransformManager() = default;

  TransformManager(const TransformManager&) = delete;
  TransformManager(TransformManager&&) = delete;

  TransformManager& operator=(const TransformManager&) = delete;
  TransformManager& operator=(TransformManager&&) = delete;

  ~TransformManager() override = default;

  void moveObject(const std::shared_ptr<BaseObject>& object, const double& dx,
                  const double& dy, const double& dz);
  void scaleObject(const std::shared_ptr<BaseObject>& object, const double& kx,
                   const double& ky, const double& kz);
  void rotateObject(const std::shared_ptr<BaseObject>& object, const double& ax,
                    const double& ay, const double& az);
  void transformObject(const std::shared_ptr<BaseObject>& object,
                       const Mtr4x4f& mtr);
};
#endif  // TRANSFORM_MANAGER_H
