#ifndef PARTICLE_SYSTEM_H
#define PARTICLE_SYSTEM_H

#include <App/Applied/SceneObjects/Model/polygonal_model.h>
#include <App/Applied/SceneObjects/camera.h>
#include <App/Applied/Visitors/base_draw_visitor.h>
#include <App/Applied/Visitors/base_transform_visitor.h>
#include <App/Applied/constants.h>

#include <App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>
#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <vector>

#include "App/Applied/Drawer/drawer.h"
#include "particle.h"

// Scalar params
#define SIM_SIZE 4
#define SIM_SCALE 5
#define VISCOSITY 6
#define REST_DENSITY 7
#define P_MASS 8
#define P_RADIUS 9
#define P_DIST 10
#define SMOOTH_RADIUS 11
#define INT_STIFF 12
#define EXT_STIFF 13
#define EXT_DAMP 14
#define LIMIT 15
#define BOUND_Z_MIN_SLOPE 16
#define FORCE_X_MAX_SIN 17
#define FORCE_X_MIN_SIN 18
#define MAX_FRAC 19
#define CLR_MODE 20

// Vector params
#define VOL_MIN 7
#define VOL_MAX 8
#define INIT_MIN 9
#define INIT_MAX 10

#define MAX_NEIGHBOR 80
#define MAX_PARAM 21

// Scalar params
#define POINT_GRAV 2
#define PLANE_GRAV 3

// Vector params
#define EMIT_POS 0
#define EMIT_ANG 1
#define EMIT_DANG 2
#define EMIT_SPREAD 3
#define EMIT_RATE 4
#define POINT_GRAV_POS 5
#define PLANE_GRAV_DIR 6

#define ELEM_MAX 2147483640

class ParticleSystem : public VisibleObject {
 public:
  ParticleSystem(std::shared_ptr<BaseObject> model);

  ~ParticleSystem() {
  }

  // control methods
  void initialize(int nmax);

  void createExample(float xMin, float xMax, float yMin, float yMax, float zMin,
                     float zMax);

  void run();

  void advance();

  void emitParticles();

  // getters
  virtual Vec3f center() const override {
    return _center;
  }

  virtual Vec3f &center() override {
    return _center;
  }

  void computeKernels();

  void computePressureGrid();  // O(kn) - spatial grid
  void computeForceGridNC();   // O(cn) - neighbor table

  // grid
  void gridSetup(Vec3f min, Vec3f max, float sim_scale, float cell_size,
                 float border);

  void gridInsertParticles();

  void gridFindCells(Vec3f p, float radius);

  // vizitor methods
  virtual void accept(BaseDrawVisitor &visitor) override {
    visitor.visit(*this);
  }

  virtual void accept(BaseTransformVisitor &visitor) override {
  }

 private:
  double _R2{}, _Poly6Kern{}, _LapKern{},
      _SpikyKern{};  // Kernel functions

  std::shared_ptr<BaseObject> _obstacle;

  // particles
  std::vector<Particle> _particles;
  int _maxParticles{};

  // SPH
  double _dt{};
  double _time{};

  // Parameters
  double _param[MAX_PARAM]{};
  Vec3f _vec[MAX_PARAM];

  // Grid
  std::vector<int> _grid;
  std::vector<int> _gridCnt;
  int _gridTotal{};
  Vec3f _gridMin;  // volume of grid (may not match domain volume exactly)
  Vec3f _gridMax;
  Vec3f _gridRes;   // resolution in each axis
  Vec3f _gridSize;  // physical size in each axis
  Vec3f _gridDelta;
  float _gridCellSize{};
  int _gridCell[8]{};

  // Neighbor Table
  unsigned short _neighborTable[65536]{};
  unsigned short _neighbor[65536][MAX_NEIGHBOR]{};
  float _neighborDist[65536][MAX_NEIGHBOR]{};

 private:
  friend class DrawVisitor;

  friend class TransformVisitor;
};

#endif  // PARTICLE_SYSTEM_H
