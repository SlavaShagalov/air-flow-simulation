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
#include "global_types.h"
#include "particle.h"

#define MAX_NEIGHBOR 80

class ParticleSystem : public VisibleObject {
 public:
  ParticleSystem(std::shared_ptr<BaseObject> model, Bound3D mb);

  ~ParticleSystem() {
  }

  // control methods
  void initialize(size_t nmax);

  void createExample(float xMin, float xMax, float yMin, float yMax, float zMin,
                     float zMax);

  void run();

  void advance();

  void emitParticles();

  void addVolume(Vec3f min, Vec3f max, float spacing);

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

  float _clr_mode;
  float _sim_scale;
  float _viscosity;
  float _rest_density;
  float _p_mass;
  float _p_radius;
  float _p_dist;
  float _smooth_radius;
  float _ext_damp;
  float _limit;
  float _int_stiff;
  float _ext_stiff;
  float _emit_rate;
  float start_speed = 5;
  size_t _maxp;

 public:
  double _R2{}, _Poly6Kern{}, _LapKern{}, _SpikyKern{};  // Kernel functions

  std::shared_ptr<BaseObject> _obstacle;
  Bound3D _mb;

  // particles
  std::vector<Particle> _particles;
  size_t _count_deleted = 0;
  double _last_emit_time = 0;

  // SPH
  double _dt{};
  double _time{};

  Vec3f _init_min;
  Vec3f _init_max;
  Vec3f _vol_min;
  Vec3f _vol_max;

  // Grid
  std::vector<int> _grid;
//  std::vector<int> _gridCnt;
  int _gridTotal{};
  Vec3f _gridMin;  // volume of grid (may not match domain volume exactly)
  Vec3f _gridMax;
  Vec3f _gridRes;   // resolution in each axis
  Vec3f _gridSize;  // physical size in each axis
  Vec3f _gridDelta;
  float _gridCellSize{};
  int _gridCell[8]{};

  // Neighbor Table
  unsigned short _neighborCount[65536]{};
  unsigned short _neighbor[65536][MAX_NEIGHBOR]{};
  float _neighborDist[65536][MAX_NEIGHBOR]{};

 private:
  friend class DrawVisitor;

  friend class TransformVisitor;
};

#endif  // PARTICLE_SYSTEM_H
