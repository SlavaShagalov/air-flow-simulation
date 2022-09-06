#ifndef PARTICLE_H
#define PARTICLE_H

#include <Domains/Applied/Primitives/Vector3D/vector_3d.hpp>

#include <Domains/Applied/SPH/common/common_defs.h>

struct Particle {
public:
  Vec3f pos;
  DWORD clr;
  int next;
  Vec3f vel;
  Vec3f vel_eval;
  unsigned short age;

  float pressure;
  float density;
  Vec3f force;
};

#endif  // PARTICLE_H
