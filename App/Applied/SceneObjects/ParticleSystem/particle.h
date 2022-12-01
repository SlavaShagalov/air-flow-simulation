#ifndef PARTICLE_H
#define PARTICLE_H

#include <App/Applied/SPH/common/common_defs.h>

#include <App/Applied/Primitives/Vector3D/vector_3d.hpp>

struct Particle {
 public:
  Vec3f pos;
  DWORD clr;
  int next;
  Vec3f vel;
  Vec3f vel_eval;

  float pressure;
  float density;
  Vec3f force;

  // output
  friend std::ostream &operator<<(std::ostream &os, const Particle &p) {
    os << "pos=" << p.pos << "; next=" << p.next << "; vel=" << p.vel
       << "; vel_eval=" << p.vel_eval << ";";
    return os;
  }
};

#endif  // PARTICLE_H
