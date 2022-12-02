#ifndef GLOBAL_TYPES_H
#define GLOBAL_TYPES_H

enum DrawMode {
  WIREFRAME,
  SIMPLE,
  GOURAND,
};

enum ParticleMode {
  SPHERE,
  VECTOR,
};

struct Bound3D {
  float xMin;
  float xMax;
  float yMin;
  float yMax;
  float zMin;
  float zMax;
};

#endif  // GLOBAL_TYPES_H
