#include <conio.h>
#include <Domains/Applied/SPH/common/common_defs.h>
#include "particle_system.h"

#include <QDebug>

#include <Domains/Applied/SceneObjects/camera.h>

#define EPSILON 0.00001f  // for collision detection
#define RTOD double(180.0) / M_PI

int ParticleSystem::_pCurr = -1;

ParticleSystem::ParticleSystem(std::shared_ptr<BaseObject> model)
    : _obstacle(model) {
  _gridRes.set(0, 0, 0);
  _pCurr = -1;
  myReset();
}

void ParticleSystem::initialize(int nParticles) {
  freeParticles();
  allocParticles(nParticles);

  setup();
  reset(nParticles);
}

void ParticleSystem::reset(int nmax) {
  reallocParticles(nmax);

  _dt = 0.003;

  // Reset parameters
  _param[MAX_FRAC] = 1.0;
  _param[POINT_GRAV] = 0.0;
  _param[PLANE_GRAV] = 1.0;

  _param[BOUND_Z_MIN_SLOPE] = 0.0;
  _param[FORCE_X_MAX_SIN] = 0.0;
  _param[FORCE_X_MIN_SIN] = 0.0;
  _param[INT_STIFF] = 1.00;
  _param[VISCOSITY] = 0.2;
  _param[INT_STIFF] = 0.50;
  _param[EXT_STIFF] = 20000;
  _param[SMOOTH_RADIUS] = 0.01;

  _vec[POINT_GRAV_POS].set(0, 0, 50);
  _vec[PLANE_GRAV_DIR].set(0, 0, -9.8);
  _vec[EMIT_POS].set(0, 0, 0);
  _vec[EMIT_RATE].set(0, 0, 0);
  _vec[EMIT_ANG].set(0, 90, 1.0);
  _vec[EMIT_DANG].set(0, 0, 0);
}

int ParticleSystem::addPointReuse() {
  int index;
  Particle* f;
  if (_nParticles < _maxParticles - 2)
    f = addParticle(index);
  else
    f = randomParticle(index);

  f->force.set(0, 0, 0);
  f->vel.set(0, 0, 0);
  f->vel_eval.set(0, 0, 0);
  f->next = 0x0;
  f->pressure = 0;
  f->density = 0;
  return index;
}

void ParticleSystem::run() {
  float ss = _param[P_DIST] / _param[SIM_SCALE];  // simulation scale

  //  if (m_Vec[EMIT_RATE].x() > 0 &&
  //      (++m_Frame) % (int)m_Vec[EMIT_RATE].x() == 0) {
  //    // m_Frame = 0;

  //    m_Vec[EMIT_POS].set(m_Vec[VOL_MAX].x(), m_Vec[VOL_MIN].y() + rand() %
  //    20,
  //                        m_Vec[VOL_MIN].z() + rand() % 20);  // Added

  //    Emit(ss);
  //  }

  gridInsertParticles();

  computePressureGrid();

  computeForceGridNC();

  advance();
}

void ParticleSystem::advance() {
  const auto model = std::dynamic_pointer_cast<PolygonalModel>(_obstacle);
  const auto& faces = model->components()->faces();
  const auto& vertices = model->components()->vertices();
  Vec3f v1, v2, v3;
  Vec3f normal, p1, p2, pi;
  float A, B, C, D, t, numerator, denominator;
  Vec3f pc1, pc2, pc3;
  float a1, a2, a3, dist;

  Particle *pbeg, *pend, *p;
  Vec3f norm;
  Vec3f accel;
  Vec3f vnext;
  Vec3f min, max;
  double adj;
  float SL, SL2, ss, radius;
  float stiff, damp, speed, diff;
  SL = _param[LIMIT];
  SL2 = SL * SL;

  stiff = _param[EXT_STIFF];
  damp = _param[EXT_DAMP];
  radius = _param[P_RADIUS];
  min = _vec[VOL_MIN];
  max = _vec[VOL_MAX];
  ss = _param[SIM_SCALE];

  Vec3i d = Vec3i((max - min).x(), (max - min).y(), (max - min).z());

  pend = _particles + _nParticles;
  for (pbeg = _particles; pbeg < pend; pbeg++) {
    p = pbeg;

    // Compute Acceleration
    accel = p->force;
    accel *= _param[P_MASS];  // a = F / m?

    // Velocity limiting
    speed =
    accel.x() * accel.x() + accel.y() * accel.y() + accel.z() * accel.z();
    if (speed > SL2) {
      accel *= SL / sqrt(speed);
    }

    // Obstacle collision
    p1 = p->pos, p2 = (p->pos + p->vel_eval);
    for (const auto& face : faces) {
      v1 = vertices[face.vertex(0)];
      v2 = vertices[face.vertex(1)];
      v3 = vertices[face.vertex(2)];
      v1 *= 30;
      v2 *= 30;
      v3 *= 30;
      //     std::cout << p1 << " " << p2 << std::endl;

      normal = Vec3f::crossProduct(v2 - v1, v3 - v1);
      normal.normalize();

      A = normal.x();
      B = normal.y();
      C = normal.z();
      D = -(A * v1.x() + B * v1.y() + C * v1.z());

      numerator = A * p1.x() + B * p1.y() + C * p1.z() + D;
      denominator =
      A * (p1.x() - p2.x()) + B * (p1.y() - p2.y()) + C * (p1.z() - p2.z());

      if (fabs(denominator) < EPSILON)
        continue;

      t = numerator / denominator;

      if (t < 0)
        continue;

      pi = p1 + (p2 - p1) * t;

      pc1 = (v1 - pi).normalize();
      pc2 = (v2 - pi).normalize();
      pc3 = (v3 - pi).normalize();

      a1 = acos(Vec3f::dotProduct(pc1, pc2));
      a2 = acos(Vec3f::dotProduct(pc2, pc3));
      a3 = acos(Vec3f::dotProduct(pc3, pc1));

      if (fabs((a1 + a2 + a3) * RTOD - 360) > 1e-3)
        continue;

      // find dist
      dist = (pi - p1).length();

      diff = 2 * radius - dist * ss;
      if (diff < 2 * radius && diff > 1e-3) {
        adj = 2 * stiff * diff - damp * Vec3f::dotProduct(normal, p->vel_eval);
        accel.x() += adj * normal.x();
        accel.y() += adj * normal.y();
        accel.z() += adj * normal.z();
      }
    }

    // Z-axis walls
    diff = 2 * radius -
           (p->pos.z() - min.z() -
            (p->pos.x() - _vec[VOL_MIN].x()) * _param[BOUND_Z_MIN_SLOPE]) *
           ss;
    if (diff > EPSILON) {
      norm.set(-_param[BOUND_Z_MIN_SLOPE], 0, 1.0 - _param[BOUND_Z_MIN_SLOPE]);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    diff = 2 * radius - (max.z() - p->pos.z()) * ss;
    if (diff > EPSILON) {
      norm.set(0, 0, -1);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // X-axis walls
    diff = 2 * radius - (p->pos.x() - min.x()) * ss;
    if (diff > EPSILON) {
      // respawn particle

      p->pos = Vec3f(max.x() - 6 * radius, min.y() + rand() % d.y(),
                     min.z() + rand() % d.z());
      p->vel = Vec3f(-5, 0, 0);
      p->vel_eval = Vec3f(-5, 0, 0);
      p->age = 0;
      continue;
    }

    diff = 2 * radius - (max.x() - p->pos.x()) * ss;
    if (diff > EPSILON) {
      norm.set(-1, 0, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // Y-axis walls
    diff = 2 * radius - (p->pos.y() - min.y()) * ss;
    //    diff = 2 * radius - (p->pos.y() + 7) * ss;
    if (diff > EPSILON) {
      norm.set(0, 1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }
    diff = 2 * radius - (max.y() - p->pos.y()) * ss;
    //    diff = 2 * radius - (7 - p->pos.y()) * ss;
    if (diff > EPSILON) {
      norm.set(0, -1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // Leapfrog Integration ----------------------------
    vnext = accel;
    vnext *= _dt;
    vnext += p->vel;  // v(t+1/2) = v(t-1/2) + a(t) dt
    p->vel_eval = p->vel;
    p->vel_eval += vnext;
    p->vel_eval *=
    0.5;  // v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5		used to compute forces later
    p->vel = vnext;
    vnext *= _dt / ss;
    p->pos += vnext;  // p(t+1) = p(t) + v(t+1/2) dt

    if (_param[CLR_MODE] == 1.0) {
      adj = fabs(vnext.x()) + fabs(vnext.y()) + fabs(vnext.z()) / 7000.0;
      adj = (adj > 1.0) ? 1.0 : adj;
      p->clr = COLORA(0, adj, adj, 1);
    } else if (_param[CLR_MODE] == 2.0) {
      float v = 0.5 + (p->pressure / 1500.0);
      if (v < 0.1)
        v = 0.1;
      if (v > 1.0)
        v = 1.0;
      p->clr = COLORA(v, 1 - v, 0, 1);
    }
  }

  _time += _dt;
}

void ParticleSystem::setup() {
  _param[SIM_SCALE] = 0.004;    // unit size
  _param[VISCOSITY] = 1.81e-5;  // pascal-second (Pa.s) = 1 kg m^-1 s^-1  (see
                                // wikipedia page on viscosity) 1.81 × 10-5
  //  _param[VISCOSITY] = 0.2;       // pascal-second (Pa.s) = 1 kg m^-1 s^-1
  //  (see
  //                                 // wikipedia page on viscosity)
  _param[REST_DENSITY] = 600.0;  // kg / m^3
  _param[P_MASS] = 0.00020543;   // kg
  _param[P_RADIUS] = 0.004;      // m
  _param[P_DIST] = 0.0059;       // m
  _param[SMOOTH_RADIUS] = 0.01;  // m
  _param[INT_STIFF] = 1.00;
  _param[EXT_STIFF] = 10000.0;
  _param[EXT_DAMP] = 256.0;
  _param[LIMIT] = 200.0;  // m / s

  computeKernels();
}

void ParticleSystem::computeKernels() {
  _param[P_DIST] = pow(_param[P_MASS] / _param[REST_DENSITY], 1 / 3.0);
  _R2 = _param[SMOOTH_RADIUS] * _param[SMOOTH_RADIUS];
  _Poly6Kern =
  315.0f / (64.0f * 3.141592 *
            pow(_param[SMOOTH_RADIUS], 9));  // Wpoly6 kernel (denominator
                                             // part) - 2003 Muller, p.4
  _SpikyKern =
  -45.0f / (3.141592 * pow(_param[SMOOTH_RADIUS],
                           6));  // Laplacian of viscocity (denominator): PI h^6
  _LapKern = 45.0f / (3.141592 * pow(_param[SMOOTH_RADIUS], 6));
}

void ParticleSystem::createExample(int n, int nmax, float xMin, float xMax,
                                   float yMin, float yMax, float zMin,
                                   float zMax) {
  reset(nmax);

  switch (n) {
    case 0:  // Wave pool
      _vec[VOL_MIN].set(-30, -30, 0);
      _vec[VOL_MAX].set(30, 30, 40);

      _vec[INIT_MIN].set(-20, -26, 10);
      _vec[INIT_MAX].set(20, 26, 40);

      _param[FORCE_X_MIN_SIN] = 12.0;
      _param[BOUND_Z_MIN_SLOPE] = 0.05;
      break;
    case 11:  // My 1
      _vec[VOL_MIN].set(-30, -30, 0);
      _vec[VOL_MAX].set(30, 30, 40);
      _vec[INIT_MIN].set(-25, -25, 20);
      _vec[INIT_MAX].set(25, 25, 40);

      _param[FORCE_X_MIN_SIN] = 12.0;  // Wall XMin Forse
                                       //      m_Param[FORCE_XMAX_SIN] = 6.0;

      _param[BOUND_Z_MIN_SLOPE] = 0.0;  // default 0.05

      _vec[PLANE_GRAV_DIR].set(0.0, 0.0, -9.8);  // g vector
      //      m_Vec[POINT_GRAV_POS].set(0, 0, 25); // ?
      //      m_Param[POINT_GRAV] = 3.5;// ?
      //      m_Param[SPH_VISC] = 0.1;                    // default 0.1
      //      m_Param[SPH_INTSTIFF] = 0.50;  // default 0.50
      //      m_Param[SPH_EXTSTIFF] = 5000;  // default 8000

      // emit
      //      m_Vec[EMIT_POS].set(-20, -20, 22);
      //      m_Vec[EMIT_RATE].set(1, 4, 0);
      //      m_Vec[EMIT_ANG].set(0, 120, 1.5);
      //      m_Vec[EMIT_DANG].set(0, 0, 0);

      break;
    case 12:
      _param[CLR_MODE] = 2.0;
      //      _param[CLR_MODE] = 1.0;

      _vec[VOL_MIN].set(xMin, yMin, zMin);
      _vec[VOL_MAX].set(xMax, yMax, zMax);

      _vec[INIT_MIN].set(xMin, yMin, zMin);
      _vec[INIT_MAX].set(xMax, yMax, zMax);

      _vec[PLANE_GRAV_DIR].set(0.0, 0.0, 0.0);

      _vec[EMIT_POS].set(xMax, yMin + 10, zMin + 10);
      _vec[EMIT_RATE].set(1, 4, 0);  // default 1, 4, 0
      _vec[EMIT_ANG].set(0, 0, 0);

      //      m_Param[FORCE_XMIN_SIN] = 12.0;
      //      m_Param[BOUND_ZMIN_SLOPE] = 0.05;
      break;
  }

  computeKernels();

  _param[SIM_SIZE] =
  _param[SIM_SCALE] * (_vec[VOL_MAX].z() - _vec[VOL_MIN].z());
  _param[P_DIST] = pow(_param[P_MASS] / _param[REST_DENSITY], 1 / 3.0);

  float ss = _param[P_DIST] * 0.87 / _param[SIM_SCALE];
  //  std::cout << "Spacing: %f " << ss << std::endl;

  addVolume(_vec[INIT_MIN], _vec[INIT_MAX],
            ss);  // Create the particles

  float cell_size = _param[SMOOTH_RADIUS] * 2.0;  // Grid cell size (2r)

  //  Grid_Setup(_vec[VOL_MIN], _vec[VOL_MAX], _param[SIM_SCALE], cell_size,
  //             1.0);  // Setup grid

  gridSetup(_vec[VOL_MIN], _vec[VOL_MAX], _param[SIM_SCALE], cell_size,
            1.0);  // Setup grid

  gridInsertParticles();

  Vec3f vmin, vmax;
  vmin = _vec[VOL_MIN];
  vmin -= Vec3f(2, 2, 2);
  vmax = _vec[VOL_MAX];
  vmax += Vec3f(2, 2, -2);
}

// Compute Pressures - Using spatial grid, and also create neighbor table
void ParticleSystem::computePressureGrid() {
  Particle *pbeg, *pend, *p;
  Particle* pcurr;
  int pndx;
  int i;
  float dx, dy, dz, sum, dsq, c;
  float d, mR, mR2;
  float radius = _param[SMOOTH_RADIUS] / _param[SIM_SCALE];
  d = _param[SIM_SCALE];
  mR = _param[SMOOTH_RADIUS];
  mR2 = mR * mR;

  pend = _particles + _nParticles;
  i = 0;
  for (pbeg = _particles; pbeg < pend; pbeg++, i++) {
    p = (Particle*)pbeg;

    sum = 0.0;
    _neighborTable[i] = 0;

    //    Grid_FindCells(p->pos, radius);
    gridFindCells(p->pos, radius);
    for (int cell = 0; cell < 8; cell++) {
      if (_gridCell[cell] != -1) {
        pndx = _grid[_gridCell[cell]];
        while (pndx != -1) {
          pcurr = _particles + pndx;
          if (pcurr == p) {
            pndx = pcurr->next;
            continue;
          }
          dx = (p->pos.x() - pcurr->pos.x()) * d;  // dist in cm
          dy = (p->pos.y() - pcurr->pos.y()) * d;
          dz = (p->pos.z() - pcurr->pos.z()) * d;
          dsq = (dx * dx + dy * dy + dz * dz);
          if (mR2 > dsq) {
            c = _R2 - dsq;
            sum += c * c * c;
            if (_neighborTable[i] < MAX_NEIGHBOR) {
              _neighbor[i][_neighborTable[i]] = pndx;
              _neighborDist[i][_neighborTable[i]] = sqrt(dsq);
              _neighborTable[i]++;
            }
          }
          pndx = pcurr->next;
        }
      }
      _gridCell[cell] = -1;
    }
    p->density = sum * _param[P_MASS] * _Poly6Kern;
    p->pressure = (p->density - _param[REST_DENSITY]) * _param[INT_STIFF];
    p->density = 1.0f / p->density;
  }
}

// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
void ParticleSystem::computeForceGridNC() {
  Particle *pbeg, *pend, *p;
  Particle* pcurr;
  Vec3f force;
  float pterm, vterm, dterm;
  int i;
  float c, d;
  float dx, dy, dz;
  float mR, visc;

  d = _param[SIM_SCALE];
  mR = _param[SMOOTH_RADIUS];
  visc = _param[VISCOSITY];

  pend = _particles + _nParticles;
  i = 0;

  for (pbeg = _particles; pbeg < pend; pbeg++, i++) {
    p = (Particle*)pbeg;

    force.set(0, 0, 0);
    for (int j = 0; j < _neighborTable[i]; j++) {
      pcurr = _particles + _neighbor[i][j];
      dx = (p->pos.x() - pcurr->pos.x()) * d;  // dist in cm
      dy = (p->pos.y() - pcurr->pos.y()) * d;
      dz = (p->pos.z() - pcurr->pos.z()) * d;
      c = (mR - _neighborDist[i][j]);
      pterm = -0.5f * c * _SpikyKern * (p->pressure + pcurr->pressure) /
              _neighborDist[i][j];
      dterm = c * p->density * pcurr->density;
      vterm = _LapKern * visc;
      force.x() +=
      (pterm * dx + vterm * (pcurr->vel_eval.x() - p->vel_eval.x())) * dterm;
      force.y() +=
      (pterm * dy + vterm * (pcurr->vel_eval.y() - p->vel_eval.y())) * dterm;
      force.z() +=
      (pterm * dz + vterm * (pcurr->vel_eval.z() - p->vel_eval.z())) * dterm;
    }
    p->force = force;
  }
}

void ParticleSystem::freeParticles() {
  free(_particles);
  _particles = NULL;
  _nParticles = 0;
}

void ParticleSystem::reallocParticles(int newSize) {
  _maxParticles = newSize;

  if (_particles != NULL)
    free(_particles);

  Particle* new_data = (Particle*)malloc(_maxParticles * sizeof(Particle));

  if (!new_data)
    qDebug() << "ERROR";

  _particles = new_data;
  _nParticles = 0;
}

void ParticleSystem::allocParticles(int nParticles) {
  _particles = (Particle*)malloc(nParticles * sizeof(Particle));
  _nParticles = 0;
  _maxParticles = nParticles;
  if (!_particles)
    qDebug() << "ERROR!";
}

int ParticleSystem::addParticleReuse() {
  int index;
  if (_nParticles < _maxParticles - 1)
    addParticle(index);
  else
    randomParticle(index);

  return index;
}

Particle* ParticleSystem::addParticle(int& index) {
  if (_nParticles >= _maxParticles) {
    if (long(_maxParticles) * 2 > ELEM_MAX) {
      qDebug() << "ERROR";
    }
    _maxParticles *= 2;

    Particle* new_data = (Particle*)malloc(_maxParticles * sizeof(Particle));

    if (!new_data)
      qDebug() << "ERROR!";

    memcpy(new_data, _particles, _nParticles * sizeof(Particle));

    free(_particles);
    _particles = new_data;
  }

  _nParticles++;
  index = _nParticles - 1;

  return _particles + index;
}

Particle* ParticleSystem::randomParticle(int& index) {
  index = _nParticles * rand() / RAND_MAX;  // mb float
  return _particles + index * sizeof(Particle);
}

void ParticleSystem::addVolume(Vec3f min, Vec3f max, float spacing) {
  Vec3f pos;
  Particle* p;
  float dx, dy, dz;

  dx = max.x() - min.x();
  dy = max.y() - min.y();
  dz = max.z() - min.z();

  for (float z = max.z(); z >= min.z(); z -= spacing) {
    for (float y = min.y(); y <= max.y(); y += spacing) {
      for (float x = min.x(); x <= max.x(); x += spacing) {
        p = getParticle(addParticleReuse());
        pos.set(x, y, z);
        p->pos = pos;
        p->clr =
        COLORA((x - min.x()) / dx, (y - min.y()) / dy, (z - min.z()) / dz, 1);
      }
    }
  }
}

void ParticleSystem::gridSetup(Vec3f min, Vec3f max, float sim_scale,
                               float cell_size, float border) {
  float world_cellsize = cell_size / sim_scale;
  _grid.clear();
  _gridMin = min;
  _gridMin -= border;
  _gridMax = max;
  _gridMax += border;
  _gridSize = _gridMax;
  _gridSize -= _gridMin;
  _gridCellSize = world_cellsize;
  _gridRes.x() =
  ceil(_gridSize.x() / world_cellsize);  // Determine grid resolution
  _gridRes.y() = ceil(_gridSize.y() / world_cellsize);
  _gridRes.z() = ceil(_gridSize.z() / world_cellsize);
  _gridSize.x() = _gridRes.x() * cell_size /
                  sim_scale;  // Adjust grid size to multiple of cell size
  _gridSize.y() = _gridRes.y() * cell_size / sim_scale;
  _gridSize.z() = _gridRes.z() * cell_size / sim_scale;
  _gridDelta = _gridRes;  // delta = translate from world space to cell #
  _gridDelta /= _gridSize;
  _gridTotal = (int)(_gridSize.x() * _gridSize.y() * _gridSize.z());

  _grid.clear();
  _gridCnt.clear();

  _grid.reserve(_gridTotal);
  _gridCnt.reserve(_gridTotal);
  for (int n = 0; n < _gridTotal; n++) {
    _grid.push_back(-1);
    _gridCnt.push_back(0);
  }
}

void ParticleSystem::gridInsertParticles() {
  Particle *pbeg, *pend, *p;

  int gs;
  int gx, gy, gz;

  pend = _particles + _nParticles;

  // set next = -1
  for (pbeg = _particles; pbeg < pend; pbeg++)
    pbeg->next = -1;

  for (int n = 0; n < _gridTotal; n++) {
    _grid[n] = -1;
    _gridCnt[n] = 0;
  }

  int n = 0;
  for (pbeg = _particles; pbeg < pend; pbeg++) {
    p = pbeg;

    gx = (int)((p->pos.x() - _gridMin.x()) * _gridDelta.x());  // Determine
                                                               // grid cell
    gy = (int)((p->pos.y() - _gridMin.y()) * _gridDelta.y());
    gz = (int)((p->pos.z() - _gridMin.z()) * _gridDelta.z());

    gs = (int)((gz * _gridRes.y() + gy) * _gridRes.x() + gx);  // WHAT??
    if (gs >= 0 && gs < _gridTotal) {
      p->next = _grid[gs];
      _grid[gs] = n;
      _gridCnt[gs]++;
    }
    n++;
  }
}

void ParticleSystem::gridFindCells(Vec3f p, float radius) {
  Vec3i sph_min;

  // Compute sphere range
  sph_min.x() = (int)((-radius + p.x() - _gridMin.x()) * _gridDelta.x());
  sph_min.y() = (int)((-radius + p.y() - _gridMin.y()) * _gridDelta.y());
  sph_min.z() = (int)((-radius + p.z() - _gridMin.z()) * _gridDelta.z());
  if (sph_min.x() < 0)
    sph_min.x() = 0;
  if (sph_min.y() < 0)
    sph_min.y() = 0;
  if (sph_min.z() < 0)
    sph_min.z() = 0;

  _gridCell[0] =
  (int)((sph_min.z() * _gridRes.y() + sph_min.y()) * _gridRes.x() +
        sph_min.x());
  _gridCell[1] = _gridCell[0] + 1;
  _gridCell[2] = (int)(_gridCell[0] + _gridRes.x());
  _gridCell[3] = _gridCell[2] + 1;

  if (sph_min.z() + 1 < _gridRes.z()) {
    _gridCell[4] = (int)(_gridCell[0] + _gridRes.y() * _gridRes.x());
    _gridCell[5] = _gridCell[4] + 1;
    _gridCell[6] = (int)(_gridCell[4] + _gridRes.x());
    _gridCell[7] = _gridCell[6] + 1;
  }
  if (sph_min.x() + 1 >= _gridRes.x()) {
    _gridCell[1] = -1;
    _gridCell[3] = -1;
    _gridCell[5] = -1;
    _gridCell[7] = -1;
  }
  if (sph_min.y() + 1 >= _gridRes.y()) {
    _gridCell[2] = -1;
    _gridCell[3] = -1;
    _gridCell[6] = -1;
    _gridCell[7] = -1;
  }
}

void ParticleSystem::myReset() {
  _time = 0;
  _dt = 0.1;
  _param[POINT_GRAV] = 100.0;
  _param[PLANE_GRAV] = 0.0;

  _vec[POINT_GRAV_POS].set(0, 0, 50.0);
  _vec[PLANE_GRAV_DIR].set(0, 0, -9.8);
  _vec[EMIT_RATE].set(1, 10, 0);
  _vec[EMIT_POS].set(50, 0, 35);
  _vec[EMIT_ANG].set(90, 45, 50.0);
  _vec[EMIT_DANG].set(0, 0, 0);
  _vec[EMIT_SPREAD].set(4, 4, 1);
}
