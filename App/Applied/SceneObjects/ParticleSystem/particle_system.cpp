#include <Domains/Applied/SPH/common/common_defs.h>
#include <Domains/Applied/SceneObjects/ParticleSystem/particle_system.h>

#include <QDebug>

#include <Domains/Applied/SceneObjects/camera.h>

#define EPSILON 0.00001f  // for collision detection
#define RTOD double(180.0) / M_PI

ParticleSystem::ParticleSystem(std::shared_ptr<BaseObject> model)
    : _obstacle(model), _particles(NULL), _nParticles(0) {}

void ParticleSystem::initialize(int nParticles) {
  allocParticles(nParticles);  // create place for n particles

  _gridRes.set(0, 0, 0);
  _time = 0;

  //  _dt = 0.1;
  _dt = 0.003;

  _param[CLR_MODE] = 2.0;

  //  _param[POINT_GRAV] = 100.0;
  _param[POINT_GRAV] = 0.0;

  //  _param[PLANE_GRAV] = 0.0;
  _param[PLANE_GRAV] = 1.0;

  _vec[POINT_GRAV_POS].set(0, 0, 50.0);

  _vec[PLANE_GRAV_DIR].set(0.0, 0.0, 0.0);

  // Emit param
  _vec[EMIT_SPREAD].set(4, 4, 1);

  //    _vec[EMIT_POS].set(50, 0, 35);
  _vec[EMIT_POS].set(0, 0, 0);

  //  _vec[EMIT_RATE].set(1, 10, 0);
  _vec[EMIT_RATE].set(0, 0, 0);

  //  _vec[EMIT_ANG].set(90, 45, 50.0);
  _vec[EMIT_ANG].set(0, 90, 1.0);

  _vec[EMIT_DANG].set(0, 0, 0);

  //
  _param[SIM_SCALE] = 0.004;  // unit size

    _param[VISCOSITY] = 0.008;  // custom
  //  _param[VISCOSITY] = 1.81e-5;  // pascal-second (Pa.s) = 1 kg m^-1 s^-1
  //  (see
  // wikipedia page on viscosity) 1.81 × 10-5
//  _param[VISCOSITY] = 0.2;  // pascal-second (Pa.s) = 1 kg m^-1 s^-1
  //  (see
  //                                 // wikipedia page on viscosity)

  //  _param[REST_DENSITY] = 600.0;  // kg / m^3; WATER
  //    _param[REST_DENSITY] = 1.225;  // kg / m^3 AIR
  _param[REST_DENSITY] = 600;  // CUSTOM

  _param[P_MASS] = 0.00020543;  // kg WATER

  _param[P_RADIUS] = 0.004;  // m

  _param[P_DIST] = 0.0059;  // m

  _param[SMOOTH_RADIUS] = 0.01;  // m

  _param[EXT_DAMP] = 256.0;
  //  _param[LIMIT] = 200.0;  // m / s DEFAULT
  _param[LIMIT] = 200.0;  // CUSTOM

  // from reset()
  _param[MAX_FRAC] = 1.0;  // ?

  _param[BOUND_Z_MIN_SLOPE] = 0.0;
  _param[FORCE_X_MAX_SIN] = 0.0;
  _param[FORCE_X_MIN_SIN] = 0.0;

  //  _param[INT_STIFF] = 1.00;
  _param[INT_STIFF] = 0.50;  // default 0.50
                             //  _param[INT_STIFF] = 0.30; //   custom

  //   _param[EXT_STIFF] = 10000.0;
  //  _param[EXT_STIFF] = 20000; // default 20000
  _param[EXT_STIFF] = 10000;  //  custom
}

void ParticleSystem::createExample(float xMin, float xMax, float yMin,
                                   float yMax, float zMin, float zMax) {
  _vec[VOL_MIN].set(xMin, yMin, zMin);
  _vec[VOL_MAX].set(xMax, yMax, zMax);

  _vec[INIT_MIN].set(xMin, yMin, zMin);
  _vec[INIT_MAX].set(xMax, yMax, zMax);

  // emit
  _vec[EMIT_POS].set(xMax, yMin + 10, zMin + 10);
  _vec[EMIT_RATE].set(1, 4, 0);  // default 1, 4, 0
  _vec[EMIT_ANG].set(0, 0, 0);

  computeKernels();

  _param[SIM_SIZE] =
  _param[SIM_SCALE] * (_vec[VOL_MAX].z() - _vec[VOL_MIN].z());

  _param[P_DIST] = pow(_param[P_MASS] / _param[REST_DENSITY], 1 / 3.0);

  float ss = _param[P_DIST] * 0.87 / _param[SIM_SCALE];
  //  std::cout << "Spacing: %f " << ss << std::endl;

  //  addVolume(_vec[INIT_MIN], _vec[INIT_MAX],
  //            ss);  // Create the particles

  float cell_size = _param[SMOOTH_RADIUS] * 2.0;  // Grid cell size (2r)
  gridSetup(_vec[VOL_MIN], _vec[VOL_MAX], _param[SIM_SCALE], cell_size,
            1.0);  // Setup grid

  gridInsertParticles();
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

  emitParticles();

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

  Particle *pend, *p;
  Vec3f norm, accel, vnext;
  Vec3f min, max;
  double adj;
  float ss, radius;
  float stiff, damp, speed, diff;

  stiff = _param[EXT_STIFF];  // коэффициент жесткости для столкновения со
                              // стенками и полигонами обьектов
  damp = _param[EXT_DAMP];
  radius = _param[P_RADIUS];
  min = _vec[VOL_MIN];
  max = _vec[VOL_MAX];
  ss = _param[SIM_SCALE];

  Vec3i d = Vec3i((max - min).x(), (max - min).y(), (max - min).z());

  pend = _particles + _nParticles;
  for (p = _particles; p < pend; p++) {
    // Compute Acceleration
    accel = p->force * _param[P_MASS];  // Why, a = F / m?

    // Velocity limiting
    speed =
    accel.x() * accel.x() + accel.y() * accel.y() + accel.z() * accel.z();
    if (speed > _param[LIMIT] * _param[LIMIT]) {
      accel *= _param[LIMIT] / sqrt(speed);
    }

    // Obstacle collision
    p1 = p->pos, p2 = (p->pos + p->vel_eval);
    for (const auto& face : faces) {
      v1 = vertices[face.vertex(0)];
      v2 = vertices[face.vertex(1)];
      v3 = vertices[face.vertex(2)];
      // TODO: solve scale problem
      v1 *= 30;
      v2 *= 30;
      v3 *= 30;
      //     std::cout << p1 << " " << p2 << std::endl;

      normal = Vec3f::crossProduct(v2 - v1, v3 - v1).normalize();

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
    diff = 2 * radius - (p->pos.z() - min.z()) * ss;
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
      // delete particle
      //      deleteParticle(p - _particles);

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
    if (diff > EPSILON) {
      norm.set(0, 1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }
    diff = 2 * radius - (max.y() - p->pos.y()) * ss;
    if (diff > EPSILON) {
      norm.set(0, -1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p->vel_eval);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // Leapfrog Integration
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

    // set color
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

void ParticleSystem::emitParticles() {
  Particle* p;
  Vec3f dir = { -5, 0, 0 };
  Vec3f pos;

  pos.setX(_vec[VOL_MAX].x());
  pos.setY(_vec[VOL_MIN].y() +
           rand() % (int)(_vec[VOL_MAX].y() - _vec[VOL_MIN].y()));
  pos.setZ(_vec[VOL_MIN].z() +
           rand() % (int)(_vec[VOL_MAX].z() - _vec[VOL_MIN].z()));

  int ind;
  p = addParticle(ind);
  //  p = (Particle*)GetElem(0, AddPointReuse());
  p->pos = pos;
  p->vel = dir;
  p->vel_eval = dir;
  p->age = 0;
  p->clr = COLORA(0, 0, 0, 1);
}

void ParticleSystem::computeKernels() {
  _param[P_DIST] = pow(_param[P_MASS] / _param[REST_DENSITY], 1 / 3.0);

  _R2 = _param[SMOOTH_RADIUS] * _param[SMOOTH_RADIUS];
  _Poly6Kern =
  315.0f /
  (64.0f * PI * pow(_param[SMOOTH_RADIUS], 9));  // Wpoly6 kernel (denominator
                                                 // part) - 2003 Muller, p.4
  _SpikyKern =
  -45.0f / (PI * pow(_param[SMOOTH_RADIUS],
                     6));  // Laplacian of viscocity (denominator): PI h^6
  _LapKern = 45.0f / (PI * pow(_param[SMOOTH_RADIUS], 6));
}

// Compute Pressures - Using spatial grid, and also create neighbor table
void ParticleSystem::computePressureGrid() {
  Particle *pend = _particles + _nParticles, *p, *pcurr;
  int pndx;
  float dx, dy, dz, sum, dSqr, c;
  float h = _param[SMOOTH_RADIUS], h2 = h * h;
  float simH = _param[SMOOTH_RADIUS] / _param[SIM_SCALE];

  int i = 0;
  for (p = _particles; p < pend; p++, i++) {
    sum = 0.0;
    _neighborTable[i] = 0;

    gridFindCells(p->pos, simH);

    for (int j = 0; j < 8; j++) {
      if (_gridCell[j] != -1) {
        pndx = _grid[_gridCell[j]];
        while (pndx != -1) {          // if j-th cell has 1 or more particles
          pcurr = _particles + pndx;  // get first particle in j neighbor cell
          if (pcurr == p) {
            pndx = pcurr->next;  // get next particle in this cell
            continue;            // ignore itself
          }

          // dist between particles in cm
          dx = (p->pos.x() - pcurr->pos.x()) * _param[SIM_SCALE];
          dy = (p->pos.y() - pcurr->pos.y()) * _param[SIM_SCALE];
          dz = (p->pos.z() - pcurr->pos.z()) * _param[SIM_SCALE];
          dSqr = (dx * dx + dy * dy + dz * dz);

          if (dSqr < h2) {
            c = _R2 - dSqr;  // kernel
            sum += c * c * c;
            if (_neighborTable[i] < MAX_NEIGHBOR) {
              _neighbor[i][_neighborTable[i]] = pndx;
              _neighborDist[i][_neighborTable[i]] = sqrt(dSqr);
              _neighborTable[i]++;
            }
          }

          pndx = pcurr->next;  // get next particle in this cell
        }
      }

      _gridCell[j] = -1;
    }

    p->density = sum * _param[P_MASS] * _Poly6Kern;
    p->pressure = (p->density - _param[REST_DENSITY]) * _param[INT_STIFF];
    p->density = 1.0f / p->density;
  }
}

// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
void ParticleSystem::computeForceGridNC() {
  Particle *pend = _particles + _nParticles, *p, *pcur;
  Vec3f force;
  float pterm, vterm, dterm;
  int i = 0;
  float c;
  float dx, dy, dz;

  for (p = _particles; p < pend; p++, i++) {
    force.set(0, 0, 0);

    for (int j = 0; j < _neighborTable[i]; j++) {
      pcur = _particles + _neighbor[i][j];

      // dist in cm
      dx = (p->pos.x() - pcur->pos.x()) * _param[SIM_SCALE];
      dy = (p->pos.y() - pcur->pos.y()) * _param[SIM_SCALE];
      dz = (p->pos.z() - pcur->pos.z()) * _param[SIM_SCALE];

      c = (_param[SMOOTH_RADIUS] - _neighborDist[i][j]);

      pterm = -0.5f * c * _SpikyKern * (p->pressure + pcur->pressure) /
              _neighborDist[i][j];

      dterm = c * p->density * pcur->density;

      vterm = _LapKern * _param[VISCOSITY];

      force.x() +=
      (pterm * dx + vterm * (pcur->vel_eval.x() - p->vel_eval.x())) * dterm;
      force.y() +=
      (pterm * dy + vterm * (pcur->vel_eval.y() - p->vel_eval.y())) * dterm;
      force.z() +=
      (pterm * dz + vterm * (pcur->vel_eval.z() - p->vel_eval.z())) * dterm;
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

void ParticleSystem::deleteParticle(const int index) {
  for (int i = index; i < _nParticles - 1; i++) {
    _particles[i] = _particles[i + 1];
  }

  _nParticles--;
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
  qDebug() << "World Cell Size = " << world_cellsize;
  _grid.clear();
  _gridMin = min;
  _gridMin -= border;
  _gridMax = max;
  _gridMax += border;
  _gridSize = _gridMax - _gridMin;
  _gridCellSize = world_cellsize;

  // Determine grid resolution
  _gridRes.x() = ceil(_gridSize.x() / _gridCellSize);
  _gridRes.y() = ceil(_gridSize.y() / _gridCellSize);
  _gridRes.z() = ceil(_gridSize.z() / _gridCellSize);

  // Adjust grid size to multiple of cell size
  _gridSize.x() = _gridRes.x() * _gridCellSize;
  _gridSize.y() = _gridRes.y() * _gridCellSize;
  _gridSize.z() = _gridRes.z() * _gridCellSize;

  // delta = translate from world space to cell #
  _gridDelta = _gridRes;
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
  Particle *pend, *p;

  int gridId;
  int xGridId, yGridId, zGridId;

  pend = _particles + _nParticles;

  // set next = -1
  for (p = _particles; p < pend; p++)
    p->next = -1;

  for (int n = 0; n < _gridTotal; n++) {
    _grid[n] = -1;
    _gridCnt[n] = 0;
  }

  int n = 0;
  for (p = _particles; p < pend; p++) {
    // Determine grid cell id in axis
    xGridId = (int)((p->pos.x() - _gridMin.x()) * _gridDelta.x());
    yGridId = (int)((p->pos.y() - _gridMin.y()) * _gridDelta.y());
    zGridId = (int)((p->pos.z() - _gridMin.z()) * _gridDelta.z());

    // Determine grid cell id in _grid array
    gridId = (int)((zGridId * _gridRes.y() + yGridId) * _gridRes.x() + xGridId);

    if (gridId >= 0 && gridId < _gridTotal) {
      p->next = _grid[gridId];  // TODO: check if it is need
      _grid[gridId] = n;
      _gridCnt[gridId]++;
    }
    n++;
  }
}

void ParticleSystem::gridFindCells(Vec3f pos, float radius) {
  Vec3i sphereMin;

  // Compute sphere range
  sphereMin.x() = (int)((-radius + pos.x() - _gridMin.x()) * _gridDelta.x());
  sphereMin.y() = (int)((-radius + pos.y() - _gridMin.y()) * _gridDelta.y());
  sphereMin.z() = (int)((-radius + pos.z() - _gridMin.z()) * _gridDelta.z());

  if (sphereMin.x() < 0)
    sphereMin.x() = 0;
  if (sphereMin.y() < 0)
    sphereMin.y() = 0;
  if (sphereMin.z() < 0)
    sphereMin.z() = 0;

  _gridCell[0] =
  (int)((sphereMin.z() * _gridRes.y() + sphereMin.y()) * _gridRes.x() +
        sphereMin.x());
  _gridCell[1] = _gridCell[0] + 1;
  _gridCell[2] = (int)(_gridCell[0] + _gridRes.x());  // like y + 1
  _gridCell[3] = _gridCell[2] + 1;

  if (sphereMin.z() + 1 < _gridRes.z()) {  // if not cell with max z
    _gridCell[4] =
    (int)(_gridCell[0] + _gridRes.y() * _gridRes.x());  // like z + 1
    _gridCell[5] = _gridCell[4] + 1;
    _gridCell[6] = (int)(_gridCell[4] + _gridRes.x());  // like y + 1
    _gridCell[7] = _gridCell[6] + 1;                    // like x + 1
  }

  if (sphereMin.x() + 1 >= _gridRes.x()) {  // if cell with max x
    _gridCell[1] = -1;
    _gridCell[3] = -1;
    _gridCell[5] = -1;
    _gridCell[7] = -1;
  }

  if (sphereMin.y() + 1 >= _gridRes.y()) {  // if cell with max y
    _gridCell[2] = -1;
    _gridCell[3] = -1;
    _gridCell[6] = -1;
    _gridCell[7] = -1;
  }
}
