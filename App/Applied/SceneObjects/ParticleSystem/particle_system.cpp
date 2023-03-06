#include <App/Applied/SPH/common/common_defs.h>
#include <App/Applied/SceneObjects/ParticleSystem/particle_system.h>
#include <App/Applied/SceneObjects/camera.h>

#include <QDebug>
#include <utility>

#define EPSILON 0.00001f  // for collision detection
#define RTOD double(180.0) / M_PI

ParticleSystem::ParticleSystem(std::shared_ptr<BaseObject> model, Bound3D mb) : _obstacle(std::move(model)), _mb(mb) {
}

void ParticleSystem::initialize(size_t nParticles) {
  _particles.reserve(nParticles);
  _maxp = nParticles;
  _gridRes.set(0, 0, 0);

  _time = 0;
  //  _dt = 0.1;
  _dt = 0.003;

  _clr_mode = 2.0;
  _sim_scale = 0.004;  // unit size

  _viscosity = 0.2;  // custom
  //  _viscosity = 1.81e-5;  // pascal-second (Pa.s) = 1 kg m^-1 s^-1
  //  (see
  // wikipedia page on viscosity) 1.81 × 10-5
  //  _viscosity = 0.2;  // pascal-second (Pa.s) = 1 kg m^-1 s^-1
  //  (see
  //                                 // wikipedia page on viscosity)

  //  _rest_density = 600.0;  // kg / m^3; WATER
  //    _rest_density = 1.225;  // kg / m^3 AIR
  _rest_density = 300;    // CUSTOM
  _p_mass = 0.00020543;   // kg WATER
  _p_radius = 0.004;      // m
  _p_dist = 0.0059;       // m
  _smooth_radius = 0.01;  // m

  _ext_damp = 256.0;
  //  _limit = 200.0;  // m / s DEFAULT
  _limit = 200.0;  // CUSTOM

  //  _int_stiff= 1.00;
  _int_stiff = 0.50;  // default 0.50
                      //    _int_stiff= 0.80; //   custom

  //   _ext_stiff = 10000.0;
  //  _ext_stiff = 20000; // default 20000
  _ext_stiff = 10000;  //  custom
}

void ParticleSystem::createExample(float xMin, float xMax, float yMin, float yMax, float zMin, float zMax) {
  _vol_min.set(xMin, yMin, zMin);
  _vol_max.set(xMax, yMax, zMax);

  _init_min.set(xMin, yMin, zMin);
  _init_max.set(xMax, yMax, zMax);

  computeKernels();

  _p_dist = pow(_p_mass / _rest_density, 1 / 3.0);

  float ss = _p_dist * 0.87 / _sim_scale;
  //  std::cout << "Spacing: %f " << ss << std::endl;

  //  addVolume(_init_min, _init_max,
  //            ss);  // Create the particles  // Create the particles with good
  //  //            positions and velocities

  float cell_size = _smooth_radius * 2.0;  // Grid cell size (2r)
  gridSetup(_vol_min, _vol_max, _sim_scale, cell_size,
            1.0);  // Setup grid

  gridInsertParticles();
}

void ParticleSystem::run() {
  float ss = _p_dist / _sim_scale;  // simulation scale

  if (_time - _last_emit_time > _emit_rate) {
    //    std::cout << "Time to emit!\n";
    emitParticles();
    _last_emit_time = _time;
  }

  gridInsertParticles();

  computePressureGrid();

  computeForceGridNC();

  advance();
}

void ParticleSystem::advance() {
  std::vector<Face> faces;
  std::vector<Vec3f> vertices;
  if (_obstacle) {
    const auto model = std::dynamic_pointer_cast<PolygonalModel>(_obstacle);
    faces = model->components()->faces();
    vertices = model->components()->vertices();
  }
  Vec3f v1, v2, v3;
  Vec3f normal, p1, p2, pi;
  float A, B, C, D, t, numerator, denominator;
  Vec3f pc1, pc2, pc3;
  float a1, a2, a3, dist;

  Vec3f norm, accel, vel_next_half;
  Vec3f min, max;
  double adj;
  float ss, radius;
  float stiff, damp, speed, diff;

  stiff = _ext_stiff;  // коэффициент жесткости для столкновения со
  // стенками и полигонами обьектов
  damp = _ext_damp;
  radius = _p_radius;
  min = _vol_min;
  max = _vol_max;
  ss = _sim_scale;

  Vec3i d = Vec3i((max - min).x(), (max - min).y(), (max - min).z());

  for (auto p_iter = _particles.begin(); p_iter != _particles.end(); ++p_iter) {
    if (std::isnan(p_iter->pos.x()) || std::isnan(p_iter->pos.y()) || std::isnan(p_iter->pos.z())) {
      auto saved = p_iter - 1;
      //      std::cout << "Broken particle, p_size = " << _particles.size()
      //                << std::endl;
      _particles.erase(p_iter);
      p_iter = saved;
      continue;
    }

    // Compute Acceleration
    accel = p_iter->force * _p_mass;  // Why, a = F / m?

    // Velocity limiting
    speed = accel.x() * accel.x() + accel.y() * accel.y() + accel.z() * accel.z();
    if (speed > _limit * _limit) {
      accel *= _limit / sqrt(speed);
    }

    // Z-axis walls
    diff = 2 * radius - (p_iter->pos.z() - min.z()) * ss;
    if (diff > EPSILON) {
      norm.set(0, 0, 1);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p_iter->vel_actual);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }
    diff = 2 * radius - (max.z() - p_iter->pos.z()) * ss;
    if (diff > EPSILON) {
      norm.set(0, 0, -1);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p_iter->vel_actual);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // X-axis walls
    diff = 2 * radius - (p_iter->pos.x() - min.x()) * ss;
    if (diff > EPSILON) {
      auto saved = p_iter - 1;
      //      std::cout << "Particle reached end, p_size = " <<
      //      _particles.size()
      //                << std::endl;
      _particles.erase(p_iter);
      p_iter = saved;
      continue;
    }
    diff = 2 * radius - (max.x() - p_iter->pos.x()) * ss;
    if (diff > EPSILON) {
      norm.set(-1, 0, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p_iter->vel_actual);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    // Y-axis walls
    diff = 2 * radius - (p_iter->pos.y() - min.y()) * ss;
    if (diff > EPSILON) {
      norm.set(0, 1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p_iter->vel_actual);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }
    diff = 2 * radius - (max.y() - p_iter->pos.y()) * ss;
    if (diff > EPSILON) {
      norm.set(0, -1, 0);

      adj = stiff * diff - damp * Vec3f::dotProduct(norm, p_iter->vel_actual);
      accel.x() += adj * norm.x();
      accel.y() += adj * norm.y();
      accel.z() += adj * norm.z();
    }

    bool flag_changed = false;
    if (_obstacle) {
      // Obstacle collision
      p1 = p_iter->pos, p2 = (p_iter->pos + p_iter->vel_actual);

      // integration
      vel_next_half = accel * _dt + p_iter->vel_prev_half;  // v(t+1/2) = v(t-1/2) + a(t) dt

      Vec3f new_vel = (p_iter->vel_prev_half + vel_next_half) * 0.5;  // v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5
                                                                      // used to compute forces later
      vel_next_half *= _dt / ss;
      Vec3f new_pos = p_iter->pos + vel_next_half;  // p(t+1) = p(t) + v(t+1/2) dt
      p2 = new_pos;

      for (const auto &face : faces) {
        v1 = vertices[face.vertex(0)];
        v2 = vertices[face.vertex(1)];
        v3 = vertices[face.vertex(2)];
        v1 *= 30;
        v2 *= 30;
        v3 *= 30;

        normal = Vec3f::crossProduct(v2 - v1, v3 - v1).normalize();

        A = normal.x();
        B = normal.y();
        C = normal.z();
        D = -(A * v1.x() + B * v1.y() + C * v1.z());

        numerator = A * p1.x() + B * p1.y() + C * p1.z() + D;
        denominator = A * (p1.x() - p2.x()) + B * (p1.y() - p2.y()) + C * (p1.z() - p2.z());

        // прямая параллельна плоскости
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

        //        if (fabs((a1 + a2 + a3) * RTOD - 360) > 1e-3)
        if (fabs((a1 + a2 + a3) - M_PI * 2.0f) > 1e-1)
          //        if (fabs((a1 + a2 + a3) * RTOD - 360) > 3)
          continue;

        // change accel
        dist = (pi - p1).length();
        diff = 2 * radius - dist * ss;
        if (diff < 2 * radius && diff > 1e-3) {
          //          adj = 2 * stiff * diff - damp * Vec3f::dotProduct(normal, p_iter->vel_actual);
          adj = 2 * stiff * diff - damp * Vec3f::dotProduct(normal, new_vel);
          accel.x() += adj * normal.x();
          accel.y() += adj * normal.y();
          accel.z() += adj * normal.z();

          //          auto saved = p_iter - 1;
          //          //      std::cout << "Broken particle, p_size = " << _particles.size()
          //          //                << std::endl;
          //          _particles.erase(p_iter);
          //          p_iter = saved;
          //          flag_changed = true;
          //          break;
        }
      }
    }

    //    if (flag_changed) {
    //      continue;
    //    }

    // Leapfrog Integration
    // calculate new velocity and position
    vel_next_half = accel * _dt + p_iter->vel_prev_half;  // v(t+1/2) = v(t-1/2) + a(t) dt

    p_iter->vel_actual = (p_iter->vel_prev_half + vel_next_half) * 0.5;  // v(t+1) = [v(t-1/2) + v(t+1/2)] * 0.5
                                                                         // used to compute forces later
    p_iter->vel_prev_half = vel_next_half;
    vel_next_half *= _dt / ss;
    p_iter->pos += vel_next_half;  // p(t+1) = p(t) + v(t+1/2) dt

    // check particle in model
    flag_changed = false;
    bool dir = false;
    int l = 0;
    if (_obstacle) {
      for (const auto &face : faces) {
        v1 = vertices[face.vertex(0)];
        v2 = vertices[face.vertex(1)];
        v3 = vertices[face.vertex(2)];
        v1 *= 30;
        v2 *= 30;
        v3 *= 30;

        normal = Vec3f::crossProduct(v2 - v1, v3 - v1).normalize();

        A = normal.x();
        B = normal.y();
        C = normal.z();

        D = -(A * v1.x() + B * v1.y() + C * v1.z());

        float value = A * p_iter->pos.x() + B * p_iter->pos.y() + C * p_iter->pos.z() + D;

        if (l == 0) {
          if (value <= 0) {
            dir = true;
          } else {
            dir = false;
          }
          ++l;
          continue;
        }

        if (value <= 0) {
          if (dir != true) {
            flag_changed = true;
            break;
          }
        } else {
          if (dir != false) {
            flag_changed = true;
            break;
          }
        }

        ++l;
      }
    }

    if (!flag_changed) {
      auto saved = p_iter - 1;
      //      std::cout << "Broken particle, p_size = " << _particles.size()
      //                << std::endl;
      _particles.erase(p_iter);
      p_iter = saved;
      continue;
    }

    // set color
    if (_clr_mode == 1.0) {
      adj = fabs(vel_next_half.x()) + fabs(vel_next_half.y()) + fabs(vel_next_half.z()) / 7000.0;
      adj = (adj > 1.0) ? 1.0 : adj;
      p_iter->clr = COLORA(0, adj, adj, 1);
    } else if (_clr_mode == 2.0) {
      float v = 0.5 + (p_iter->pressure / 1500.0);
      if (v < 0.1)
        v = 0.1;
      if (v > 1.0)
        v = 1.0;
      p_iter->clr = COLORA(v, 1 - v, 0, 1);
    }
  }

  _time += _dt;
}

void ParticleSystem::emitParticles() {
  //  std::cout << "Emit...\n";
  Vec3f dir = {-start_speed, 0, 0};
  Vec3f pos;

  int i = 0;
  while (_particles.size() < _maxp && i < _maxp * 0.05) {
    pos.setX(_vol_max.x());
    pos.setY(_vol_min.y() + 1 + rand() % (int)(_vol_max.y() - 2 - _vol_min.y()));
    pos.setZ(_vol_min.z() + 1 + rand() % (int)(_vol_max.z() - 2 - _vol_min.z()));

    Particle &p = _particles.emplace_back();
    p.pos = pos;
    p.vel_prev_half = dir;
    p.vel_actual = dir;
    p.clr = COLORA(0, 0, 0, 1);
    //    std::cout << "Emmited particle: " << p.pos << p.vel_prev_half << std::endl;
    //    std::cout << "Particles Size = " << _particles.size() << std::endl;
    ++i;
  }
}

void ParticleSystem::computeKernels() {
  _p_dist = pow(_p_mass / _rest_density, 1 / 3.0);

  _R2 = _smooth_radius * _smooth_radius;
  _Poly6Kern = 315.0f / (64.0f * PI * pow(_smooth_radius, 9));  // Wpoly6 kernel (denominator
  // part) - 2003 Muller, p.4
  _SpikyKern = -45.0f / (PI * pow(_smooth_radius,
                                  6));  // Laplacian of viscocity (denominator): PI h^6
  _LapKern = 45.0f / (PI * pow(_smooth_radius, 6));
}

// Compute Pressures - Using spatial grid, and also create neighbor table
void ParticleSystem::computePressureGrid() {
  int pIdx;
  float dx, dy, dz, sum, dSqr, c;
  float h = _smooth_radius, h2 = h * h;
  float simH = _smooth_radius / _sim_scale;

  int i = 0;

  Particle *pCur;
  for (auto &p : _particles) {
    sum = 0.0;
    _neighborCount[i] = 0;

    gridFindCells(p.pos, simH);

    for (int &gridIdx : _gridCell) {
      if (gridIdx != -1) {
        pIdx = _grid[gridIdx];
        while (pIdx != -1) {           // if gridIdx-th cell has 1 or more particles
          pCur = &(_particles[pIdx]);  // get particle by index
          if (pCur == &p) {
            pIdx = pCur->next;  // get next particle in this cell
            continue;           // ignore itself
          }

          // dist between particles in cm
          dx = (p.pos.x() - pCur->pos.x()) * _sim_scale;
          dy = (p.pos.y() - pCur->pos.y()) * _sim_scale;
          dz = (p.pos.z() - pCur->pos.z()) * _sim_scale;
          dSqr = (dx * dx + dy * dy + dz * dz);

          if (dSqr < h2) {   // нужна ли тут проверка?
            c = _R2 - dSqr;  // kernel
            sum += c * c * c;
            if (_neighborCount[i] < MAX_NEIGHBOR) {
              _neighbor[i][_neighborCount[i]] = pIdx;
              _neighborDist[i][_neighborCount[i]] = sqrt(dSqr);
              _neighborCount[i]++;
            }
          }

          pIdx = pCur->next;  // get next particle in this cell
        }
      }

      gridIdx = -1;  // точно ли это нужно?
    }

    p.density = sum * _p_mass * _Poly6Kern;
    p.pressure = (p.density - _rest_density) * _int_stiff;
    p.density = 1.0f / p.density;
    ++i;
  }
}

// Compute Forces - Using spatial grid with saved neighbor table. Fastest.
void ParticleSystem::computeForceGridNC() {
  Vec3f force;
  float pterm, vterm, dterm;
  int i = 0;
  float c;
  float dx, dy, dz;

  Particle *pCur;
  for (auto &p : _particles) {
    force.set(0, 0, 0);

    for (int j = 0; j < _neighborCount[i]; j++) {
      pCur = &(_particles[_neighbor[i][j]]);

      // dist in cm
      dx = (p.pos.x() - pCur->pos.x()) * _sim_scale;
      dy = (p.pos.y() - pCur->pos.y()) * _sim_scale;
      dz = (p.pos.z() - pCur->pos.z()) * _sim_scale;

      c = _smooth_radius - _neighborDist[i][j];

      pterm = -0.5f * c * _SpikyKern * (p.pressure + pCur->pressure) / _neighborDist[i][j];

      dterm = c * p.density * pCur->density;

      vterm = _LapKern * _viscosity;

      force.x() += (pterm * dx + vterm * (pCur->vel_actual.x() - p.vel_actual.x())) * dterm;
      force.y() += (pterm * dy + vterm * (pCur->vel_actual.y() - p.vel_actual.y())) * dterm;
      force.z() += (pterm * dz + vterm * (pCur->vel_actual.z() - p.vel_actual.z())) * dterm;
    }

    p.force = force;
    ++i;
  }
}

void ParticleSystem::gridSetup(Vec3f min, Vec3f max, float sim_scale, float cell_size, float border) {
  float world_cellsize = cell_size / sim_scale;
  //  std::cout << "[DBG]: World Cell Size = " << world_cellsize << std::endl;
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
  //  _gridCnt.clear();

  _grid.reserve(_gridTotal);
  //  _gridCnt.reserve(_gridTotal);

  for (int n = 0; n < _gridTotal; n++) {
    _grid.push_back(-1);
    //    _gridCnt.push_back(0);
  }
}

void ParticleSystem::gridInsertParticles() {
  //  qDebug() << "gridInsertParticles()";
  int gridIdx;

  // init particles
  for (auto &p : _particles) {
    p.next = -1;
  }

  // init grid
  for (int i = 0; i < _gridTotal; i++) {
    _grid[i] = -1;
    //    _gridCnt[i] = 0;
  }

  int pIdx = 0;
  int xIdx, yIdx, zIdx;
  for (auto &p : _particles) {
    // Determine grid cell id
    xIdx = (int)((p.pos.x() - _gridMin.x()) * _gridDelta.x());
    yIdx = (int)((p.pos.y() - _gridMin.y()) * _gridDelta.y());
    zIdx = (int)((p.pos.z() - _gridMin.z()) * _gridDelta.z());

    // Determine grid cell id in _grid array
    gridIdx = (int)((zIdx * _gridRes.y() + yIdx) * _gridRes.x() + xIdx);

    if (gridIdx >= 0 && gridIdx < _gridTotal) {
      p.next = _grid[gridIdx];
      _grid[gridIdx] = pIdx;
      //      _gridCnt[gridIdx]++;
    }
    pIdx++;
  }

  //  auto it = _particles.begin();
  //  for (int i = 0; i < 10 && it != _particles.end(); ++i, ++it) {
  //    std::cout << *it << std::endl;
  //  }
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

  _gridCell[0] = (int)((sphereMin.z() * _gridRes.y() + sphereMin.y()) * _gridRes.x() + sphereMin.x());
  _gridCell[1] = _gridCell[0] + 1;
  _gridCell[2] = (int)(_gridCell[0] + _gridRes.x());  // like y + 1
  _gridCell[3] = _gridCell[2] + 1;

  if (sphereMin.z() + 1 < _gridRes.z()) {                              // if not cell with max z
    _gridCell[4] = (int)(_gridCell[0] + _gridRes.y() * _gridRes.x());  // like z + 1
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
