#include "draw_manager.h"

#include <App/Applied/Visitors/draw_visitor.h>

#include <App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp>
#include <App/Applied/Primitives/Vector4D/vector_4d.hpp>
#include <memory>
#include <thread>
#include <utility>

#define N_THRS 4

void DrawManager::drawScene(const std::shared_ptr<Scene>& scene, const std::shared_ptr<BaseDrawer> drawer,
                            const std::shared_ptr<Camera> camera, const DrawMode mode,
                            const ParticleMode particleMode) {
  //  qDebug() << "DrawManager::drawScene() START";
  // get lights
  const auto lights = scene->lights();

  drawer->clear();

  // draw objects
  //  int intent = drawer->height() / N_THRS;
  //
  //  int ymin, ymax;
  //  std::vector<std::thread> th_vec;
  //  std::vector<DrawVisitor> vis_vec;
  //  for (int i = 0; i < N_THRS; ++i) {
  //    ymin = i * intent;
  //    ymax = (i + 1) * intent;
  //    vis_vec.push_back(DrawVisitor(drawer, camera, lights, mode, particleMode, ymin, ymax));
  //    th_vec.emplace_back([&]() { scene->objects()->accept(vis_vec.at(i)); });
  //  }
  //
  //  for (int i = 0; i < N_THRS; ++i) {
  //    th_vec.at(i).join();
  //  }

  int intent = drawer->height() / 16;
  int y1_min = 0, y1_max = intent;
  auto visitor1 = DrawVisitor(drawer, camera, lights, mode, particleMode, y1_min, y1_max);
  std::thread t1([&]() { scene->objects()->accept(visitor1); });
  //  scene->objects()->accept(visitor1);

  int y2_min = intent, y2_max = intent * 2;
  auto visitor2 = DrawVisitor(drawer, camera, lights, mode, particleMode, y2_min, y2_max);
  std::thread t2([&]() { scene->objects()->accept(visitor2); });

  int y3_min = intent * 2, y3_max = intent * 3;
  auto visitor3 = DrawVisitor(drawer, camera, lights, mode, particleMode, y3_min, y3_max);
  std::thread t3([&]() { scene->objects()->accept(visitor3); });

  int y4_min = intent * 3, y4_max = intent * 4;
  auto visitor4 = DrawVisitor(drawer, camera, lights, mode, particleMode, y4_min, y4_max);
  std::thread t4([&]() { scene->objects()->accept(visitor4); });

  int y5_min = intent * 4, y5_max = intent * 5;
  auto visitor5 = DrawVisitor(drawer, camera, lights, mode, particleMode, y5_min, y5_max);
  std::thread t5([&]() { scene->objects()->accept(visitor5); });

  int y6_min = intent * 5, y6_max = intent * 6;
  auto visitor6 = DrawVisitor(drawer, camera, lights, mode, particleMode, y6_min, y6_max);
  std::thread t6([&]() { scene->objects()->accept(visitor6); });

  int y7_min = intent * 6, y7_max = intent * 7;
  auto visitor7 = DrawVisitor(drawer, camera, lights, mode, particleMode, y7_min, y7_max);
  std::thread t7([&]() { scene->objects()->accept(visitor7); });

  int y8_min = intent * 7, y8_max = intent * 8;
  auto visitor8 = DrawVisitor(drawer, camera, lights, mode, particleMode, y8_min, y8_max);
  std::thread t8([&]() { scene->objects()->accept(visitor8); });

  int y9_min = intent * 8, y9_max = intent * 9;
  auto visitor9 = DrawVisitor(drawer, camera, lights, mode, particleMode, y9_min, y9_max);
  std::thread t9([&]() { scene->objects()->accept(visitor9); });

  int y10_min = intent * 9, y10_max = intent * 10;
  auto visitor10 = DrawVisitor(drawer, camera, lights, mode, particleMode, y10_min, y10_max);
  std::thread t10([&]() { scene->objects()->accept(visitor10); });

  int y11_min = intent * 10, y11_max = intent * 11;
  auto visitor11 = DrawVisitor(drawer, camera, lights, mode, particleMode, y11_min, y11_max);
  std::thread t11([&]() { scene->objects()->accept(visitor11); });

  int y12_min = intent * 11, y12_max = intent * 12;
  auto visitor12 = DrawVisitor(drawer, camera, lights, mode, particleMode, y12_min, y12_max);
  std::thread t12([&]() { scene->objects()->accept(visitor12); });

  int y13_min = intent * 12, y13_max = intent * 13;
  auto visitor13 = DrawVisitor(drawer, camera, lights, mode, particleMode, y13_min, y13_max);
  std::thread t13([&]() { scene->objects()->accept(visitor13); });

  int y14_min = intent * 13, y14_max = intent * 14;
  auto visitor14 = DrawVisitor(drawer, camera, lights, mode, particleMode, y14_min, y14_max);
  std::thread t14([&]() { scene->objects()->accept(visitor14); });

  int y15_min = intent * 14, y15_max = intent * 15;
  auto visitor15 = DrawVisitor(drawer, camera, lights, mode, particleMode, y15_min, y15_max);
  std::thread t15([&]() { scene->objects()->accept(visitor15); });

  int y16_min = intent * 15, y16_max = intent * 16;
  auto visitor16 = DrawVisitor(drawer, camera, lights, mode, particleMode, y16_min, y16_max);
  std::thread t16([&]() { scene->objects()->accept(visitor16); });
//
//  int y17_min = intent * 16, y17_max = intent * 17;
//  auto visitor17 = DrawVisitor(drawer, camera, lights, mode, particleMode, y17_min, y17_max);
//  std::thread t17([&]() { scene->objects()->accept(visitor17); });
//
//  int y18_min = intent * 17, y18_max = intent * 18;
//  auto visitor18 = DrawVisitor(drawer, camera, lights, mode, particleMode, y18_min, y18_max);
//  std::thread t18([&]() { scene->objects()->accept(visitor18); });
//
//  int y19_min = intent * 18, y19_max = intent * 19;
//  auto visitor19 = DrawVisitor(drawer, camera, lights, mode, particleMode, y19_min, y19_max);
//  std::thread t19([&]() { scene->objects()->accept(visitor19); });
//
//  int y20_min = intent * 19, y20_max = intent * 20;
//  auto visitor20 = DrawVisitor(drawer, camera, lights, mode, particleMode, y20_min, y20_max);
//  std::thread t20([&]() { scene->objects()->accept(visitor20); });
//
//  int y21_min = intent * 20, y21_max = intent * 21;
//  auto visitor21 = DrawVisitor(drawer, camera, lights, mode, particleMode, y21_min, y21_max);
//  std::thread t21([&]() { scene->objects()->accept(visitor21); });
//
//  int y22_min = intent * 21, y22_max = intent * 22;
//  auto visitor22 = DrawVisitor(drawer, camera, lights, mode, particleMode, y22_min, y22_max);
//  std::thread t22([&]() { scene->objects()->accept(visitor22); });
//
//  int y23_min = intent * 22, y23_max = intent * 23;
//  auto visitor23 = DrawVisitor(drawer, camera, lights, mode, particleMode, y23_min, y23_max);
//  std::thread t23([&]() { scene->objects()->accept(visitor23); });
//
//  int y24_min = intent * 23, y24_max = intent * 24;
//  auto visitor24 = DrawVisitor(drawer, camera, lights, mode, particleMode, y24_min, y24_max);
//  std::thread t24([&]() { scene->objects()->accept(visitor24); });
//
//  int y25_min = intent * 24, y25_max = intent * 25;
//  auto visitor25 = DrawVisitor(drawer, camera, lights, mode, particleMode, y25_min, y25_max);
//  std::thread t25([&]() { scene->objects()->accept(visitor25); });
//
//  int y26_min = intent * 25, y26_max = intent * 26;
//  auto visitor26 = DrawVisitor(drawer, camera, lights, mode, particleMode, y26_min, y26_max);
//  std::thread t26([&]() { scene->objects()->accept(visitor26); });
//
//  int y27_min = intent * 26, y27_max = intent * 27;
//  auto visitor27 = DrawVisitor(drawer, camera, lights, mode, particleMode, y27_min, y27_max);
//  std::thread t27([&]() { scene->objects()->accept(visitor27); });
//
//  int y28_min = intent * 27, y28_max = intent * 28;
//  auto visitor28 = DrawVisitor(drawer, camera, lights, mode, particleMode, y28_min, y28_max);
//  std::thread t28([&]() { scene->objects()->accept(visitor28); });
//
//  int y29_min = intent * 28, y29_max = intent * 29;
//  auto visitor29 = DrawVisitor(drawer, camera, lights, mode, particleMode, y29_min, y29_max);
//  std::thread t29([&]() { scene->objects()->accept(visitor29); });
//
//  int y30_min = intent * 29, y30_max = intent * 30;
//  auto visitor30 = DrawVisitor(drawer, camera, lights, mode, particleMode, y30_min, y30_max);
//  std::thread t30([&]() { scene->objects()->accept(visitor30); });
//
//  int y31_min = intent * 30, y31_max = intent * 31;
//  auto visitor31 = DrawVisitor(drawer, camera, lights, mode, particleMode, y31_min, y31_max);
//  std::thread t31([&]() { scene->objects()->accept(visitor31); });
//
//  int y32_min = intent * 31, y32_max = intent * 32;
//  auto visitor32 = DrawVisitor(drawer, camera, lights, mode, particleMode, y32_min, y32_max);
//  std::thread t32([&]() { scene->objects()->accept(visitor32); });

  t1.join();
  t2.join();
  t3.join();
  t4.join();
  t5.join();
  t6.join();
  t7.join();
  t8.join();
  t9.join();
  t10.join();
  t11.join();
  t12.join();
  t13.join();
  t14.join();
  t15.join();
  t16.join();
//  t17.join();
//  t18.join();
//  t19.join();
//  t20.join();
//  t21.join();
//  t22.join();
//  t23.join();
//  t24.join();
//  t25.join();
//  t26.join();
//  t27.join();
//  t28.join();
//  t29.join();
//  t30.join();
//  t31.join();
//  t32.join();
  drawer->update();
  //  qDebug() << "DrawManager::drawScene() END";
}
