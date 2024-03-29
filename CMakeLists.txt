cmake_minimum_required(VERSION 3.14)
project(air-flow-simulation)

# variables
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif ()

# options
option(BUILD_TESTS "Build the unit tests" ON)
option(USE_SANITIZER_FLAGS "Use sanitizer flags" ON)
option(USE_COVERAGE_FLAGS "Use coverage flags" ON)

find_package(Qt5 COMPONENTS Core Gui Widgets REQUIRED)

include_directories(
        App/Applied
        App/Applied/Color
        App/Applied/Commands
        App/Applied/Commands/CameraCommands
        App/Applied/Commands/ModelCommands
        App/Applied/Commands/SceneCommands
        App/Applied/Containers/Iterator
        App/Applied/Containers/matrix/iterator
        App/Applied/Containers/matrix/matrix
        App/Applied/Containers/Vector
        App/Applied/Drawer
        App/Applied/Exceptions
        App/Applied/Facade
        App/Applied/Image
        App/Applied/Load/Builders
        App/Applied/Load/Builders/Camera
        App/Applied/Load/Builders/Model
        App/Applied/Load/Directors
        App/Applied/Load/Directors/CameraDirector
        App/Applied/Load/Directors/ModelDirector
        App/Applied/Load/Loaders
        App/Applied/Load/Loaders/CameraLoader
        App/Applied/Load/Loaders/ModelLoader
        App/Applied/Managers
        App/Applied/Primitives/Edge
        App/Applied/Primitives/Face
        App/Applied/Scene
        App/Applied/SceneObjects
        App/Applied/SceneObjects/Model
        App/Applied/SceneObjects/ParticleSystem
        App/Applied/Solutions
        App/Applied/SPH/common
        App/Applied/Visitors
        App/Ui/Qt
        App/Ui/Qt/Drawer
        App/Workers
)

set(SRCS
        App/Applied/Color/color.c++
        App/Applied/Color/color.h
        App/Applied/Commands/CameraCommands/add_camera_command.cpp
        App/Applied/Commands/CameraCommands/add_camera_command.h
        App/Applied/Commands/CameraCommands/camera_command.h
        App/Applied/Commands/CameraCommands/count_cameras_command.cpp
        App/Applied/Commands/CameraCommands/count_cameras_command.h
        App/Applied/Commands/CameraCommands/load_camera_command.cpp
        App/Applied/Commands/CameraCommands/load_camera_command.h
        App/Applied/Commands/CameraCommands/move_camera_command.cpp
        App/Applied/Commands/CameraCommands/move_camera_command.h
        App/Applied/Commands/CameraCommands/remove_camera_command.cpp
        App/Applied/Commands/CameraCommands/remove_camera_command.h
        App/Applied/Commands/CameraCommands/rotate_camera_command.h
        App/Applied/Commands/CameraCommands/set_cur_camera_command.cpp
        App/Applied/Commands/CameraCommands/set_cur_camera_command.h
        App/Applied/Commands/LightCommands/add_light_command.hpp
        App/Applied/Commands/ModelCommands/add_model_command.cpp
        App/Applied/Commands/ModelCommands/add_model_command.h
        App/Applied/Commands/ModelCommands/count_models_command.cpp
        App/Applied/Commands/ModelCommands/count_models_command.h
        App/Applied/Commands/ModelCommands/load_model_command.cpp
        App/Applied/Commands/ModelCommands/load_model_command.h
        App/Applied/Commands/ModelCommands/load_polygonal_model_command.cpp
        App/Applied/Commands/ModelCommands/load_polygonal_model_command.h
        App/Applied/Commands/ModelCommands/model_command.h
        App/Applied/Commands/ModelCommands/move_model_command.cpp
        App/Applied/Commands/ModelCommands/move_model_command.h
        App/Applied/Commands/ModelCommands/remove_model_command.cpp
        App/Applied/Commands/ModelCommands/remove_model_command.h
        App/Applied/Commands/ModelCommands/rotate_model_command.cpp
        App/Applied/Commands/ModelCommands/rotate_model_command.h
        App/Applied/Commands/ModelCommands/scale_model_command.cpp
        App/Applied/Commands/ModelCommands/scale_model_command.h
        App/Applied/Commands/ModelCommands/transform_model_command.cpp
        App/Applied/Commands/ModelCommands/transform_model_command.h
        App/Applied/Commands/ParticleSystemCommands/add_particle_system_command.hpp
        App/Applied/Commands/ParticleSystemCommands/remove_particle_system_command.hpp
        App/Applied/Commands/SceneCommands/draw_scene_command.cpp
        App/Applied/Commands/SceneCommands/draw_scene_command.h
        App/Applied/Commands/SceneCommands/get_cur_camera_command.cpp
        App/Applied/Commands/SceneCommands/get_cur_camera_command.h
        App/Applied/Commands/SceneCommands/get_object_command.cpp
        App/Applied/Commands/SceneCommands/get_object_command.h
        App/Applied/Commands/SceneCommands/get_scene_command.cpp
        App/Applied/Commands/SceneCommands/get_scene_command.h
        App/Applied/Commands/SceneCommands/scene_command.h
        App/Applied/Commands/base_command.h
        App/Applied/Containers/Iterator/base_iterator.h
        App/Applied/Containers/Iterator/base_iterator.hpp
        App/Applied/Containers/Iterator/vector_const_iterator.h
        App/Applied/Containers/Iterator/vector_const_iterator.hpp
        App/Applied/Containers/Iterator/vector_iterator.h
        App/Applied/Containers/Iterator/vector_iterator.hpp
        App/Applied/Containers/matrix/errors/errors.cpp
        App/Applied/Containers/matrix/errors/errors.hpp
        App/Applied/Containers/matrix/iterator/const_matrix_iterator.h
        App/Applied/Containers/matrix/iterator/const_matrix_iterator.hpp
        App/Applied/Containers/matrix/iterator/matrix_iterator.h
        App/Applied/Containers/matrix/iterator/matrix_iterator.hpp
        App/Applied/Containers/matrix/matrix/matrix.h
        App/Applied/Containers/matrix/matrix/matrix.hpp
        App/Applied/Containers/matrix/matrix/matrix_base.cpp
        App/Applied/Containers/matrix/matrix/matrix_base.h
        App/Applied/Containers/Vector/base_vector.h
        App/Applied/Containers/Vector/base_vector.hpp
        App/Applied/Containers/Vector/vector.h
        App/Applied/Containers/Vector/vector.hpp
        App/Applied/Drawer/drawer.h
        App/Applied/Drawer/solution_drawer_factory.cpp
        App/Applied/Drawer/solution_drawer_factory.h
        App/Applied/Exceptions/base_exception.h
        App/Applied/Exceptions/camera_exceptions.h
        App/Applied/Exceptions/load_exceptions.h
        App/Applied/Exceptions/model_exceptions.h
        App/Applied/Facade/facade.h
        App/Applied/Image/image.cpp
        App/Applied/Image/image.h
        App/Applied/Load/Builders/Camera/camera_builder.h
        App/Applied/Load/Builders/Camera/file_camera_builder.cpp
        App/Applied/Load/Builders/Camera/file_camera_builder.h
        App/Applied/Load/Builders/Model/file_model_builder.h
        App/Applied/Load/Builders/Model/file_polygon_model_builder.cpp
        App/Applied/Load/Builders/Model/file_polygon_model_builder.h
        App/Applied/Load/Builders/Model/file_wireframe_model_builder.cpp
        App/Applied/Load/Builders/Model/file_wireframe_model_builder.h
        App/Applied/Load/Builders/Model/model_builder.h
        App/Applied/Load/Builders/object_builder.h
        App/Applied/Load/Directors/CameraDirector/camera_load_director.cpp
        App/Applied/Load/Directors/CameraDirector/camera_load_director.h
        App/Applied/Load/Directors/ModelDirector/model_load_director.h
        App/Applied/Load/Directors/ModelDirector/polygonal_model_load_director.cpp
        App/Applied/Load/Directors/ModelDirector/polygonal_model_load_director.h
        App/Applied/Load/Directors/ModelDirector/wireframe_model_load_director.cpp
        App/Applied/Load/Directors/ModelDirector/wireframe_model_load_director.h
        App/Applied/Load/Directors/base_load_director.h
        App/Applied/Load/Loaders/CameraLoader/camera_file_loader.cpp
        App/Applied/Load/Loaders/CameraLoader/camera_file_loader.h
        App/Applied/Load/Loaders/ModelLoader/polygonal_model_file_loader.cpp
        App/Applied/Load/Loaders/ModelLoader/polygonal_model_file_loader.h
        App/Applied/Load/Loaders/ModelLoader/wireframe_model_file_loader.cpp
        App/Applied/Load/Loaders/ModelLoader/wireframe_model_file_loader.h
        App/Applied/Load/Loaders/base_file_loader.h
        App/Applied/Managers/base_manager.h
        App/Applied/Managers/draw_manager.cpp
        App/Applied/Managers/draw_manager.h
        App/Applied/Managers/load_manager.cpp
        App/Applied/Managers/load_manager.h
        App/Applied/Managers/main_manager.cpp
        App/Applied/Managers/main_manager.h
        App/Applied/Managers/manager_creator.h
        App/Applied/Managers/scene_manager.cpp
        App/Applied/Managers/scene_manager.h
        App/Applied/Managers/transform_manager.cpp
        App/Applied/Managers/transform_manager.h
        App/Applied/Primitives/Edge/edge.cpp
        App/Applied/Primitives/Edge/edge.h
        App/Applied/Primitives/Face/face.cpp
        App/Applied/Primitives/Face/face.h
        App/Applied/Primitives/Matrix4x4/matrix_4x4.hpp
        App/Applied/Primitives/Vector2D/vector_2d.hpp
        App/Applied/Primitives/Vector3D/vector_3d.hpp
        App/Applied/Primitives/Vector4D/vector_4d.hpp
        App/Applied/Scene/scene.cpp
        App/Applied/Scene/scene.h
        App/Applied/SceneObjects/Model/model.h
        App/Applied/SceneObjects/Model/polygonal_model.c++
        App/Applied/SceneObjects/Model/polygonal_model.h
        App/Applied/SceneObjects/Model/polygonal_model_components.cpp
        App/Applied/SceneObjects/Model/polygonal_model_components.h
        App/Applied/SceneObjects/Model/wireframe_model.cpp
        App/Applied/SceneObjects/Model/wireframe_model.h
        App/Applied/SceneObjects/Model/wireframe_model_components.cpp
        App/Applied/SceneObjects/Model/wireframe_model_components.h
        App/Applied/SceneObjects/ParticleSystem/particle.cpp
        App/Applied/SceneObjects/ParticleSystem/particle.h
        App/Applied/SceneObjects/ParticleSystem/particle_system.cpp
        App/Applied/SceneObjects/ParticleSystem/particle_system.h
        App/Applied/SceneObjects/base_object.h
        App/Applied/SceneObjects/camera.cpp
        App/Applied/SceneObjects/camera.h
        App/Applied/SceneObjects/composite.cpp
        App/Applied/SceneObjects/composite.h
        App/Applied/SceneObjects/invisible_object.h
        App/Applied/SceneObjects/light.c++
        App/Applied/SceneObjects/light.h
        App/Applied/SceneObjects/visible_object.h
        App/Applied/Solutions/load_director_solution.cpp
        App/Applied/Solutions/load_director_solution.h
        App/Applied/SPH/common/common_defs.h
        App/Applied/Visitors/base_draw_visitor.h
        App/Applied/Visitors/base_transform_visitor.h
        App/Applied/Visitors/draw_visitor.cpp
        App/Applied/Visitors/draw_visitor.h
        App/Applied/Visitors/transform_visitor.cpp
        App/Applied/Visitors/transform_visitor.h
        App/Applied/constants.h
        App/Applied/global_types.h
        App/Applied/utils.cpp
        App/Applied/utils.h
        App/Ui/Qt/Drawer/qt_drawer.cpp
        App/Ui/Qt/Drawer/qt_drawer.h
        App/Ui/Qt/Drawer/qt_drawer_factory.cpp
        App/Ui/Qt/Drawer/qt_drawer_factory.h
        App/Ui/Qt/custom_graphics_view.c++
        App/Ui/Qt/custom_graphics_view.h
        App/Ui/Qt/mainwindow.c++
        App/Ui/Qt/mainwindow.h
        App/Ui/Qt/mainwindow.ui
        App/main.cpp
        App/Ui/Qt/Drawer/thread.h
        App/Workers/sim_worker.h
        App/Workers/sim_worker.cpp
        App/Workers/gui_worker.h
        App/Workers/gui_worker.cpp
        App/Workers/main_worker.h
        App/Workers/main_worker.cpp
        )

# setup
if (${CMAKE_BUILD_TYPE} STREQUAL Debug)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O0")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -DDEBUG")
    #    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wpedantic -Wextra -Werror")
else ()
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g0 -DNDEBUG")
    set(USE_SANITIZER_FLAGS OFF)
    set(USE_COVERAGE_FLAGS OFF)
endif ()

if (USE_SANITIZER_FLAGS)
#    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fsanitize=address,undefined,leak -fno-sanitize-recover=all -fsanitize-undefined-trap-on-error")
endif ()

add_executable(${CMAKE_PROJECT_NAME} ${SRCS})

target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Qt5::Core Qt5::Gui Qt5::Widgets)

# tests
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(Test)
endif ()
