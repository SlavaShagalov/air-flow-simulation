QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

CONFIG += c++20

QMAKE_CXXFLAGS += -O2
CONFIG += console

# You can make your code fail to compile if it uses deprecated APIs.
# In order to do so, uncomment the following line.
#DEFINES += QT_DISABLE_DEPRECATED_BEFORE=0x060000    # disables all the APIs deprecated before Qt 6.0.0

SOURCES += \
	Domains/Applied/Color/color.c++ \
	Domains/Applied/Commands/CameraCommands/add_camera_command.cpp \
	Domains/Applied/Commands/CameraCommands/count_cameras_command.cpp \
	Domains/Applied/Commands/CameraCommands/load_camera_command.cpp \
	Domains/Applied/Commands/CameraCommands/move_camera_command.cpp \
	Domains/Applied/Commands/CameraCommands/remove_camera_command.cpp \
	Domains/Applied/Commands/CameraCommands/set_cur_camera_command.cpp \
	Domains/Applied/Commands/ModelCommands/add_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/count_models_command.cpp \
	Domains/Applied/Commands/ModelCommands/load_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/load_polygonal_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/move_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/remove_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/rotate_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/scale_model_command.cpp \
	Domains/Applied/Commands/ModelCommands/transform_model_command.cpp \
	Domains/Applied/Commands/SceneCommands/draw_scene_command.cpp \
	Domains/Applied/Commands/SceneCommands/get_cur_camera_command.cpp \
	Domains/Applied/Commands/SceneCommands/get_object_command.cpp \
	Domains/Applied/Commands/SceneCommands/get_scene_command.cpp \
	Domains/Applied/Containers/matrix/errors/errors.cpp \
	Domains/Applied/Containers/matrix/matrix/matrix_base.cpp \
	Domains/Applied/Drawer/solution_drawer_factory.cpp \
	Domains/Applied/Image/image.cpp \
	Domains/Applied/Load/Builders/Camera/file_camera_builder.cpp \
	Domains/Applied/Load/Builders/Model/file_polygon_model_builder.cpp \
	Domains/Applied/Load/Builders/Model/file_wireframe_model_builder.cpp \
	Domains/Applied/Load/Directors/CameraDirector/camera_load_director.cpp \
	Domains/Applied/Load/Directors/ModelDirector/polygonal_model_load_director.cpp \
	Domains/Applied/Load/Directors/ModelDirector/wireframe_model_load_director.cpp \
	Domains/Applied/Load/Loaders/CameraLoader/camera_file_loader.cpp \
	Domains/Applied/Load/Loaders/ModelLoader/polygonal_model_file_loader.cpp \
	Domains/Applied/Load/Loaders/ModelLoader/wireframe_model_file_loader.cpp \
	Domains/Applied/Managers/draw_manager.cpp \
	Domains/Applied/Managers/load_manager.cpp \
	Domains/Applied/Managers/main_manager.cpp \
	Domains/Applied/Managers/scene_manager.cpp \
	Domains/Applied/Managers/transform_manager.cpp \
	Domains/Applied/Primitives/Edge/edge.cpp \
	Domains/Applied/Primitives/Face/face.cpp \
	Domains/Applied/Scene/scene.cpp \
	Domains/Applied/SceneObjects/Model/polygonal_model.c++ \
	Domains/Applied/SceneObjects/Model/polygonal_model_components.cpp \
	Domains/Applied/SceneObjects/Model/wireframe_model.cpp \
	Domains/Applied/SceneObjects/Model/wireframe_model_components.cpp \
	Domains/Applied/SceneObjects/ParticleSystem/particle.cpp \
	Domains/Applied/SceneObjects/ParticleSystem/particle_system.cpp \
	Domains/Applied/SceneObjects/camera.cpp \
	Domains/Applied/SceneObjects/composite.cpp \
	Domains/Applied/SceneObjects/light.c++ \
	Domains/Applied/Solutions/load_director_solution.cpp \
	Domains/Applied/Visitors/draw_visitor.cpp \
	Domains/Applied/Visitors/transform_visitor.cpp \
	Domains/Applied/utils.cpp \
	Domains/Ui/Qt/Drawer/qt_drawer.cpp \
	Domains/Ui/Qt/Drawer/qt_drawer_factory.cpp \
	Domains/Ui/Qt/custom_graphics_view.c++ \
	Domains/Ui/Qt/main_window.c++ \
	Domains/main.cpp \
	debug/moc_custom_graphics_view.cpp \
	debug/moc_main_window.cpp

HEADERS += \
	Domains/Applied/Color/color.h \
	Domains/Applied/Commands/CameraCommands/add_camera_command.h \
	Domains/Applied/Commands/CameraCommands/camera_command.h \
	Domains/Applied/Commands/CameraCommands/count_cameras_command.h \
	Domains/Applied/Commands/CameraCommands/load_camera_command.h \
	Domains/Applied/Commands/CameraCommands/move_camera_command.h \
	Domains/Applied/Commands/CameraCommands/remove_camera_command.h \
	Domains/Applied/Commands/CameraCommands/rotate_camera_command.h \
	Domains/Applied/Commands/CameraCommands/set_cur_camera_command.h \
	Domains/Applied/Commands/LightCommands/add_light_command.hpp \
	Domains/Applied/Commands/ModelCommands/add_model_command.h \
	Domains/Applied/Commands/ModelCommands/count_models_command.h \
	Domains/Applied/Commands/ModelCommands/load_model_command.h \
	Domains/Applied/Commands/ModelCommands/load_polygonal_model_command.h \
	Domains/Applied/Commands/ModelCommands/model_command.h \
	Domains/Applied/Commands/ModelCommands/move_model_command.h \
	Domains/Applied/Commands/ModelCommands/remove_model_command.h \
	Domains/Applied/Commands/ModelCommands/rotate_model_command.h \
	Domains/Applied/Commands/ModelCommands/scale_model_command.h \
	Domains/Applied/Commands/ModelCommands/transform_model_command.h \
	Domains/Applied/Commands/ParticleSystemCommands/add_particle_system_command.hpp \
	Domains/Applied/Commands/ParticleSystemCommands/remove_particle_system_command.hpp \
	Domains/Applied/Commands/SceneCommands/draw_scene_command.h \
	Domains/Applied/Commands/SceneCommands/get_cur_camera_command.h \
	Domains/Applied/Commands/SceneCommands/get_object_command.h \
	Domains/Applied/Commands/SceneCommands/get_scene_command.h \
	Domains/Applied/Commands/SceneCommands/scene_command.h \
	Domains/Applied/Commands/base_command.h \
	Domains/Applied/Containers/Iterator/base_iterator.h \
	Domains/Applied/Containers/Iterator/base_iterator.hpp \
	Domains/Applied/Containers/Iterator/vector_const_iterator.h \
	Domains/Applied/Containers/Iterator/vector_const_iterator.hpp \
	Domains/Applied/Containers/Iterator/vector_iterator.h \
	Domains/Applied/Containers/Iterator/vector_iterator.hpp \
	Domains/Applied/Containers/Vector/base_vector.h \
	Domains/Applied/Containers/Vector/base_vector.hpp \
	Domains/Applied/Containers/Vector/vector.h \
	Domains/Applied/Containers/Vector/vector.hpp \
	Domains/Applied/Containers/matrix/errors/errors.hpp \
	Domains/Applied/Containers/matrix/iterator/const_matrix_iterator.h \
	Domains/Applied/Containers/matrix/iterator/const_matrix_iterator.hpp \
	Domains/Applied/Containers/matrix/iterator/matrix_iterator.h \
	Domains/Applied/Containers/matrix/iterator/matrix_iterator.hpp \
	Domains/Applied/Containers/matrix/matrix/matrix.h \
	Domains/Applied/Containers/matrix/matrix/matrix.hpp \
	Domains/Applied/Containers/matrix/matrix/matrix_base.h \
	Domains/Applied/Drawer/drawer.h \
	Domains/Applied/Drawer/solution_drawer_factory.h \
	Domains/Applied/Exceptions/base_exception.h \
	Domains/Applied/Exceptions/camera_exceptions.h \
	Domains/Applied/Exceptions/load_exceptions.h \
	Domains/Applied/Exceptions/model_exceptions.h \
	Domains/Applied/Exceptions/ui_exceptions.h \
	Domains/Applied/Facade/facade.h \
	Domains/Applied/Image/image.h \
	Domains/Applied/Load/Builders/Camera/camera_builder.h \
	Domains/Applied/Load/Builders/Camera/file_camera_builder.h \
	Domains/Applied/Load/Builders/Model/file_model_builder.h \
	Domains/Applied/Load/Builders/Model/file_polygon_model_builder.h \
	Domains/Applied/Load/Builders/Model/file_wireframe_model_builder.h \
	Domains/Applied/Load/Builders/Model/model_builder.h \
	Domains/Applied/Load/Builders/object_builder.h \
	Domains/Applied/Load/Directors/CameraDirector/camera_load_director.h \
	Domains/Applied/Load/Directors/ModelDirector/model_load_director.h \
	Domains/Applied/Load/Directors/ModelDirector/polygonal_model_load_director.h \
	Domains/Applied/Load/Directors/ModelDirector/wireframe_model_load_director.h \
	Domains/Applied/Load/Directors/base_load_director.h \
	Domains/Applied/Load/Loaders/CameraLoader/camera_file_loader.h \
	Domains/Applied/Load/Loaders/ModelLoader/polygonal_model_file_loader.h \
	Domains/Applied/Load/Loaders/ModelLoader/wireframe_model_file_loader.h \
	Domains/Applied/Load/Loaders/base_file_loader.h \
	Domains/Applied/Managers/base_manager.h \
	Domains/Applied/Managers/draw_manager.h \
	Domains/Applied/Managers/load_manager.h \
	Domains/Applied/Managers/main_manager.h \
	Domains/Applied/Managers/manager_creator.h \
	Domains/Applied/Managers/scene_manager.h \
	Domains/Applied/Managers/transform_manager.h \
	Domains/Applied/Primitives/Edge/edge.h \
	Domains/Applied/Primitives/Face/face.h \
	Domains/Applied/Primitives/Matrix4x4/matrix_4x4.hpp \
	Domains/Applied/Primitives/Vector2D/vector_2d.hpp \
	Domains/Applied/Primitives/Vector3D/vector_3d.hpp \
	Domains/Applied/Primitives/Vector4D/vector_4d.hpp \
	Domains/Applied/SPH/common/common_defs.h \
	Domains/Applied/Scene/scene.h \
	Domains/Applied/SceneObjects/Model/model.h \
	Domains/Applied/SceneObjects/Model/polygonal_model.h \
	Domains/Applied/SceneObjects/Model/polygonal_model_components.h \
	Domains/Applied/SceneObjects/Model/wireframe_model.h \
	Domains/Applied/SceneObjects/Model/wireframe_model_components.h \
	Domains/Applied/SceneObjects/ParticleSystem/particle.h \
	Domains/Applied/SceneObjects/ParticleSystem/particle_system.h \
	Domains/Applied/SceneObjects/base_object.h \
	Domains/Applied/SceneObjects/camera.h \
	Domains/Applied/SceneObjects/composite.h \
	Domains/Applied/SceneObjects/invisible_object.h \
	Domains/Applied/SceneObjects/light.h \
	Domains/Applied/SceneObjects/visible_object.h \
	Domains/Applied/Solutions/load_director_solution.h \
	Domains/Applied/Visitors/base_draw_visitor.h \
	Domains/Applied/Visitors/base_transform_visitor.h \
	Domains/Applied/Visitors/draw_visitor.h \
	Domains/Applied/Visitors/transform_visitor.h \
	Domains/Applied/global_types.h \
	Domains/Applied/utils.h \
	Domains/Ui/Qt/Drawer/qt_drawer.h \
	Domains/Ui/Qt/Drawer/qt_drawer_factory.h \
	Domains/Ui/Qt/custom_graphics_view.h \
	Domains/Ui/Qt/main_window.h \
	debug/moc_predefs.h

FORMS += \
	Domains/Ui/Qt/main_window.ui

# Default rules for deployment.
qnx: target.path = /tmp/$${TARGET}/bin
else: unix:!android: target.path = /opt/$${TARGET}/bin
!isEmpty(target.path): INSTALLS += target

DISTFILES += \
	.gitignore \
	.qmake.stash \
	Data/Models/Polygonal/Other/Airplane_Project.obj \
	Data/Models/Polygonal/Other/B206JR_obj.obj \
	Data/Models/Polygonal/Other/Su_57.obj \
	Data/Models/Polygonal/Other/Torque_Twister.obj \
	Data/Models/Polygonal/Other/WW2-Plane-LowPoly.obj \
	Data/Models/Polygonal/Other/paper_plane.obj \
	Data/Models/Polygonal/Other/plane.obj \
	Data/Models/Polygonal/Other/plane1.obj \
	Data/Models/Polygonal/Other/redkin_shell.obj \
	Data/Models/Polygonal/Other/throttle_bar2line.obj \
	Data/Models/Polygonal/Triangulated/african_head.obj \
	Data/Models/Polygonal/Triangulated/cube.obj \
	Data/Models/Polygonal/Triangulated/heart.obj \
	README.md \
	debug/add_camera_command.o \
	debug/add_model_command.o \
	debug/air-flow-simulation.exe \
	debug/camera.o \
	debug/camera_file_loader.o \
	debug/camera_load_director.o \
	debug/color.o \
	debug/composite.o \
	debug/count_cameras_command.o \
	debug/count_models_command.o \
	debug/custom_graphics_view.o \
	debug/draw_manager.o \
	debug/draw_scene_command.o \
	debug/draw_visitor.o \
	debug/edge.o \
	debug/errors.o \
	debug/face.o \
	debug/file_camera_builder.o \
	debug/file_polygon_model_builder.o \
	debug/file_wireframe_model_builder.o \
	debug/get_cur_camera_command.o \
	debug/get_object_command.o \
	debug/get_scene_command.o \
	debug/image.o \
	debug/light.o \
	debug/load_camera_command.o \
	debug/load_director_solution.o \
	debug/load_manager.o \
	debug/load_model_command.o \
	debug/load_polygonal_model_command.o \
	debug/main.o \
	debug/main_manager.o \
	debug/main_window.o \
	debug/matrix_base.o \
	debug/moc_custom_graphics_view.o \
	debug/moc_main_window.o \
	debug/move_camera_command.o \
	debug/move_model_command.o \
	debug/particle.o \
	debug/particle_system.o \
	debug/polygonal_model.o \
	debug/polygonal_model_components.o \
	debug/polygonal_model_file_loader.o \
	debug/polygonal_model_load_director.o \
	debug/qt_drawer.o \
	debug/qt_drawer_factory.o \
	debug/remove_camera_command.o \
	debug/remove_model_command.o \
	debug/rotate_model_command.o \
	debug/scale_model_command.o \
	debug/scene.o \
	debug/scene_manager.o \
	debug/set_cur_camera_command.o \
	debug/solution_drawer_factory.o \
	debug/transform_manager.o \
	debug/transform_model_command.o \
	debug/transform_visitor.o \
	debug/utils.o \
	debug/wireframe_model.o \
	debug/wireframe_model_components.o \
	debug/wireframe_model_file_loader.o \
	debug/wireframe_model_load_director.o \
	object_script.air-flow-simulation.Debug \
	object_script.air-flow-simulation.Release

