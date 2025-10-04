# cmake_minimum_required(VERSION 3.16)

# project(qt_ros_robot VERSION 0.1 LANGUAGES CXX)

# set(CMAKE_AUTOUIC ON)
# set(CMAKE_AUTOMOC ON)
# set(CMAKE_AUTORCC ON)

# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
# find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

# set(PROJECT_SOURCES
#         main.cpp
#         mainwindow.cpp
#         mainwindow.h
#         mainwindow.ui
# )

# if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
#     qt_add_executable(qt_ros_robot
#         MANUAL_FINALIZATION
#         ${PROJECT_SOURCES}
#         direction_menu.cpp
#         direction_menu.h
#     )
# # Define target properties for Android with Qt 6 as:
# #    set_property(TARGET qt_ros_robot APPEND PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
# #                 ${CMAKE_CURRENT_SOURCE_DIR}/android)
# # For more information, see https://doc.qt.io/qt-6/qt-add-executable.html#target-creation
# else()
#     if(ANDROID)
#         add_library(qt_ros_robot SHARED
#             ${PROJECT_SOURCES}
#         )
# # Define properties for Android with Qt 5 after find_package() calls as:
# #    set(ANDROID_PACKAGE_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/android")
#     else()
#         add_executable(qt_ros_robot
#             ${PROJECT_SOURCES}
#         )
#     endif()
# endif()

# target_link_libraries(qt_ros_robot PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# # Qt for iOS sets MACOSX_BUNDLE_GUI_IDENTIFIER automatically since Qt 6.1.
# # If you are developing for iOS or macOS you should consider setting an
# # explicit, fixed bundle identifier manually though.
# if(${QT_VERSION} VERSION_LESS 6.1.0)
#   set(BUNDLE_ID_OPTION MACOSX_BUNDLE_GUI_IDENTIFIER com.example.qt_ros_robot)
# endif()
# set_target_properties(qt_ros_robot PROPERTIES
#     ${BUNDLE_ID_OPTION}
#     MACOSX_BUNDLE_BUNDLE_VERSION ${PROJECT_VERSION}
#     MACOSX_BUNDLE_SHORT_VERSION_STRING ${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}
#     MACOSX_BUNDLE TRUE
#     WIN32_EXECUTABLE TRUE
# )

# include(GNUInstallDirs)
# install(TARGETS qt_ros_robot
#     BUNDLE DESTINATION .
#     LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
#     RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
# )

# if(QT_VERSION_MAJOR EQUAL 6)
#     qt_finalize_executable(qt_ros_robot)
# endif()







cmake_minimum_required(VERSION 3.16)
project(qt_ros_robot VERSION 0.0.1 LANGUAGES CXX)

# ------------------------------
# ROS 2 + CMake setup
# ------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------
# Qt setup (works for Qt5 or Qt6)
# ------------------------------
find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Widgets)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Widgets)

set(PROJECT_SOURCES
    main.cpp
    mainwindow.cpp
    mainwindow.h
    mainwindow.ui
    direction_menu.cpp
    direction_menu.h
    piemenu.cpp
    piemenu.h
)

# ------------------------------
# Executable
# ------------------------------
if(${QT_VERSION_MAJOR} GREATER_EQUAL 6)
    qt_add_executable(qt_ros_robot
        MANUAL_FINALIZATION
        ${PROJECT_SOURCES}
    )
else()
    add_executable(qt_ros_robot
        ${PROJECT_SOURCES}
    )
endif()

# ------------------------------
# Linking
# ------------------------------
# target_link_libraries(qt_ros_robot PRIVATE Qt${QT_VERSION_MAJOR}::Widgets)

# ament_target_dependencies(qt_ros_robot
#     rclcpp
# )

target_link_libraries(qt_ros_robot
    PRIVATE
        Qt${QT_VERSION_MAJOR}::Widgets
        rclcpp::rclcpp
)


# ------------------------------
# Install (ROS 2 style)
# ------------------------------
install(TARGETS qt_ros_robot
    DESTINATION lib/${PROJECT_NAME}
)

# Qt finalize for Qt6
if(QT_VERSION_MAJOR EQUAL 6)
    qt_finalize_executable(qt_ros_robot)
endif()

ament_package()
