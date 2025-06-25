cmake_minimum_required(VERSION 3.8)
project(encoders_cpp_pkg)

# Стандарт C++
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Опции компиляции
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Поиск зависимостей
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED) # Для Float32MultiArray

# Поиск libgpiod с помощью PkgConfig
find_package(PkgConfig REQUIRED)
pkg_check_modules(LibGPIODCXX REQUIRED libgpiodcxx) # Для C++ биндингов libgpiod

# --- Добавление исполняемого файла ---
add_executable(encoder_node_exe src/encoder_node.cpp)

# Зависимости от других пакетов ROS 2 для нашего исполняемого файла
ament_target_dependencies(encoder_node_exe
    rclcpp
    std_msgs # Узел использует сообщения std_msgs
)

# --- Подключение заголовочных файлов ---
target_include_directories(encoder_node_exe PUBLIC
    # Для заголовочных файлов твоего пакета (например, include/encoder_node.hpp)
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include> # Для установки в install/${PROJECT_NAME}/include

    # Для заголовочных файлов libgpiodcxx
    ${LibGPIODCXX_INCLUDE_DIRS}
)

# --- Линковка с библиотеками ---
target_link_libraries(encoder_node_exe
    # Библиотеки libgpiodcxx
    ${LibGPIODCXX_LIBRARIES}
    # pthread необходим для std::thread, который используется в gpiod::line::event_wait
    # и в твоих потоках. PkgConfig для libgpiodcxx может его не включать.
    Threads::Threads # Стандартный способ подключения pthreads в CMake
)

# --- Правила установки ---
# Установка исполняемого файла
install(TARGETS
    encoder_node_exe
    DESTINATION lib/${PROJECT_NAME}
)

# Установка заголовочных файлов твоего пакета (encoder_node.hpp)
install(DIRECTORY include/
  DESTINATION include # Будет установлено в install/${PROJECT_NAME}/include/
  FILES_MATCHING PATTERN "encoder_node.hpp"
)

# --- Экспорт зависимостей ---
# Если другие пакеты будут зависеть от этого, они должны знать о rclcpp и std_msgs
ament_export_dependencies(rclcpp std_msgs)

# Завершающая команда для пакетов ament
ament_package()
