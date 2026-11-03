cmake_minimum_required(VERSION 3.16)

project(STR400_SDK VERSION 0.0.1 LANGUAGES CXX)

# 使用 C++17。protocol.cpp 中使用了 std::chrono::ceil，需要 C++17。
set(CMAKE_CXX_STANDARD 17)
# 编译器不支持 C++17 时直接报错，不自动降级到旧标准。
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# 不启用编译器自己的 C++ 扩展，尽量使用标准 C++。
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(str400_kinematics
    src/kinematics.cpp
    src/joint_mapping.cpp
)
target_include_directories(str400_kinematics PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(str400_kinematics PRIVATE -Wall -Wextra -Wpedantic)

# Studio 只读 WebSocket 示例；可单独构建此目标，不连接机械臂串口。
option(BUILD_STUDIO_POSE_READER "Build the read-only Studio pose reader" ON)
if(BUILD_STUDIO_POSE_READER)
    find_package(Boost 1.70 QUIET COMPONENTS system)
    find_package(jsoncpp CONFIG QUIET)
    if(Boost_FOUND AND TARGET jsoncpp_lib)
        find_package(Threads REQUIRED)
        add_executable(read_studio_pose src/read_studio_pose.cpp)
        target_link_libraries(read_studio_pose PRIVATE
            Boost::system jsoncpp_lib Threads::Threads str400_kinematics)
        target_compile_options(read_studio_pose PRIVATE -Wall -Wextra -Wpedantic)
    else()
        message(STATUS "Skipping read_studio_pose: install libboost-system-dev and libjsoncpp-dev")
    endif()
endif()

# 生成一个名为 str400_demo 的可执行文件。
add_executable(str400_demo
    src/main.cpp
    src/protocol.cpp
    src/robot_reader.cpp
)

add_executable(str400_demo_enable
    "src/main_copy.cpp"
    src/protocol.cpp
    src/robot_reader.cpp
)

add_executable(str400_demo_move
    src/main_move.cpp
    src/protocol.cpp
    src/robot_reader.cpp
    src/robot_driver.cpp
)

# MOVEJ 命令行测试程序：abs 为绝对设备角，rel 为相对设备角增量。
add_executable(str400_demo_movej
    src/main_movej.cpp
    src/movej.cpp
    src/protocol.cpp
    src/robot_reader.cpp
)
target_include_directories(str400_demo_movej
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(str400_demo_movej
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(str400_demo_read_error
    src/main_read_error.cpp
    src/protocol.cpp
)
target_include_directories(str400_demo_read_error
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_compile_options(str400_demo_read_error
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

add_executable(str400_demo_movel
    src/main_movel.cpp
    src/protocol.cpp
    src/robot_reader.cpp
    src/movel.cpp
)
target_link_libraries(str400_demo_movel PRIVATE str400_kinematics)
target_compile_options(str400_demo_movel PRIVATE -Wall -Wextra -Wpedantic)

# 告诉编译器到项目的 include 目录中查找头文件。代替include_directories(include), 只给 str400_demo 这个目标添加搜索路径
target_include_directories(str400_demo
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(str400_demo_enable
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(str400_demo_move
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(str400_demo_movel
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 打开常用警告，帮助发现潜在问题。
target_compile_options(str400_demo
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

target_compile_options(str400_demo_enable
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

target_compile_options(str400_demo_move
    PRIVATE
        -Wall
        -Wextra
        -Wpedantic
)

include(CTest)
if(BUILD_TESTING)
    find_package(Threads REQUIRED)
    add_executable(protocol_receive_test
        gpt_generator/tests/protocol_receive_test.cpp
        src/protocol.cpp
        src/robot_reader.cpp
    )
    target_include_directories(protocol_receive_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_link_libraries(protocol_receive_test PRIVATE Threads::Threads)
    target_compile_options(protocol_receive_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME protocol_receive COMMAND protocol_receive_test)
    set_tests_properties(protocol_receive PROPERTIES TIMEOUT 15)

    add_executable(robot_driver_limits_test gpt_generator/tests/robot_driver_limits_test.cpp src/robot_driver.cpp)
    target_include_directories(robot_driver_limits_test PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
    target_compile_options(robot_driver_limits_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME robot_driver_limits COMMAND robot_driver_limits_test)

    add_executable(joint_mapping_test gpt_generator/tests/joint_mapping_test.cpp)
    target_link_libraries(joint_mapping_test PRIVATE str400_kinematics)
    target_compile_options(joint_mapping_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME joint_mapping COMMAND joint_mapping_test)

    add_executable(singularity_axis_test gpt_generator/tests/singularity_axis_probe.cpp)
    target_link_libraries(singularity_axis_test PRIVATE str400_kinematics)
    target_compile_options(singularity_axis_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME singularity_axis COMMAND singularity_axis_test)
    set_tests_properties(singularity_axis PROPERTIES TIMEOUT 60)

    # robot_reader 在测试中以桩函数替代，不链接串口实现。
    add_executable(movel_mapping_test
        gpt_generator/tests/movel_mapping_test.cpp
        src/movel.cpp
    )
    target_link_libraries(movel_mapping_test PRIVATE str400_kinematics)
    target_compile_options(movel_mapping_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME movel_mapping COMMAND movel_mapping_test)
    set_tests_properties(movel_mapping PROPERTIES TIMEOUT 10)

    add_executable(movel_planning_test gpt_generator/tests/movel_planning_test.cpp)
    target_link_libraries(movel_planning_test PRIVATE str400_kinematics)
    target_compile_options(movel_planning_test PRIVATE -Wall -Wextra -Wpedantic)
    add_test(NAME movel_planning COMMAND movel_planning_test)
    set_tests_properties(movel_planning PROPERTIES TIMEOUT 30)

    if(TARGET read_studio_pose)
        find_package(Python3 QUIET COMPONENTS Interpreter)
        if(Python3_Interpreter_FOUND)
            add_test(NAME studio_pose_reader
                COMMAND ${Python3_EXECUTABLE}
                    ${CMAKE_CURRENT_SOURCE_DIR}/gpt_generator/tests/studio_pose_reader_test.py
                    $<TARGET_FILE:read_studio_pose>)
            set_tests_properties(studio_pose_reader PROPERTIES TIMEOUT 20)
            find_program(STR400_STRACE_EXECUTABLE strace)
            find_program(STR400_NODE_EXECUTABLE node)
            if(STR400_STRACE_EXECUTABLE AND STR400_NODE_EXECUTABLE)
                add_test(NAME studio_capture_launcher
                    COMMAND ${Python3_EXECUTABLE}
                        ${CMAKE_CURRENT_SOURCE_DIR}/gpt_generator/tests/studio_capture_launcher_test.py
                        ${CMAKE_CURRENT_SOURCE_DIR}/gpt_generator/tools/capture_studio.sh)
                set_tests_properties(studio_capture_launcher PROPERTIES TIMEOUT 30)
            endif()
        endif()
    endif()
endif()
