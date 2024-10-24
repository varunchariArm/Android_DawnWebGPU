# For more information about using CMake with Android Studio, read the
# documentation: https://d.android.com/studio/projects/add-native-code.html

cmake_minimum_required(VERSION 3.22.1)

project("dawnwebgpu")

#Set Dawn build options
option(DAWN_FETCH_DEPENDENCIES "" ON)
option(DAWN_USE_GLFW "" ON)
option(DAWN_SUPPORTS_GLFW_FOR_WINDOWING "" OFF)
option(DAWN_USE_X11 "" OFF)
option(ENABLE_PCH "" OFF)

include(utils.cmake)

add_subdirectory(webgpu)

# Creates your game shared library. The name must be the same as the
# one used for loading in your Kotlin/Java or AndroidManifest.txt files.
add_library(dawnwebgpu SHARED
        main.cpp
        webgpuRenderer.cpp
)

target_include_directories(dawnwebgpu PRIVATE .)

# Searches for a package provided by the game activity dependency
find_package(game-activity REQUIRED CONFIG)

# Configure libraries CMake uses to link your target library.
target_link_libraries(dawnwebgpu
        # The game activity
        game-activity::game-activity

        # webgpu dependency
        webgpu
        jnigraphics
        android
        log)

set_target_properties(dawnwebgpu PROPERTIES
        CXX_STANDARD 17
        VS_DEBUGGER_ENVIRONMENT "DAWN_DEBUG_BREAK_ON_ERROR=1"
)
target_treat_all_warnings_as_errors(dawnwebgpu)
target_copy_webgpu_binaries(dawnwebgpu)