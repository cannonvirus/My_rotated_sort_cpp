cmake_minimum_required(VERSION 3.5.1)
project("sort-cpp" VERSION 1.0)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
# DEBUG 시에는 사용 X, 배포때만 켜서 사용할 것
# set(CMAKE_BUILD_TYPE Release)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# add_compile_options(-Wall -std=c++14 -O2 -fPIC)

# Try to find OpenCV =============================================================================
find_package(OpenCV REQUIRED)
if (OpenCV_FOUND)
    message(STATUS "OpenCV library status:")
    message(STATUS "    version: ${OpenCV_VERSION}")
    message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")
else ()
    message(FATAL_ERROR "Could not locate OpenCV")
endif()
# Try to find OpenCV =============================================================================

include_directories(${PROJECT_SOURCE_DIR}/include)


file(GLOB SOURCE_FILES src/*.cpp)

add_library(tracker SHARED ${SOURCE_FILES})
add_library(tracker2 STATIC ${SOURCE_FILES})

set(OUTPUT_EXEC "${CMAKE_PROJECT_NAME}_app")
add_executable(${OUTPUT_EXEC} ${SOURCE_FILES})
target_link_libraries (
        ${OUTPUT_EXEC}
        ${OpenCV_LIBS}
)

install(TARGETS tracker tracker2 ${OUTPUT_EXEC}
        RUNTIME DESTINATION bin
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        PUBLIC_HEADER DESTINATION include
        )
