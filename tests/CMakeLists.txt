include("${PROJECT_SOURCE_DIR}/3rdparty/catch2/catch2.cmake")

# SET(GCC_COVERAGE_COMPILE_FLAGS "-fprofile-arcs -ftest-coverage")
# SET(GCC_COVERAGE_LINK_FLAGS    "--coverage")
# SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} ${GCC_COVERAGE_COMPILE_FLAGS}" )
# SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_COVERAGE_LINK_FLAGS}" )
# # set(CMAKE_CXX_OUTPUT_EXTENSION_REPLACE ON)

add_executable(ufomap_tests
	map_test.cpp
)

target_link_libraries(ufomap_tests PRIVATE UFO::Map Catch2::Catch2WithMain)

list(APPEND CMAKE_MODULE_PATH ${catch2_SOURCE_DIR}/extras)
include(CTest)
include(Catch)
catch_discover_tests(ufomap_tests)