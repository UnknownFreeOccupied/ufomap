option(UFOMAP_BUILD_DOCS     "Generate documentation" OFF)
option(UFOMAP_BUILD_TESTS    "Unit testing"           OFF)
option(UFOMAP_BUILD_COVERAGE "Test Coverage"          OFF)

add_library(Map INTERFACE)
add_library(UFO::Map ALIAS Map)

target_link_libraries(Map
	INTERFACE
		UFO::Cloud
		# UFO::Compression
		UFO::Compute
		UFO::Container
		UFO::Core
		UFO::Execution
		UFO::Geometry
		UFO::Math
		UFO::Morton
		UFO::Time
		UFO::Utility
		UFO::Vision
)

include(GNUInstallDirs)

target_include_directories(Map 
	INTERFACE
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
		$<INSTALL_INTERFACE:include>
)

if(UFO_BUILD_TESTS OR UFOMAP_BUILD_TESTS)
  add_subdirectory(tests)
endif()

if(UFO_BUILD_DOCS OR UFOMAP_BUILD_DOCS)
	add_subdirectory(docs)
endif()

install(TARGETS Map EXPORT Map-targets
	COMPONENT Map
	LIBRARY  DESTINATION lib
	ARCHIVE  DESTINATION lib
	RUNTIME  DESTINATION bin
  INCLUDES DESTINATION include
)

install(EXPORT Map-targets
  FILE "Map-targets.cmake"
  NAMESPACE UFO::
  DESTINATION lib/cmake/${PROJECT_NAME}
	COMPONENT Map
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
	"${CMAKE_CURRENT_BINARY_DIR}/Map-config-version.cmake"
	VERSION ${PROJECT_VERSION}
	COMPATIBILITY SameMajorVersion
)

configure_package_config_file(
	"${CMAKE_CURRENT_SOURCE_DIR}/cmake/Map-config.cmake.in"
	"${CMAKE_CURRENT_BINARY_DIR}/Map-config.cmake"
	INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/Map-config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/Map-config-version.cmake"
	DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}
  COMPONENT Map
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include
	COMPONENT Map
	DESTINATION ${CMAKE_INSTALL_PREFIX}
)