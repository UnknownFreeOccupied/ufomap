find_package(ufogeometry QUIET)
if (NOT ufogeometry_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufogeometry
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufogeometry.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufogeometry)
endif()