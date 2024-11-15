find_package(ufocompute QUIET)
if (NOT ufocompute_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufocompute
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocompute.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufocompute)
endif()