find_package(ufotime QUIET)
if (NOT ufotime_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufotime
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufotime.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufotime)
endif()