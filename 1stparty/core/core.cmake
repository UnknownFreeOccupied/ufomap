find_package(ufocore QUIET)
if (NOT ufocore_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufocore
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocore.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufocore)
endif()