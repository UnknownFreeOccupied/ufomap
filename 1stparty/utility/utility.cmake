find_package(ufoutility QUIET)
if (NOT ufoutility_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufoutility
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufoutility
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufoutility)
endif()