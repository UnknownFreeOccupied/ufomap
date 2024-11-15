find_package(ufovision QUIET)
if (NOT ufovision_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufovision
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufovision.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufovision)
endif()