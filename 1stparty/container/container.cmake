find_package(ufocontainer QUIET)
if (NOT ufocontainer_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufocontainer
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocontainer.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufocontainer)
endif()