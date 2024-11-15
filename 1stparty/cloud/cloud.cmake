find_package(ufocloud QUIET)
if (NOT ufocloud_FOUND)
  Include(FetchContent)

  FetchContent_Declare(
    ufocloud
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocloud.git
    GIT_TAG        main
    GIT_PROGRESS   TRUE
  )

  FetchContent_MakeAvailable(ufocloud)
endif()