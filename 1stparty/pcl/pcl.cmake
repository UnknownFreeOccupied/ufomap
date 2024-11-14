Include(FetchContent)

FetchContent_Declare(
  ufopcl
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufopcl.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufopcl)