Include(FetchContent)

FetchContent_Declare(
  ufocompute
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocompute.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufocompute)