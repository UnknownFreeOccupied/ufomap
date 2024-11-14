Include(FetchContent)

FetchContent_Declare(
  ufomath
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomath.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufomath)