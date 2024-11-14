Include(FetchContent)

FetchContent_Declare(
  ufocompression
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufocompression.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufocompression)