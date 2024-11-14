Include(FetchContent)

FetchContent_Declare(
  ufoutility
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufoutility.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufoutility)