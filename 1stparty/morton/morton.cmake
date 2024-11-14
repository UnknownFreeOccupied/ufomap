Include(FetchContent)

FetchContent_Declare(
  ufomorton
  GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomorton.git
  GIT_TAG        main
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(ufomorton)