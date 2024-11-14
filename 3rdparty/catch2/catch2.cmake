Include(FetchContent)

FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG        8ac8190e494a381072c89f5e161b92a08d98b37b # v3.5.3
  GIT_PROGRESS   TRUE
)

FetchContent_MakeAvailable(Catch2)