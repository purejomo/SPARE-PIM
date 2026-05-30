add_library(
  ramulator-golden
  OBJECT 
  elemax_sdpa.cpp
  tensor_utils.cpp
)

target_link_libraries(
  ramulator-golden 
  PUBLIC ramulator-base
)

target_sources(
  ramulator
  PRIVATE 
  $<TARGET_OBJECTS:ramulator-golden>
)
