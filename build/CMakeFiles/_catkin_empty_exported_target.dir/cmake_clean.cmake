file(REMOVE_RECURSE
  "doc/hpp-foot.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/_catkin_empty_exported_target.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
