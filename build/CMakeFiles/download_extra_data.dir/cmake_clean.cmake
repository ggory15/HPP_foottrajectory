file(REMOVE_RECURSE
  "doc/hpp-foot.doxytag"
  "doc/doxygen.log"
  "doc/doxygen-html"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/download_extra_data.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
