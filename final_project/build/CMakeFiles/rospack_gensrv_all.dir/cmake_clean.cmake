file(REMOVE_RECURSE
  "../srv_gen"
  "../srv_gen"
  "../src/final_project/srv"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/rospack_gensrv_all.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
