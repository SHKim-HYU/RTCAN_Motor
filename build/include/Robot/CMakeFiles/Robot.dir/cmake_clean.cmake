file(REMOVE_RECURSE
  "libRobotd.pdb"
  "libRobotd.so"
)

# Per-language clean rules from dependency scanning.
foreach(lang )
  include(CMakeFiles/Robot.dir/cmake_clean_${lang}.cmake OPTIONAL)
endforeach()
