FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/merryMsg/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/merryMsg/msg/__init__.py"
  "../src/merryMsg/msg/_Msg.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
