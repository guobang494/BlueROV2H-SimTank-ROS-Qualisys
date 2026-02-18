catkin_install_python(PROGRAMS
  scripts/sim_to_real_bridge.py
  scripts/thruster_allocator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
