set (dirs
  actor
  city_terrain
  ocean
)

INSTALL(DIRECTORY ${dirs} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim-${CITYSIM_MAJOR_VERSION}/models)
