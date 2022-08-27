file(GLOB files "*.glsl")

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim-${CITYSIM_MAJOR_VERSION}/media/materials/programs)
