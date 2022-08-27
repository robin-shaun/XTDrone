file(GLOB files "*.world")

install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/citysim-${CITYSIM_MAJOR_VERSION}/worlds/)
