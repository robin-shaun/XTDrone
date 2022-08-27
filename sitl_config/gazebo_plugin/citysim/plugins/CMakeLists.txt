add_definitions(${Qt5Core_DEFINITIONS})
set (CMAKE_AUTOMOC ON)

include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${Qt5Core_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

set (plugins
  LensFlareVisualPlugin
  BloomVisualPlugin
)

foreach (src ${plugins})
  add_library(${src} SHARED ${src}.cc)
  target_link_libraries(${src} ${GAZEBO_LIBRARIES})
  install (TARGETS ${src} DESTINATION ${CITYSIM_PLUGIN_INSTALL_DIR})
endforeach (src ${plugins})

set (GUIplugin
  TrafficLightsGUIPlugin
)

foreach (guiSrc ${GUIplugin})
  add_library(${guiSrc} SHARED ${guiSrc}.cc)
  target_link_libraries(${guiSrc}
                        ${GAZEBO_LIBRARIES}
                        ${Qt5Core_LIBRARIES}
                        ${Qt5Widgets_LIBRARIES} 
                        ${GAZEBO_LIBRARIES}
  )
  install (TARGETS ${guiSrc} DESTINATION ${CITYSIM_PLUGIN_INSTALL_DIR})
endforeach ()
