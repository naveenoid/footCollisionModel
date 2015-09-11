set(modulename ${PROJECT_NAME})

file(GLOB scripts ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.template
                  ${CMAKE_CURRENT_SOURCE_DIR}/scripts/*.xml)

# yarp_install( FILES ${scripts}
#               DESTINATION ${CODYCO_APPLICATIONS_TEMPLATES_INSTALL_DIR})


