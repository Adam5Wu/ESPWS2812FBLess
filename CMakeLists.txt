cmake_minimum_required(VERSION 3.10)

set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS "src/LSDriver.cpp"
                   "src/LSRenderer.cpp"
                   "src/LSTarget.cpp")

set(COMPONENT_REQUIRES esp8266 freertos)

register_component()
