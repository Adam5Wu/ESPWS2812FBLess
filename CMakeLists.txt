cmake_minimum_required(VERSION 3.10)

set(COMPONENT_ADD_INCLUDEDIRS include)
set(COMPONENT_SRCS "src/LSFrame.cpp"
                   "src/LSTarget.cpp"
                   "src/LSRenderer.cpp"
                   "src/LSDriver.cpp")

set(COMPONENT_REQUIRES esp8266 freertos esp_ringbuf ZWUtils-IDF8266)

register_component()
