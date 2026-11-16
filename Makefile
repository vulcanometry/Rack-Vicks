# Path to the VCV Rack SDK. Override on the command line or via environment:
#   make RACK_DIR=/path/to/Rack-SDK
RACK_DIR ?= ../Rack-SDK

FLAGS +=
CFLAGS +=
CXXFLAGS +=
LDFLAGS +=

SOURCES += $(wildcard src/*.cpp)

DISTRIBUTABLES += res
DISTRIBUTABLES += $(wildcard LICENSE*)

include $(RACK_DIR)/plugin.mk
