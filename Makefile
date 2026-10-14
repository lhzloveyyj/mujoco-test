# 仿真程序:make 编译出 sim,然后 ./sim 运行。
#
#   src/control.cpp  你的算法(平时只改这个)
#   src/robot.hpp    模型加载与步进
#   src/viewer.cpp   窗口、渲染、鼠标交互
#   src/main.cpp     入口

MJ ?= $(firstword $(wildcard $(HOME)/workspace/miniconda3/lib/python*/site-packages/mujoco))
MJLIB := $(notdir $(firstword $(wildcard $(MJ)/libmujoco.so.*)))
MODEL := $(abspath models/two_link_arm.xml)

CXX   ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra -I$(MJ)/include -DFKIK_MODEL_XML='"$(MODEL)"'
# -lglfw 是系统的 GLFW(原生窗口用);$(MJLIB) 是 mujoco 自带的 C 库
LDFLAGS  := -lglfw -L$(MJ) -l:$(MJLIB) -Wl,-rpath,$(MJ)

.PHONY: all run clean check-env

all: sim

SRC := $(wildcard src/*.cpp)
HDR := $(wildcard src/*.hpp)

sim: $(SRC) $(HDR)
	$(CXX) $(CXXFLAGS) -o $@ $(SRC) $(LDFLAGS)

run: sim
	./sim

check-env:
	@echo "MJ    = $(MJ)"
	@echo "MJLIB = $(MJLIB)"
	@echo "MODEL = $(MODEL)"
	@echo "GLFW  = $(shell ls /usr/lib/x86_64-linux-gnu/libglfw.so 2>/dev/null || echo '缺失:请装 libglfw3-dev')"

clean:
	rm -f sim
