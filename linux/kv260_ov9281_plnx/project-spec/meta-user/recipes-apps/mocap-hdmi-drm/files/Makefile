APP = mocap-hdmi-drm

APP_OBJS = mocap-hdmi-drm.o

CXXFLAGS += -std=c++17 -O2 -Wall -I. $(shell pkg-config --cflags libdrm)
LDLIBS += $(shell pkg-config --libs libdrm) -lpthread

all: build

build: $(APP)

$(APP): $(APP_OBJS)
	$(CXX) -o $@ $(APP_OBJS) $(LDFLAGS) $(LDLIBS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -f $(APP) *.o
