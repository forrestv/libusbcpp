all: testrift

testrift: testrift.cpp ../libusbcpp.h
	g++ -o testrift testrift.cpp `pkg-config libusb-1.0 --cflags --libs` -I../ -std=c++11 -Wall -ftrapv -ggdb
