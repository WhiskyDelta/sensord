# Makefile for sensord
#Some compiler stuff and flags
CFLAGS += -g -Wall
EXECUTABLE = sensord sensorcal
_OBJ = ms5611.o ams5915.o ads1110.o main.o nmea.o timer.o KalmanFilter1d.o cmdline_parser.o configfile_parser.o vario.o AirDensity.o 24c16.o mpu9150.o ukf_handler.o
_OBJ_CAL = 24c16.o ams5915.o sensorcal.o mpu9150.o
_OBJ_TEST = test.o mpu9150.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))
OBJ_CAL = $(patsubst %,$(ODIR)/%,$(_OBJ_CAL))
OBJ_TEST = $(patsubst %,$(ODIR)/%,$(_OBJ_TEST))
LIBS = -lrt -lm -L. -lcukf
ODIR = obj
BINDIR = /opt/bin/
GIT_VERSION := $(shell git describe --dirty)

#targets

$(ODIR)/%.o: %.c
	mkdir -p $(ODIR)
	$(CC) -DVERSION_GIT=\"$(GIT_VERSION)\" -c -o $@ $< $(CFLAGS)
	
all: sensord sensorcal testmpu

version.h: 
	@echo Git version $(GIT_VERSION)
	
doc: 
	@echo Running doxygen to create documentation
	doxygen
	
sensord: $(OBJ)
	$(CC) -g -o $@ $^ $(LIBS)
	
sensorcal: $(OBJ_CAL)
	$(CC) -g -o $@ $^

testmpu: $(OBJ_TEST)
	$(CC) -g -o $@ $^

install: sensord sensorcal
	install -D sensord $(BINDIR)/$(EXECUTABLE)
	
test: test.o obj/nmea.o
	$(CC) -g -o $@ $^ $(LIBS)

sensord_fastsample: sensord_fastsample.o
	$(CC) -g -o $@ $^ $(LIBS)

i2c_test: i2c_test.o ms5611.o
	$(CC) -g -o $@ $^ $(LIBS)
	
clean:
	rm -f $(ODIR)/*.o *~ core $(EXECUTABLE)
	rm -fr doc

.PHONY: clean all doc
