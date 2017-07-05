LIBTARGET := libviewtools.a
TARGET := test
CXXFLAGS += $(EXTRA_CXXFLAGS) -Wall -g
LDFLAGS += $(EXTRA_LDFLAGS) -L. -lviewtools

.PHONY: all clean
all: $(LIBTARGET)
clean:
	-$(RM) *.o $(TARGET) $(LIBTARGET)

$(LIBTARGET): triangulate.o filter.o scaler.o amplifier.o
	ar rcs $@ $^

%.o: %.cpp %.h
	g++ $(CXXFLAGS) -o $@ -c $<
test.o: test.cpp $(TARGET_OBJECTS:.o=.h)
	g++ $(CXXFLAGS) -o $@ -c $<
$(TARGET): test.o $(LIBTARGET)
	g++ -o $@ $(filter-out %.a,$^) $(LDFLAGS)
