PROGRAM = main
LIB = motion
CC      = g++
CFLAGS  = -Wall -std=c++11 -Wno-int-in-bool-context -Wno-misleading-indentation
LDFLAGS = -L/usr/X11R6/lib -L/usr/pkg/lib -I /usr/local/include/eigen3/
LDLIBS  = -lglut -lGLU -lGL -lm

$(PROGRAM): $(PROGRAM).o $(LIB).o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LDLIBS)

$(PROGRAM).o: $(PROGRAM).cpp $(LIB).hpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

$(LIB).o: $(LIB).cpp $(LIB).hpp
	$(CC) $(CFLAGS) $(LDFLAGS) -c -o $@ $<

.PHONY: clean dist

clean:
	-rm *.o $(PROGRAM)
