CXX = g++
CFLAGS = -O3 -fopenmp -lpthread 
OBJECTS = main.o
exec: $(OBJECTS)
	$(CXX) $(CFLAGS) main.cpp -o exec  

mcarlo: mcarlo.cpp
	$(CXX) $(CFLAGS) mcarlo.cpp -o mcarlo

clean: 
	del exec mcarlo
