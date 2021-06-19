CXX = g++-10
OBJECTS = main.o

exec : $(OBJECTS)
	$(CXX) -O3  -fopenmp -lpthread  main.cpp  ../liblbfgs-master/lib/lbfgs.c -o exec

clean:
	rm main.o exec *.svg