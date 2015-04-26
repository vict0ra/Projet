#!/usr/bin/make -f

main: main.o MyClass.o
	$(CC) main.o MyClass.o `pkg-config --cflags --libs opencv` -o ./main

MyClass.o: MyClass.cpp MyClass.h

clean:
	rm -f *.o main
