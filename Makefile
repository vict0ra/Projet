#!/usr/bin/make -f

main: main.o 
	$(CC) main.o `pkg-config --cflags --libs opencv` -o ./main

clean:
	rm -f *.o main
