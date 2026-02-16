CC = gcc
#CFLAGS  = -O2 -Wall -g -I/usr/local/include/modbus
CFLAGS  = -O2 -Wall -g `pkg-config --cflags libmodbus`
#LDFLAGS = -O2 -Wall -g -L/usr/local/lib -lmodbus
LDFLAGS = -O2 -Wall -g `pkg-config --libs libmodbus`

TAC = tac1100
%.o: %.c
	$(CC) -c -o $@ $< $(CFLAGS)

${TAC}: tac1100.o 
	$(CC) -o $@ tac1100.o $(LDFLAGS)
	chmod 4711 ${TAC}

strip:
	strip ${TAC}

clean:
	rm -f *.o ${TAC}

install: ${TAC}
	install -m 4711 $(TAC) /usr/local/bin

uninstall:
	rm -f /usr/local/bin/$(TAC)
