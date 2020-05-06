#

#DEBUG	= -g -O0
DEBUG	= -O3
CC	= gcc
INCLUDE	= -I/usr/local/include -I/usr/local/include/modbus
INCLUDE	+= -I/usr/include
CFLAGS	= $(DEBUG) -Wall $(INCLUDE) -Winline -pipe -DBUILD_FOR_RPI_RS48

LDFLAGS	= -L/usr/local/lib
LDFLAGS	= -L/usr/lib
LDLIBS    = -lwiringPi -lwiringPiDev -lpthread -lrt -lm -lcrypt -lmodbus


###############################################################################

SRC	=	master.c slave.c

OBJ	=	$(SRC:.c=.o)

BINS	=	$(SRC:.c=)

all:	$(BINS)

master:	master.o
	echo [link]
	$(CC) -o $@ master.o $(LDFLAGS) $(LDLIBS)

slave:	slave.o
	echo [link]
	$(CC) -o $@ slave.o $(LDFLAGS) $(LDLIBS)

.c.o:
	echo [CC] $<
	$(CC) -c $(CFLAGS) $< -o $@

clean:
	@echo "[Clean]"
	@rm -f $(OBJ) *~ core tags $(BINS)

tags:	$(SRC)
	@echo [ctags]
	@ctags $(SRC)

depend:
	makedepend -Y $(SRC)

# DO NOT DELETE
