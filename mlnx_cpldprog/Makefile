incdir	= include
DESTDIR = $(KERNEL_SRC)/

CFLAGS =-g -Wall
DEPS = main.h utilities.h vmopcode.h jtag_handlers.h 
OBJ = jtag_handlers.o utilities.o main.o

CFLAGS += -I$(DESTDIR)$(incdir)

default: mlnx_cpldprog
    
all: mlnx_cpldprog

%.o: %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

mlnx_cpldprog: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)


clean:
	rm -rf *.o mlnx_cpldprog

