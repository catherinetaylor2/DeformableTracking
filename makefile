IDIR = /usr/include
CC = clang++ -std=c++11
CFLAGS = -I$(IDIR)

ODIR = obj

LIBS = $(pkg-config opencv --cflags --libs)



_OBJ = grabcut.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

$(ODIR)/%.o: %.cpp ;
	$(CC) -c -o $@ $< $(CFLAGS)


Deform: $(OBJ) ;
	clang++ -o $@ $^ $(CFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 