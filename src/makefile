IDIR = ../include/
CC = clang++ -std=c++11
CFLAGS = -I$(IDIR) `pkg-config opencv --cflags`

ODIR = obj

LIBS = `pkg-config opencv --libs`



_OBJ = main.o grabcut.o distTransform.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

$(ODIR)/%.o: %.cpp ;
	$(CC) -c -o $@ $< $(CFLAGS) 


Deform: $(OBJ) ;
	clang++ -o $@ $^ $(CFLAGS) $(LIBS)

# .PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

