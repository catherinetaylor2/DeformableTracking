IDIR = -I./../include/  -I/home/catherine/Documents/mc_dev/src/ -I/usr/include/pcl-1.8 -I/usr/include/vtk-6.3 -I/usr/include/boost -I/usr/include/pcl-1.8
CC = clang++ -std=c++11
CFLAGS = $(IDIR) `pkg-config opencv --cflags`  `pkg-config eigen3 --cflags --libs`
ODIR = obj

LIBS = `pkg-config opencv --libs` -L/usr/local/lib/ -L/usr/lib/x86_64-linux-gnu/ -lboost_filesystem -lboost_system -lpcl_visualization



_OBJ = main.o grabcut.o distTransform.o calibration.o pointcloud.o
OBJ = $(patsubst %, $(ODIR)/%, $(_OBJ))

$(ODIR)/%.o: %.cpp ;
	$(CC) -c -o $@ $< $(CFLAGS) 


run: $(OBJ) ;
	clang++ -o $@ $^ $(CFLAGS) $(LIBS)

# .PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ core $(INCDIR)/*~ 

