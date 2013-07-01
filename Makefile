CC		=	g++
RM		=	rm
RMFLAGS	=	-f
CFLAGS	=   -o
LDFLAGS	=
DBFLAGS = 	-g

PROBTRACKERDIR = ./

SOURCES = $(PROBTRACKERDIR)CTracker.cxx \
		  ProbTracker.cxx \
		  CSource.cxx \
		  LGlbLib.cxx C2DOFKF.cxx \
		  LGeometric.cxx \
		  COxfordFeatureList.cxx \
		  CAnisDetector.cxx \
		  COpenPrj.cxx

INCLUDES = -I/usr/local/include/opencv

LIBS	 = -L/usr/local/lib -lopencv_video -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_nonfree -lopencv_features2d -lopencv_flann -lopencv_legacy -lopencv_objdetect -lopencv_calib3d

EXECUTABLE = ProbTracker

all: build run

run: 	$(EXECUTABLE)
	./$(EXECUTABLE)

build: 	$(SOURCES)
	$(CC) $(SOURCES) $(LIBS) $(CFLAGS) $(EXECUTABLE)

debug:	$(SOURCES)
	$(CC) $(DBFLAGS) $(SOURCES) $(LIBS) $(CFLAGS) $(EXECUTABLE)

clean:
	$(RM) $(EXECUTABLE) $(RMFLAGS)
