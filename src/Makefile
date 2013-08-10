andor3-objs = Andor3Camera.o Andor3Interface.o Andor3DetInfoCtrlObj.o Andor3SyncCtrlObj.o

SRCS = $(andor3-objs:.o=.cpp) 

CXXFLAGS += -I../include -I../../../hardware/include -I../../../common/include \
			-I/usr/local/include \
			-Wall -pthread -fPIC -g

all:	Andor3.o

Andor3.o:	$(andor3-objs)
	$(LD) -o $@ -r $+

clean:
	rm -f *.o *.P

%.o : %.cpp
	$(COMPILE.cpp) -MD $(CXXFLAGS) -o $@ $<
	@cp $*.d $*.P; \
	sed -e 's/#.*//' -e 's/^[^:]*: *//' -e 's/ *\\$$//' \
	-e '/^$$/ d' -e 's/$$/ :/' < $*.d >> $*.P; \
	rm -f $*.d

-include $(SRCS:.cpp=.P)
