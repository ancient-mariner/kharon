
CC = gcc

all: liblua.a 

liblua.a:
	tar zxf ../packages/lua-5.3.3.tar.gz
	cd lua-5.3.3; make linux ; make install INSTALL_TOP=$(PWD)/..

clean:
	rm -f *.o *.a 

refresh: clean all

