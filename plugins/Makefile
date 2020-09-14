
LIB := libcar_control_plugin.so

all: $(LIB)

$(LIB):
	(cd car_control && make)
	cp car_control/*.so .

clean:
	rm -rf $(LIB)
	(cd car_control && make clean)

.PHONY: all $(LIB) clean
