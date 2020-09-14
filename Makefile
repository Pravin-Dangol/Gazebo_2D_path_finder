
all:
	(cd cmd && make)
	(cd brain && make)
	(cd plugins/car_control && make)

clean:
	(cd cmd && make clean)
	(cd brain && make clean)
	(cd plugins/car_control && make clean)

.PHONY: all clean
