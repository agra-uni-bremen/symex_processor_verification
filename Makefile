build:
	 docker build --tag symbolic-microrv32 .
run:
	docker run --rm -it --ulimit='stack=-1:-1' -v $(shell pwd)/src:/home/klee/src symbolic-microrv32 
