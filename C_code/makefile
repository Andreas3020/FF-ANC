RBFmake: RBFm.c
	gcc -o RBFm -I/usr/include/python3.6m RBFm.c -lpython3.6m -lpthread

RBF_TFmake: RBF_TFmLP.c
	gcc -o RBF_TFm -I/usr/include/python3.6m RBF_TFmLP.c -lpython3.6m -lpthread -lfftw3 -lm

run: RBFmake
	PYTHONPATH=. ./RBFm

runTF: RBF_TFmake
	PYTHONPATH=. ./RBF_TFm
