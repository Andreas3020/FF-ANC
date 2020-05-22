RBFmake: RBF_time.c
	gcc -o RBFm -I/usr/include/python3.6m RBF_time.c -lpython3.6m -lpthread

RBF_TFmake: RBF_TF.c
	gcc -o RBF_TFm -I/usr/include/python3.6m RBF_TF.c -lpython3.6m -lpthread -lfftw3 -lm

run: RBFmake
	PYTHONPATH=. ./RBFm

runTF: RBF_TFmake
	PYTHONPATH=. ./RBF_TFm
