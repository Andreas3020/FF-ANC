ANC_time_make: ANC_time.c
	gcc -o ANC_time -I/usr/include/python3.6m ANC_time.c -lpython3.6m -lpthread

ANC_TF_make: ANC_TF.c
	gcc -o ANC_TF -I/usr/include/python3.6m ANC_TF.c -lpython3.6m -lpthread -lfftw3 -lm

run: ANC_time_make
	PYTHONPATH=. ./ANC_time
runTF: ANC_TF_make
	PYTHONPATH=. ./ANC_TF
