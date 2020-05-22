/******************************* RBF.C ***********************************
  * Authors: Shana Reynders & Andreas Dierickx
  * Titel: Active noise control for ego-noise suppression in an unmanned aerial vehicle
  * Date: 2019-2020
  -----------------------------------------------------------------------------
  * Programs purpose is to use a Reference Based Filter, to filter out the
  * propellor noise of a UAV. The program uses 2 threads and 1 main thread.
  * This implementation of the RBF is executed in the time-frequency domain.
  *
  * The first thread will continiously record the sound in the microphone
  * beneath propellors and the sound in the error microphone and write this in
  * an array. In this program this is simulated by first reading a file with
  * test data and writing this to one large array (in the main). Afterwards
  * during the first thread, it will read from this array instead of recording.
  * These read values will then be written to the recording array, which is
  * used in the second thread. When writing these values, the delay between then
  * propellers is taken into account.
  *
  * The second thread will continiously manipulate the input from the first
  * thread to produce an inverse sound which will be written to the outputarray.
  * This output is used to calculate the total error, by subtracting this
  * output from the error-microphones signal.
  * This thread will also, after the calculation is done, update the NLMS
  * coëfficients, using the inputs, error and the stepsize.
  * The whole program works in two phases. This is necessary because the
  * NLMS update can only be done after the time necessary for 1 new recording,
  * as the error recording for the predicted output is only done in the
  * second phase.
  *
  * For the schemactic overview of the code check the thesis paper
******************************************************************************/

#include <stdio.h>
#include <Python.h>
#include <numpy/ndarrayobject.h>
#include <pthread.h>
#include <time.h>
#include <fftw3.h>
#include <math.h>
#include <complex.h>

typedef struct{
    int duration;
    pthread_mutex_t* mut1r;
    pthread_mutex_t* mut2r;
    pthread_cond_t* cond11;
    pthread_cond_t* cond21;
    pthread_cond_t* cond12;
    pthread_cond_t* cond22;
    int* written1;
    int* written2;
}recordArgs;

typedef struct{
    int duration;
    float stepsize;
    pthread_mutex_t* mut1r;
    pthread_mutex_t* mut2r;
    pthread_mutex_t* mut1p;
    pthread_mutex_t* mut2p;
    pthread_cond_t* cond11;
    pthread_cond_t* cond12;
    pthread_cond_t* cond21;
    pthread_cond_t* cond22;
    int* written1;
    int* written2;
    int N_STFT;
    int R_STFT;
}calcArgs;

/*
* Definition of the static variables
* FOR FREQ OF 8820Hz
*/
#define LENGTH 8 // the delay between the two closest propellers and the error-micropohone at the certain frequency (8 for 8820Hz)
#define USED_BINS 1 // After FFT, how many bins are used (induces a low pass filter)
#define DURATION 12127 // Nr of times the loop is executing (consisting of 2 cycles each time)
#define INPUTLENGTH 248886 // length of the input recording
#define STEPSIZE 0.1
#define interpropellerDelay 2 // sample delay between the two closer propellers and the two further away

/*
* Declaration of the functions
*/
npy_float64** audioSet();
int writeFilesResult(npy_float64 output[]);
int writeFilesSpeaker(npy_float64 output[]);

void* micRecord(void* val);
void* calc(void* val);

void transformation_func(int* resultPos, npy_float64 input[LENGTH + interpropellerDelay][5], fftw_complex*** coefs, npy_float64 output[LENGTH], npy_float64 prevOutput[LENGTH], fftw_complex** prevError, fftw_complex*** p, int N_STFT, int R_STFT, int N_STFT_HALF, npy_float64* FFTin, fftw_complex* FFTout, fftw_complex* IFFTin, npy_float64* IFFTout, fftw_plan planFFT, fftw_plan planIFFT);
void nlmsUpdate(float stepsize, fftw_complex*** coefs, fftw_complex*** p, fftw_complex** prevError, int N_STFT_HALF);

void TFtransform(int N_STFT, int R_STFT, int N_STFT_HALF, npy_float64 input[N_STFT * 2], npy_float64* FFTin, fftw_complex* FFTout, fftw_plan planFFT, fftw_complex** TFdomain);
void outputTransform(int N_STFT, int R_STFT, int N_STFT_HALF, fftw_complex* IFFTin, npy_float64* IFFTout, fftw_plan planIFFT, npy_float64* output, fftw_complex** outTF);

/*
  * *********** Future work **************
  * try to implement wisdom for FFTW
  * precalculate the hann window
  *************************************************
*/

/*
* Definition of global variables
*/
npy_float64** input; // the global array from which record data is read from
npy_float64 result[DURATION*2*LENGTH]; // the global array where the results are written to, to be able to save them afterwards
npy_float64 outArray[DURATION*2*LENGTH]; // the global array where the results are written to, to be able to save them afterwards

// recording and play arrays where the inputs and outputs are written to
npy_float64 rec1[LENGTH + interpropellerDelay][5];
npy_float64 rec2[LENGTH + interpropellerDelay][5];
npy_float64 play1[LENGTH];
npy_float64 play2[LENGTH];

int main(){
  // init python imports
  Py_Initialize();
  PyEval_InitThreads();
  import_array();

  // initialize the conditions for broadcast and conditional wait
  // condx1: sends a signal from micRecord to calc that a recording is written to recx.data. Now the transformation_func can start
  // condx2: sends a signal from calc to micRecord that the NLMS update is done. A new recording can start
  pthread_cond_t c11 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond11 = &c11;
  pthread_cond_t c12 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond12 = &c12;

  pthread_cond_t c21 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond21 = &c21;
  pthread_cond_t c22 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond22 = &c22;

  // initializes the mutex for the permission to read in the recx.data and the playx.data files
  pthread_mutex_t mutex11r= PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t* mutex1r = &mutex11r;

  pthread_mutex_t mutex22r= PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t* mutex2r = &mutex22r;

  pthread_mutex_t mutex11p= PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t* mutex1p = &mutex11p;

  pthread_mutex_t mutex22p= PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t* mutex2p = &mutex22p;

  // writtenx indicates how far the recording, output generation and the output sending is at the moment
  // '0' means that it should start recording, '1' means that the recording is done and that the calculation
  // can start.
  int w1 = 0;
  int* written1 = &w1;
  int w2 = 0;
  int* written2 = &w2;

  int duration = DURATION; // how many cycles
  float stepsize = STEPSIZE; // determines the rate of convergence, but also the accuracy
  int N_STFT = 2* LENGTH; // size of each frame added to the FFT array
  int R_STFT = N_STFT*0.5; // overlap in the FFT array (50%)

  //init recx as zero
  memset( rec1, 0, 2*(LENGTH + interpropellerDelay)*sizeof(npy_float64));
  memset( rec2, 0, 2*(LENGTH + interpropellerDelay)*sizeof(npy_float64));

  // used to time how long a function takes to execute
  time_t first, second;
  time_t etime = 0;

  // get the input data from a wav file, using python code. This returns a 2D array.
  input = malloc(5*INPUTLENGTH*sizeof(npy_float64));
  input = audioSet();

  // arguments to be passed to the threads
  recordArgs* rArgs = &(recordArgs){.duration = duration, .mut1r = mutex1r, .mut2r = mutex2r, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .written1 = written1, .written2 = written2};

  calcArgs* cArgs = &(calcArgs){.duration = duration, .stepsize = stepsize, .mut1r = mutex1r, .mut2r = mutex2r, .mut1p = mutex1p, .mut2p = mutex2p, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .written1 = written1, .written2 = written2, .N_STFT = N_STFT, .R_STFT = R_STFT};

  // allows multiple python functions to run at the same time
  Py_BEGIN_ALLOW_THREADS;

  printf("thread start\n");

  first = clock();

  // start the 2 threads. One to record the signal, one to manipulate the signal
  pthread_t recordHandle;
  if( pthread_create(&recordHandle , NULL, micRecord, rArgs))exit( EXIT_FAILURE );

  pthread_t calcHandle;
  if( pthread_create(&calcHandle , NULL, calc, cArgs))exit( EXIT_FAILURE );

  // joins the threads when they are finished
  void* value;
  if(pthread_join(recordHandle, &value)==-1)
  {
      exit(EXIT_FAILURE);
      printf("could not join\n");
  }

  if(pthread_join(calcHandle, &value)==-1)
  {
      exit(EXIT_FAILURE);
      printf("could not join\n");
  }

  printf("Threads joined\n");
  Py_END_ALLOW_THREADS;

  second = clock();
  etime = second - first;
  printf("Total elapsed time in betwee total: %ld\n", etime);

  // takes the result array and writes this to a wav file using a python extension.
  // this way the result can be saved and reused for analysation
  writeFilesResult(result);
  writeFilesSpeaker(outArray);

  free(input);
  Py_Finalize();
  return 0;
}

/*
  * Function which calls a Python extension to read recorded data from a
  * .wav file. This data is returned in a **-type array (an array defined by
  * using ** instead of [][]).
*/
npy_float64** audioSet(){
  //variables init
  PyObject *pName, *pModule, *pFunc, *pArgs;
  PyObject* pValue;
  npy_float64** array;

  // import the .py file in which the to-call python function is located
  pName = PyUnicode_FromString("readFiles");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if(pModule == NULL) printf("its null\n");

  if (pModule != NULL) {
    // Get the reference to the to-call python function and checks if its callable
    pFunc = PyObject_GetAttrString(pModule, "recordSound");
    if (pFunc && PyCallable_Check(pFunc)) {

      // call the python function and return the numpy array in pValue
      pValue = PyObject_CallObject(pFunc, NULL);

      // check if the python function worked and did actually return
      if (pValue == NULL) {
        Py_DECREF(pFunc);
        Py_DECREF(pModule);
        PyErr_Print();
        fprintf(stderr,"Call failed\n");
        return NULL;
      }

      // get the type description from the array content
      PyArray_Descr *descr;
      descr = PyArray_DescrFromType(PyArray_TYPE(pValue));

      // convert the numpy array to, a by c-compiler useable format, npy_float64 array
      if (PyArray_AsCArray(&pValue, (void*)&array, PyArray_DIMS(pValue), PyArray_NDIM(pValue), descr) < 0)  {
        PyErr_SetString(PyExc_TypeError, "error converting to c array");
        return NULL;
      }
      Py_DECREF(pValue);
    }

    // if there was no such function in the .py file
    else {
        if (PyErr_Occurred())
            PyErr_Print();
        fprintf(stderr, "Cannot find function \n");
    }
    Py_XDECREF(pFunc);
    Py_DECREF(pModule);
  }

  // if there was no such .py file
  else {
    PyErr_Print();
    fprintf(stderr, "Failed to load \\n");
    return NULL;
  }

  return array;
}

/*
  * Function which calls a Python extension to write the residual error to a
  * .wav file. This way the result can be easily analysed and replayed.
*/
int writeFilesResult(npy_float64 output[]){
  // init variables
  PyObject *pName, *pModule, *pFunc;
  PyObject *pValue;
  npy_intp dims2[2] = {DURATION*LENGTH*2,1};

  // convert the, by c-compiler understandable, array to an array understandable by a python-compiler
  PyObject * pyArray = PyArray_SimpleNewFromData(2, dims2,NPY_FLOAT64, output);
  if (pyArray == NULL){
    printf("Conversion failed\n");
  }

  // import the .py file in which the to-call python function is located
  pName = PyUnicode_FromString("writeFiles_result");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if(pModule == NULL) printf("its null\n");

  if (pModule != NULL) {
      // Get the reference to the to-call python function and checks if its callable
      pFunc = PyObject_GetAttrString(pModule, "writeArray");
      if (pFunc && PyCallable_Check(pFunc)) {

        // set arguments you want to pass along to the python function
        PyObject* pArg = PyTuple_New(1);
        PyTuple_SetItem(pArg, 0, pyArray);

        // call the python function to write the resulting signal to a file
        pValue = PyObject_CallObject(pFunc, pArg);
        Py_DECREF(pArg);

        // check if array is written to a file
        if (pValue != NULL) {
            Py_DECREF(pValue);
        }

        // if the array was not correctly written
        else {
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            PyErr_Print();
            fprintf(stderr,"Call failed\n");
            return 1;
        }
      }

      // if no such function exists in the .py file
      else {
          if (PyErr_Occurred())
              PyErr_Print();
          fprintf(stderr, "Cannot find function \n");
      }
      Py_XDECREF(pFunc);
      Py_DECREF(pModule);
    }
    // if no such .py file exists
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \\n");
        return 1;
    }

  return 0;
}


/*
  * Function which calls a Python extension to write the speaker signal to a
  * .wav file. This way the result can be easily analysed and replayed.
*/
int writeFilesSpeaker(npy_float64 output[]){
  // init variables
  PyObject *pName, *pModule, *pFunc;
  PyObject *pValue;
  npy_intp dims2[2] = {DURATION*LENGTH*2,1};

  // convert the, by c-compiler understandable, array to an array understandable by a python-compiler
  PyObject * pyArray = PyArray_SimpleNewFromData(2, dims2,NPY_FLOAT64, output);
  if (pyArray == NULL){
    printf("Conversion failed\n");
  }

  // import the .py file in which the to-call python function is located
  pName = PyUnicode_FromString("writeFiles_speaker");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if(pModule == NULL) printf("its null\n");

  if (pModule != NULL) {
      // Get the reference to the to-call python function and checks if its callable
      pFunc = PyObject_GetAttrString(pModule, "writeArray");
      if (pFunc && PyCallable_Check(pFunc)) {

        // set arguments you want to pass along to the python function
        PyObject* pArg = PyTuple_New(1);
        PyTuple_SetItem(pArg, 0, pyArray);

        // call the python function to write the resulting signal to a file
        pValue = PyObject_CallObject(pFunc, pArg);
        Py_DECREF(pArg);

        // check if array is written to a file
        if (pValue != NULL) {
            Py_DECREF(pValue);
        }

        // if the array was not correctly written
        else {
            Py_DECREF(pFunc);
            Py_DECREF(pModule);
            PyErr_Print();
            fprintf(stderr,"Call failed\n");
            return 1;
        }
      }

      // if no such function exists in the .py file
      else {
          if (PyErr_Occurred())
              PyErr_Print();
          fprintf(stderr, "Cannot find function \n");
      }
      Py_XDECREF(pFunc);
      Py_DECREF(pModule);
    }
    // if no such .py file exists
    else {
        PyErr_Print();
        fprintf(stderr, "Failed to load \\n");
        return 1;
    }

  return 0;
}

/*
  * Continiously reads the sound of the microphones beneath propellors and the
  * error microphone from the input array which contains the data of a
  * recording in a soundfile. Afterwards this is written the recx array
*/
void* micRecord(void* rArgs){
  printf("in micRecord\n\n");

  // initialize all variables
  //separate rArgs into all its variables
  recordArgs args = *(recordArgs*)rArgs;
    int duration = args.duration;
    int* written1 = args.written1;
    int* written2 = args.written2;
    pthread_mutex_t* mutex1r = args.mut1r;
    pthread_mutex_t* mutex2r = args.mut2r;

    int pos = 0; // keeps track of where the next data sample should be read from in the input array

    // used to determine the time each funtion takes to execute
    time_t etime = 0;
    time_t first, second;

  // loop through the 2 phases of the reading
  for(int i = 0; i<duration; i++){

    // write the input to the rec1[][] array
    pthread_mutex_lock(mutex1r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written1 != 0){
        pthread_cond_wait(args.cond12, mutex1r);
      }
      first = clock();

      // loops through the rec array to put in new inputs. Each propeller has
      // its own collum in the rec array. Two propellers have a larger delay
      // than the other two. Those are inserted in a delayed position.
      for(int x = 0; x < LENGTH; x++, pos++){

        // if the delay between the two furthes propellers and the closest ones
        // is zero
        if(interpropellerDelay == 0){
          rec1[x][0] = input[pos][0];
          rec1[x][1] = input[pos][1];
          rec1[x][2] = input[pos][2];
          rec1[x][3] = input[pos][3];
        }
        else{
          // if the delayed position is larger than length it is passed onto the
          // next cycle to be used there. Here the passed on values are added
          if((LENGTH + x) < (LENGTH + interpropellerDelay)){
            rec1[x][2] = rec2[LENGTH + x][2];
            rec1[x][3] = rec2[LENGTH + x][3];
          }

          // add the first two propeller recordings
          rec1[x][0] = input[pos][0];
          rec1[x][1] = input[pos][1];

          // add the recordings of the two propellers further away
          rec1[x + interpropellerDelay][2] = input[pos][2];
          rec1[x + interpropellerDelay][3] = input[pos][3];
        }
        // insert the error microphone recording
        rec1[x][4] = input[pos][4];
      }

      // If the interpropellerDelay is larger than length it does not suffice
      // to use the passed on values once as there are more passed on values
      // than input positions to insert them in. Therefore a shift is necessary
      if(interpropellerDelay > LENGTH){
        for(int y = LENGTH; y < (interpropellerDelay - LENGTH); y++){
          rec1[y][0] = rec2[y + LENGTH][0];
          rec1[y][1] = rec2[y + LENGTH][1];
          rec1[y][2] = rec2[y + LENGTH][2];
          rec1[y][3] = rec2[y + LENGTH][3];
        }
      }

      *written1 = 1; // set written to 1 to indicate that the next transformation_func can start
    pthread_mutex_unlock(mutex1r);
    pthread_cond_broadcast(args.cond11); // broadcast that the first cycle input is written to the array and the transformation_func can start

    // write the input array to the rec2[][] array
    pthread_mutex_lock(mutex2r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written2 != 0){
        pthread_cond_wait(args.cond22, mutex2r);
      }
      second = clock();
      etime += second - first;

      // loops through the rec array to put in new inputs. Each propeller has
      // its own collum in the rec array. Two propellers have a larger delay
      // than the other two. Those are inserted in a delayed position.
      for(int x = 0; x < LENGTH; x++, pos++){

        // if the delay between the two furthes propellers and the closest ones
        // is zero
        if(interpropellerDelay == 0){
          rec2[x][0] = input[pos][0];
          rec2[x][1] = input[pos][1];
          rec2[x][2] = input[pos][2];
          rec2[x][3] = input[pos][3];
        }
        else{
          // if the delayed position is larger than length it is passed onto the
          // next cycle to be used there. Here the passed on values are added
          if((LENGTH + x) < (LENGTH + interpropellerDelay)){
            rec2[x][2] = rec1[LENGTH + x][2];
            rec2[x][3] = rec1[LENGTH + x][3];
          }

          // add the first two propeller recordings
          rec2[x][0] = input[pos][0];
          rec2[x][1] = input[pos][1];

          // add the recordings of the two propellers further away
          rec2[x + interpropellerDelay][2] = input[pos][2];
          rec2[x + interpropellerDelay][3] = input[pos][3];
        }

        // insert the error microphone recording
        rec2[x][4] = input[pos][4];
      }

      // If the interpropellerDelay is larger than length it does not suffice
      // to use the passed on values once as there are more passed on values
      // than input positions to insert them in. Therefore a shift is necessary
      if(interpropellerDelay > LENGTH){
        for(int y = LENGTH; y < (interpropellerDelay - LENGTH); y++){
          rec2[y][0] = rec1[y + LENGTH][0];
          rec2[y][1] = rec1[y + LENGTH][1];
          rec2[y][2] = rec1[y + LENGTH][2];
          rec2[y][3] = rec1[y + LENGTH][3];
        }
      }

      // put the error-microphone recording in
      *written2 = 1; // set written to 1 to indicate that the next transformation_func can start
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond21); // broadcast that the second cycle input is written to the array and the transformation_func can start
  }

  printf("mic duration: %d\telapsed time in betwee total: %ld\taverage time microsec: %ld\taverage lost samples: %f\n", duration, etime, etime/(duration), (double)etime/(duration*113));
  printf("micRecord thread done\n");
  pthread_exit(NULL);
}

/*
  * Continiously manipulates the input from the first thread to produce an
  * inverse sound, using the transformation_func principle which will be written to the playx
  * array. This function will also, after the transformation_func is done, update the NLMS
  * coëfficients.
*/
void* calc(void* cArgs){
  printf("in calc\n\n");

  // initialize all variables
  //separate cArgs into all its variables
  calcArgs args = *(calcArgs*)cArgs;
    int duration = args.duration;
    float stepsize = args.stepsize;
    pthread_mutex_t* mutex1r = args.mut1r;
    pthread_mutex_t* mutex2r = args.mut2r;
    pthread_mutex_t* mutex1p = args.mut1p;
    pthread_mutex_t* mutex2p = args.mut2p;
    int* written1 = args.written1;
    int* written2 = args.written2;
    int N_STFT = args.N_STFT;
    int R_STFT = args.R_STFT;
    int N_STFT_HALF = N_STFT * 0.5 +1; // size of Time-frequency array

    // keeps track of where the next postion is to write a value to in the result array
    int resultPos = 0;

    // init the coefficients to random values between +0.5 and -0.5 for four propellers
    fftw_complex*** coefs = (fftw_complex***) malloc(4 * sizeof(fftw_complex*));
    for(int i = 0; i < 4; i++){
      coefs[i] = (fftw_complex**) malloc(2 * sizeof(fftw_complex));
      for(int k = 0; k < 2; k++){
        coefs[i][k] = (fftw_complex*) malloc(USED_BINS * sizeof(fftw_complex));
        for(int j = 0; j < USED_BINS; j++){
          coefs[i][k][j][0] = ((float)rand()/((float)RAND_MAX)) - 0.5;
          coefs[i][k][j][1] = ((float)rand()/((float)RAND_MAX)) - 0.5;
        }
      }
    }

    // init the prevError and p  array in the Time-frequency domain (complex), used for the NLMS update
    fftw_complex** prevError = (fftw_complex**)fftw_malloc(2 * sizeof(fftw_complex));
    prevError[0] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
    memset(prevError[0], 0, N_STFT_HALF* sizeof(npy_float64));
    prevError[1] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
    memset(prevError[1], 0, N_STFT_HALF* sizeof(npy_float64));

    fftw_complex*** p1 = (fftw_complex***) fftw_malloc(4 * sizeof(fftw_complex));
    fftw_complex*** p2 = (fftw_complex***) fftw_malloc(4 * sizeof(fftw_complex));
    for(int i = 0; i < 4; i++){
      p1[i] = (fftw_complex**) malloc(2 * sizeof(fftw_complex));
      p2[i] = (fftw_complex**) malloc(2 * sizeof(fftw_complex));
      for(int k = 0; k < 2; k++){
        p1[i][k] = (fftw_complex*) malloc(USED_BINS * sizeof(fftw_complex));
        p2[i][k] = (fftw_complex*) malloc(USED_BINS * sizeof(fftw_complex));
        memset( p1[i][k], 0, USED_BINS * sizeof(fftw_complex));
        memset( p2[i][k], 0, USED_BINS * sizeof(fftw_complex));
      }
    }

    // init the arrays for the transformation to the TF-domain
    npy_float64 *FFTin, *IFFTout;
    fftw_complex *FFTout, *IFFTin;
    fftw_plan planFFT, planIFFT;

    FFTin = (npy_float64*) fftw_malloc(N_STFT * sizeof(npy_float64));
    FFTout = (fftw_complex*) fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
    IFFTin = (fftw_complex*) fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
    IFFTout = (npy_float64*) fftw_malloc(N_STFT * sizeof(npy_float64));

    // init the plans for the FFT
    planFFT = fftw_plan_dft_r2c_1d(N_STFT, FFTin, FFTout, FFTW_MEASURE);
    planIFFT = fftw_plan_dft_c2r_1d(N_STFT, IFFTin, IFFTout, FFTW_MEASURE);

    int outPos = 0;
  // loops through the transformation_func and the NLMS of the two phases
  for(int i = 0; i< duration; i++){

    // Calculate the current output signal and the previous error.
    // Also insert the previous inputs in prevInputs.
    pthread_mutex_lock(mutex1r);
      // checks if the new input is already written to rec1, if not wait for the broadcast
      if(*written1 != 1){
        pthread_cond_wait(args.cond11, mutex1r);
      }

      // calls transformation_func
      pthread_mutex_lock(mutex1p);
      transformation_func(&resultPos, rec1, coefs, play1, play2, prevError, p1, N_STFT, R_STFT, N_STFT_HALF, FFTin, FFTout, IFFTin, IFFTout, planFFT, planIFFT);
      *written1 = 0; // indicate that the transformation_func is finished

      pthread_mutex_unlock(mutex1p);
    pthread_mutex_unlock(mutex1r);
    pthread_cond_broadcast(args.cond12); // if the next input may be read

    // Calculate the current output signal and the previous error.
    // Also insert the previous inputs in prevInputs.
    pthread_mutex_lock(mutex2r);
      // checks if the new input is already written to rec2, if not wait for the broadcast
      if(*written2 != 1){
        pthread_cond_wait(args.cond21, mutex2r);
      }

      // calls transformation_func
      pthread_mutex_lock(mutex2p);
      transformation_func(&resultPos, rec2, coefs, play2, play1, prevError, p2, N_STFT, R_STFT, N_STFT_HALF, FFTin, FFTout, IFFTin, IFFTout, planFFT, planIFFT);

      *written2 = 0; // indicate that the FIR is finished

      pthread_mutex_unlock(mutex2p);
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond22);

    // calls the update function to update the coefficients
    nlmsUpdate(stepsize, coefs, p1, prevError, N_STFT_HALF);
  }

  // free all the allocated memory blocks
  fftw_free(FFTin);
  fftw_free(FFTout);
  fftw_free(IFFTin);
  fftw_free(IFFTout);
  fftw_destroy_plan(planFFT);
  fftw_destroy_plan(planIFFT);

  for(int i = 0; i < 4; i++){
    for(int k = 0; k < 2; k++){
      fftw_free(p1[i][k]);
      fftw_free(p2[i][k]);
      fftw_free(coefs[i][k]);
    }
    fftw_free(p1[i]);
    fftw_free(p2[i]);
    fftw_free(coefs[i]);
  }

  for(int k = 0; k < 2; k++){
    fftw_free(prevError[k]);
  }

  fftw_free(p1);
  fftw_free(p2);
  fftw_free(coefs);
  fftw_free(prevError);

  printf("calc thread done\n");
  pthread_exit(NULL);
}

/*
  * This function is responsible for the calculation of the new output. This is
  * Done by transforming the inputs to the TF domain and multiplying them
  * with their corresponding coefficients. Afterwards this is converted back to
  * the time domain.
  * During transformation_func the previous error is also calculated and the
  * previous inputs are written to the prevInputs array to be used in NLMS
*/
void transformation_func(int* resultPos, npy_float64 input[LENGTH + interpropellerDelay][5], fftw_complex*** coefs, npy_float64 output[LENGTH], npy_float64 prevOutput[LENGTH], fftw_complex** prevError, fftw_complex*** p, int N_STFT, int R_STFT, int N_STFT_HALF, npy_float64* FFTin, fftw_complex* FFTout, fftw_complex* IFFTin, npy_float64* IFFTout, fftw_plan planFFT, fftw_plan planIFFT){
  // init the variables
  npy_float64* norm = malloc(USED_BINS * sizeof(npy_float64)); // norm used in calculation of p

  // Error array in the time domain
  npy_float64* Error = malloc(N_STFT * 2 * sizeof(npy_float64));
  memset(Error, 0, N_STFT * 2 * sizeof(npy_float64));

  // inputs in the Time-frequency domain
  fftw_complex** TFdomain = (fftw_complex**)fftw_malloc(2 * sizeof(fftw_complex));
  TFdomain[0] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
  memset(TFdomain[0], 0, N_STFT_HALF* sizeof(npy_float64));
  TFdomain[1] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
  memset(TFdomain[1], 0, N_STFT_HALF* sizeof(npy_float64));

  // output in the time-frequency domain
  fftw_complex** outTF = (fftw_complex**)fftw_malloc(2 * sizeof(fftw_complex));
  outTF[0] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
  memset(outTF[0], 0, N_STFT_HALF* sizeof(npy_float64));
  outTF[1] = (fftw_complex*)fftw_malloc(N_STFT_HALF * sizeof(fftw_complex));
  memset(outTF[1], 0, N_STFT_HALF* sizeof(npy_float64));

  // loops through the whole input array and calculates
  // the previous error which is used in the NLMS update.
  for(int x = 0; x < LENGTH; x++, (*resultPos)++){

    // subtract the last calculated outputs from the samples of the
    // error-microphone of the new input array. The previous calculated
    // output is the one that predicts the current error microphone and thus
    // this gives the total error.
    Error[x + (N_STFT - R_STFT)] = input[x][4] - prevOutput[x];
    result[*resultPos] = Error[x + (N_STFT - R_STFT)];
    outArray[*resultPos] = prevOutput[x];
  }

  // Transforms the error array to the Time-frequency domain
  TFtransform(N_STFT, R_STFT, N_STFT_HALF, Error, FFTin, FFTout, planFFT, TFdomain);

  for(int k = 0; k < 2; k++){
    for(int i = 0; i < (N_STFT_HALF); i++){
      prevError[k][i][0] = TFdomain[k][i][0];
      prevError[k][i][1] = TFdomain[k][i][1];
    }
  }

  // loops through the input array of each propeller and converts it to the TF domain.
  // Afterwards it is multiplied with with their coefficients and the
  // results of each propeller is added togheter. In the meantime p is calculated.
  for(int i = 0; i < 4; i++){
    // insert the inputs in an array which can be converted to the TF domain.
    // this array is zeropadded at the end the beginning
    npy_float64* tInput = malloc((N_STFT - R_STFT) * 3 * sizeof(npy_float64));
    memset(tInput, 0,(N_STFT - R_STFT) * 3 * sizeof(npy_float64));

    for(int j = 0; j < LENGTH; j++){
      tInput[j + N_STFT - R_STFT] = input[j][i];
    }

    // Transforms the input array into the Time-frequency domain
    TFtransform(N_STFT, R_STFT, N_STFT_HALF, tInput, FFTin, FFTout, planFFT, TFdomain);
    // set the norm to zero again
    memset(norm, 0, USED_BINS * sizeof(npy_float64));

    // multiply the coefficients with the TF domain
    // Also calculate the norm of the propeller in the TF domain
    for(int k = 0; k < 2; k++){
      for(int j = 0; j < USED_BINS; j++){
        outTF[k][j][0] += TFdomain[k][j][0]*coefs[i][k][j][0];
        outTF[k][j][1] += TFdomain[k][j][1]*coefs[i][k][j][1];
        norm[j] += TFdomain[k][j][0] * TFdomain[k][j][0] + TFdomain[k][j][1] * TFdomain[k][j][1];
      }
    }
      // calculate the p array using the norm of the propeller input
    for(int k = 0; k < 2; k++){
      for(int j = 0; j < USED_BINS; j++){
        p[i][k][j][0] = TFdomain[k][j][0]/(norm[j]+1);
        p[i][k][j][1] = TFdomain[k][j][1]/(norm[j]+1);
      }
    }
  }

  // Ttransforms the output array (IFFTin) to back to the time domain
  outputTransform(N_STFT, R_STFT, N_STFT_HALF, IFFTin, IFFTout, planIFFT, output, outTF);

  // free all the malloced memory blocks
  free(norm);
  free(Error);

  for(int k = 0; k < 2; k++){
    fftw_free(TFdomain[k]);
    fftw_free(outTF[k]);
  }
  fftw_free(TFdomain);
  fftw_free(outTF);
}

/*
  * Updates the filter coëfficients using the input data (in the form of the p array), the stepsize and the error value.
  * The error value is in the time frequency domain and is already calculated in the last executed transformation_func.
*/
void nlmsUpdate(float stepsize, fftw_complex*** coefs, fftw_complex*** p, fftw_complex** prevError, int N_STFT_HALF){
  // There is one error for each frequency bin
  // The new coefficients is the old one added to a multiplication of the frequency bin error, the stepsize and the sum of the bins p values
  for(int i = 0; i < 4; i++){
    for(int k = 0; k < 2; k++){
      for(int j = 0; j < USED_BINS; j++){
        coefs[i][k][j][0] += p[i][k][j][0] * prevError[k][j][0] * stepsize;
        coefs[i][k][j][1] += p[i][k][j][1] * prevError[k][j][1] * stepsize;
      }
    }
  }
}

/*
  * Transforms the an array to the Time-frequency domain
*/
void TFtransform(int N_STFT, int R_STFT, int N_STFT_HALF, npy_float64 input[2*N_STFT], npy_float64* FFTin, fftw_complex* FFTout, fftw_plan planFFT, fftw_complex** TFdomain){
  // init the variables
  npy_float64* x_frame = malloc(N_STFT * sizeof(npy_float64));

  // loop through the input array using 50% overlap (2 + 0.5*2 = 3)
  for(int i = 0; i < 2; i++){
    // add frames using the 50% overlap and fit them behind each other in FFTin
    for(int j = 0; j < N_STFT; j++){
      x_frame[j] = input[i*(N_STFT - R_STFT) + j];
    }
    // implement squared Hann window on each frame
    for(int j = 0; j < N_STFT; j++) {
      npy_float64 multiplier = sqrt(0.5 * (1 - cos(2*M_PI*j/N_STFT)));
      FFTin[j] = multiplier * x_frame[j];
    }

    // execute the fft
    fftw_execute(planFFT);

    // write away to the TFdomain array
    for(int j = 0; j < N_STFT_HALF; j++){
      TFdomain[i][j][0] = FFTout[j][0];
      TFdomain[i][j][1] = FFTout[j][1];
    }
  }
  // free the malloced memory blocks
  free(x_frame);
}

/*
  * Ttransforms the output array to back to the time domain
*/
void outputTransform(int N_STFT, int R_STFT, int N_STFT_HALF, fftw_complex* IFFTin, npy_float64* IFFTout, fftw_plan planIFFT, npy_float64* output, fftw_complex** outTF){
  // set output back to zero
  memset(output, 0, N_STFT*sizeof(npy_float64));

  // loop through the outTF array
  for(int k = 0; k < 2; k++){
    // set IFFTin on zero
    memset(IFFTin, 0, N_STFT_HALF * sizeof(fftw_complex));

    // write the values to the IFFT array
    for(int j = 0; j < USED_BINS; j++){
      IFFTin[j][0] = outTF[k][j][0];
      IFFTin[j][1] = outTF[k][j][1];
    }

    // execute the IFFT on the output array
    fftw_execute(planIFFT);

    // sum and add the output values and recuperate the output using the window
    for(int i = 0; i < N_STFT; i++){
      if(k == 0 && i >= R_STFT){
        npy_float64 multiplier = sqrt(0.5 * (1 - cos(2*M_PI*i/N_STFT)));
        output[i - R_STFT] += multiplier * IFFTout[i]/N_STFT;
      }
      else if(k == 1 && i < R_STFT){
        npy_float64 multiplier = sqrt(0.5 * (1 - cos(2*M_PI*i/N_STFT)));
        output[i] += multiplier * IFFTout[i]/N_STFT;
      }
    }
  }
}
