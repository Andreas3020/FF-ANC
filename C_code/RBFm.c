/******************************* RBFm.C ***********************************
  * Authors: Shana Reynders & Andreas Dierickx
  * Titel: Active noise control for ego-noise suppression in an unmanned aerial vehicle
  * Date: 2019-2020
  -----------------------------------------------------------------------------
  * Programs purpose is to use a Reference Based Filter (RBF), to filter out the
  * propellor noise of a UAV. The program uses 2 threads and 1 main thread.
  * This implementation of the RBF is executed in the time domain.
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
  * thread to produce an inverse sound which will be written to the inputBuffer.
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
}calcArgs;

/*
* Definition of the static variables
* FOR FREQ OF 8820Hz
*/
#define DELAY 8 // the delay between the closest propeller at 8820Hz is 8 samples
#define LENGTH  1 // Indicates how many input samples are recorded each loop and has to be a natural divider of DELAY
#define NROFCOEFS 15
#define DURATION 97016 // Nr of times the loop is executing (consisting of 2 cycles each time)
#define INPUTLENGTH 248886 // length of the input recording
#define STEPSIZE 4.5
#define interpropellerDelay 2 // sample delay between the two closer propellers and the two further away


/*
* Declaration of the functions
*/
npy_float64** audioSet();
int writeFilesResult(npy_float64 output[]);
int writeFilesSpeaker(npy_float64 output[]);

void* micRecord(void* val);
void* calc(void* val);

void fir(int* insertPos, int* bufferPos, npy_float64 input[LENGTH + interpropellerDelay][5], npy_float64 firArray[NROFCOEFS][4], npy_float64 coefs[NROFCOEFS][4], npy_float64 play[LENGTH], npy_float64 inputBuffer[DELAY][4], npy_float64 prevPlay[LENGTH], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError[LENGTH]);
void nlmsUpdate(int insertPos, float stepsize, npy_float64 coefs[NROFCOEFS][4], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError);

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
  // condx1: sends a signal from micRecord to calc that a recording is written to recx.data. Now the fir can start
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

  int duration = DURATION; // how many loops
  float stepsize = STEPSIZE; // determines the rate of convergence, but also the stability

  //init recx as zero
  memset( rec1, 0, 2*(LENGTH + interpropellerDelay)*sizeof(npy_float64));
  memset( rec2, 0, 2*(LENGTH + interpropellerDelay)*sizeof(npy_float64));

  // used to time how long a function takes to execute
  time_t first, second;
  time_t etime = 0;

  // get the input data from a .wav file, using python code. This returns a 2D array.
  input = malloc(5*INPUTLENGTH*sizeof(npy_float64));
  input = audioSet();

  // arguments to be passed to the threads
  recordArgs* rArgs = &(recordArgs){.duration = duration, .mut1r = mutex1r, .mut2r = mutex2r, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .written1 = written1, .written2 = written2};

  calcArgs* cArgs = &(calcArgs){.duration = duration, .stepsize = stepsize, .mut1r = mutex1r, .mut2r = mutex2r, .mut1p = mutex1p, .mut2p = mutex2p, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .written1 = written1, .written2 = written2};

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
    first = clock();
  // loop through the 2 cycles of the reading for duration times
  for(int i = 0; i<duration; i++){

    // write the input to the rec1[][] array
    pthread_mutex_lock(mutex1r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written1 != 0){
        pthread_cond_wait(args.cond12, mutex1r);
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

      *written1 = 1; // set written to 1 to indicate that the next FIR can start
    pthread_mutex_unlock(mutex1r);
    pthread_cond_broadcast(args.cond11); // broadcast that the first cycle input is written to the array and the FIR can start

    // write the input array to the rec2[][] array
    pthread_mutex_lock(mutex2r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written2 != 0){
        pthread_cond_wait(args.cond22, mutex2r);
      }
      first = clock();

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
      *written2 = 1; // set written to 1 to indicate that the next FIR can start
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond21); // broadcast that the second cycle input is written to the array and the FIR can start
  }

  printf("duration: %d\telapsed time in betwee total: %ld\taverage time microsec: %ld\taverage lost samples: %f\n", duration, etime, etime/(duration), (double)etime/(duration*454));
  printf("micRecord thread done\n");
  pthread_exit(NULL);
}

/*
  * Continiously manipulates the input from the first thread to produce an
  * inverse sound, using the FIR principle which will be written to the playx
  * array. This function will also, after the FIR is done, update the NLMS
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

    // init the coefficients to random values between +0.5 and -0.5 for four propellers
    npy_float64 coefs[NROFCOEFS][4];
    for(int i = 0; i < NROFCOEFS; i++){
      for(int j = 0; j < 4; j++){
        coefs[i][j] = ((float)rand()/((float)RAND_MAX)) - 0.5;
      }
    }

    // init the FIR array with zero's and thus also doing zero-padding
    npy_float64 firArray[NROFCOEFS][4];
    memset( firArray, 0, 4*NROFCOEFS*sizeof(npy_float64));

    // init the prevInputs to zero, used for the NLMS update
    npy_float64 prevInputs[NROFCOEFS][4];
    memset( prevInputs, 0, 4*NROFCOEFS*sizeof(npy_float64));

    // init the prevError, used for the NLMS update
    npy_float64 prevError[LENGTH];
    memset( prevError, 0, LENGTH*sizeof(npy_float64));

    // init the inputBuffer, used to keep track of next outputs
    npy_float64 inputBuffer[DELAY][4];
    memset( inputBuffer, 0, DELAY*4*sizeof(npy_float64));

    int insertPos = 0; // keeps track of where the next postion is to write a value to, in the FIR array
    int resultPos = 0; // keeps track of where the next postion is to write a value to, in the result array
    int bufferPos = 0; // keeps track of where the next postion is to write a value to, in the input buffer

  // loops through the FIR and the NLMS of the two cycles
  for(int i = 0; i< duration; i++){

    // Insert rec1[][] into the FIR array and calculate the current output signal
    // Also calculate the previous error and insert the previous inputs in prevInputs
    pthread_mutex_lock(mutex1r);

      // checks if the new input is already written to rec1, if not wait for the broadcast
      if(*written1 != 1){
        pthread_cond_wait(args.cond11, mutex1r);
      }

      // calls the FIR function
      pthread_mutex_lock(mutex1p);
      fir(&insertPos, &bufferPos, rec1, firArray, coefs, play1, inputBuffer, play2, prevInputs, prevError);

      // write the residual error to the result array for further analysis
      for(int j = 0; j < LENGTH; j++, resultPos++){
        result[resultPos] = prevError[j];
        outArray[resultPos] = play2[j];
      }

      *written1 = 0; // indicate that the FIR is finished

      pthread_mutex_unlock(mutex1p);
    pthread_mutex_unlock(mutex1r);
    pthread_cond_broadcast(args.cond12); // if the next input may be read

    // Insert rec1[][] into the FIR array and calculate the current output signal
    // Also calculate the previous error and insert the previous inputs in prevInputs
    pthread_mutex_lock(mutex2r);

      // checks if the new input is already written to rec2, if not wait for the broadcast
      if(*written2 != 1){
        pthread_cond_wait(args.cond21, mutex2r);
      }

      pthread_mutex_lock(mutex2p);
      // calls the FIR function
      fir(&insertPos, &bufferPos, rec2, firArray, coefs, play2, inputBuffer, play1, prevInputs, prevError);
      for(int j = 0; j < LENGTH; j++, resultPos++){
        result[resultPos] = prevError[j];
        outArray[resultPos] = play1[j];
      }

      *written2 = 0; // indicate that the FIR is finished

      pthread_mutex_unlock(mutex2p);
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond22);

    // calls the update function to update the coefficients
    nlmsUpdate(insertPos, stepsize, coefs, prevInputs, prevError[LENGTH-1]);
  }

  printf("calc thread done\n");
  pthread_exit(NULL);
}

/*
  * Writes the data one by one from the input array in the firArray. Then it
  * multiplies this with the coef array which contains the filter coëfficients.
  * The multiplication is summed and put in the output array. This is done for
  * every element of the input array.
  * During the FIR-algorithm, the previous error is also calculated and the
  * previous inputs are written to the prevInputs array to be used in NLMS update
*/
void fir(int* insertPos, int* bufferPos, npy_float64 input[LENGTH + interpropellerDelay][5], npy_float64 firArray[NROFCOEFS][4], npy_float64 coefs[NROFCOEFS][4], npy_float64 play[LENGTH], npy_float64 inputBuffer[DELAY][4], npy_float64 prevPlay[LENGTH], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError[LENGTH]){
  // loops through the whole input array, puts it in the rigth position in the FIR array
  // and generates the output. In the meantime it also updates the prevInputs array and
  // calculates the previous error, used in the NLMS update.
  for(int x = 0; x < LENGTH; x++, (*insertPos)++, (*bufferPos)++){

    // insertPos keeps track of where the last insert was to create a buffer similar to a circular buffer
    if(*insertPos >= NROFCOEFS){
      *insertPos = 0;
    }

    // bufferPos keeps track of where the last insert was to create a buffer
    // which holds the input values untill they are needed
    if((*bufferPos) >= DELAY){
      *bufferPos = 0;
    }

    // subtract the last calculated outputs of the prev FIR calculation from the
    // samples of the error-microphone of the new input array. The previous calculated
    // output is the one that predicts the current error microphone and thus
    // this gives the total error. (LENGTH is the delay between the input and the error-microphone)
    prevError[x] = input[x][4] - prevPlay[x];

    play[x] = 0;

    // loop through 4 times, one for every propeller
    for(int y = 0; y < 4; y++){

      //take the x'th sample of the previous inputs which were used to calculate
      // the new output and insert them in the prevInputs array to update it
      if(((*insertPos) - LENGTH) < 0){
        prevInputs[*insertPos][y] = firArray[(*insertPos) - LENGTH + NROFCOEFS][y];
      }
      else{
        prevInputs[*insertPos][y] = firArray[(*insertPos) - LENGTH][y];
      }

      // insert the input whose signal now will reach the error microphone and
      // thus whose output will be calculated from the buffer and into the FIR array
      int pos = (*bufferPos) + LENGTH;
      if(pos >= DELAY){
        pos -= DELAY;
      }
      firArray[*insertPos][y] = inputBuffer[pos][y];

      inputBuffer[*bufferPos][y] = input[x][y];

      // multiply the current FIR array with the coëfficient array such that the
      // most recent insertion is multiplied with the first element of the
      // coëfficient array. Then add those togheter to result in one output.
      // This loops through the whole FIR array for each propeller
      for(int j = 0; j < NROFCOEFS; j++){
        if((*insertPos - j) < 0){
          play[x] += firArray[*insertPos - j + NROFCOEFS][y] * coefs[j][y];
        }
        else{
          play[x] += firArray[*insertPos - j][y] * coefs[j][y];
        }
      }
    }
  }
}

/*
  * Updates the filter coëfficients using the input data and the error value.
  * The error value equals to the error on the LAST output sample. so
  * prevError[LENGTH-1]. A value gotten from a multiplication using the input
  * data, the error value and the stepsize is added to the original value of the coefficient.
*/
void nlmsUpdate(int insertPos, float stepsize, npy_float64 coefs[NROFCOEFS][4], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError){
  // init the p array which will contain the previous inputs divided by the norm
  npy_float64 p[NROFCOEFS][4];

  // loop through the coefficients of every propeller
  for(int i = 0; i < 4; i++){
    npy_float64 norm = 0;

    // calculate the norm. This is the sum of the squared root of each input element
    for(int j = 0; j < NROFCOEFS; j++){
      norm += prevInputs[j][i]*prevInputs[j][i];
    }

    // calculate the p array. This is the division of each element of the input array by the norm plus 1
    for(int j =0; j < NROFCOEFS; j++){
      p[j][i] = prevInputs[j][i]/(norm+1);
    }

    // multiply each input element with the stepsize and the error on the output.
    // insertPos indicates the place where the next element should be inserted
    // minus 1 gives the most recent value. If j increases, the used samples
    // become older.
    for(int j = 0; j < NROFCOEFS; j++){
      // if negative, add the NROFCOEFS
      if(insertPos - 1 - j < 0){
        coefs[j][i] += (p[insertPos - 1 - j + NROFCOEFS][i] * prevError * stepsize);
      }
      else{
        coefs[j][i] += p[insertPos - 1 - j][i] * prevError * stepsize;
      }
    }
  }
}
