/******************************* RBFm.C ***********************************
  * Authors: Shana Reynders & Andreas Dierickx
  * Titel: Active noise control for ego-noise suppression in an unmanned aerial vehicle
  * Date: 2019-2020
  -----------------------------------------------------------------------------
  * THIS PROGRAM HAS NOT BEEN TESTED, BUT SHOULD WORK WITH SOME MINOR
  * ADJUSTMENTS UNIQUE TO THE DEVICE. MOSTLY IN THE PYTHON EXTENSIONS!
  *
  *
  * Programs purpose is to use Active Noise Cancellation  in the time domain
  * to cancel out the propellor noise of the UAV. The program uses 3 threads
  * and 1 main thread.
  *
  * The first thread will continiously record the sound in the microphone
  * beneath propellors and in the error microphone. Afterwards this will be
  * written in an array. This array is one that is a global variable and thus
  * available to the other threads. Afterwards, this array is then used in the
  * second thread. When writing these values, the delay between then
  * propellers is taken into account.
  *
  * The second thread will continiously manipulate the input from the first
  * thread to produce an inverse sound which will be written to the playx[]
  * array. This thread will also, after the calculation is done, update the NLMS
  * coëfficients using the recordings of the error microphone as total error.
  *
  * The third thread will read the output from the playx[] array and send this
  * to the speaker.
  *
  * The whole program works in two phases. This is necessary because the
  * NLMS update can only be done after the time necessary for a new recording.
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
    pthread_cond_t* cond13;
    pthread_cond_t* cond21;
    pthread_cond_t* cond22;
    pthread_cond_t* cond23;
    int* written1;
    int* written2;
}calcArgs;

typedef struct{
    int duration;
    pthread_mutex_t* mut1r;
    pthread_mutex_t* mut2r;
    pthread_mutex_t* mut1p;
    pthread_mutex_t* mut2p;
    pthread_cond_t* cond12;
    pthread_cond_t* cond13;
    pthread_cond_t* cond22;
    pthread_cond_t* cond23;
    int* written1;
    int* written2;
}playArgs;

/*
* Definition of the static variables
* FOR FREQ OF 8820Hz
*/
#define DELAY 8 // the delay between the closest propeller at 8820Hz is 8 samples
#define LENGTH  1 // Indicates how many input samples are recorded each loop and has to be a natural divider of DELAY
#define NROFCOEFS 15
#define DURATION 20 // Nr of times the loop is executing (consisting of 2 cycles each time)
#define STEPSIZE 4.5
#define interpropellerDelay 2 // sample delay between the two closer propellers and the two further away


/*
* Declaration of the functions
*/
npy_float** audioReceive();
int writeFilesResult(npy_float64 output[]);
int writeFilesSpeaker(npy_float64 output[]);
int audioSend(npy_float64 output[]);

void* micRecord(void* val);
void* calc(void* val);
void* speakerPlay(void* val);

void fir(int* insertPos, int* bufferPos, npy_float64 input[LENGTH + interpropellerDelay][5], npy_float64 firArray[NROFCOEFS][4], npy_float64 coefs[NROFCOEFS][4], npy_float64 play[LENGTH], npy_float64 inputBuffer[DELAY][4], npy_float64 prevPlay[LENGTH], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError[LENGTH]);
void nlmsUpdate(int insertPos, float stepsize, npy_float64 coefs[NROFCOEFS][4], npy_float64 prevInputs[NROFCOEFS][4], npy_float64 prevError);

/*
* Definition of global variables
*/
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
  // condx1: sends a signal from micRecord to calc that a recording is written to recx[]
  // condx2: sends a signal from calc to playSound that the output sound is written to playx[] and thus can start playing.
  //         sends a signal from calc to micRecord that a next recording can start
  // condx3: sends a signal from playSound calc that the playing of sound is done
  pthread_cond_t c11 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond11 = &c11;
  pthread_cond_t c12 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond12 = &c12;
  pthread_cond_t c13 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond13 = &c13;

  pthread_cond_t c21 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond21 = &c21;
  pthread_cond_t c22 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond22 = &c22;
  pthread_cond_t c23 = PTHREAD_COND_INITIALIZER;
  pthread_cond_t* cond23 = &c23;

  // initializes the mutex for the permission to read in the recx[] and the playx[] arrays
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

  // arguments to be passed to the threads
  recordArgs* rArgs = &(recordArgs){.duration = duration, .mut1r = mutex1r, .mut2r = mutex2r, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .written1 = written1, .written2 = written2};

  calcArgs* cArgs = &(calcArgs){.duration = duration, .stepsize = stepsize, .mut1r = mutex1r, .mut2r = mutex2r, .mut1p = mutex1p, .mut2p = mutex2p, .cond11 = cond11, .cond21 = cond21, .cond12 = cond12, .cond22 = cond22, .cond13 = cond13, .cond23 = cond23, .written1 = written1, .written2 = written2};

  playArgs* pArgs = &(playArgs){.duration = duration, .mut1r = mutex1r, .mut2r = mutex2r, .mut1p = mutex1p, .mut2p = mutex2p, .cond12 = cond12, .cond22 = cond22, .cond13 = cond13, .cond23 = cond23, .written1 = written1, .written2 = written2};

  // allows multiple python functions to run at the same time
  Py_BEGIN_ALLOW_THREADS;

  printf("thread start\n");

  first = clock();

  // start the 3 threads. One to record the signal, one to manipulate the signal, one to send out a signal
  pthread_t recordHandle;
  if( pthread_create(&recordHandle , NULL, micRecord, rArgs))exit( EXIT_FAILURE );

  pthread_t calcHandle;
  if( pthread_create(&calcHandle , NULL, calc, cArgs))exit( EXIT_FAILURE );

  pthread_t playHandle;
  if( pthread_create(&playHandle , NULL, speakerPlay, pArgs))exit( EXIT_FAILURE );

  // joins the threads when they are finished
  void* result;
  if(pthread_join(recordHandle, &result)==-1)
  {
      exit(EXIT_FAILURE);
      printf("could not join\n");
  }

  if(pthread_join(calcHandle, &result)==-1)
  {
      exit(EXIT_FAILURE);
      printf("could not join\n");
  }

  if(pthread_join(playHandle, &result)==-1)
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
  * Calls the python-extension receiveSound which will record the sound in the
  * microphones. The python-extension return an array which, after conversion
  * to a by c understandable array, is returned to the micRecord function.
*/
npy_float** audioReceive(){
  // ensures the python thread safety for the garbage collector (GC)
  PyGILState_STATE micState;
  micState = PyGILState_Ensure();

  //variables init
  PyObject *pName, *pModule, *pFunc, *pArgs;
  PyObject* pValue;
  npy_float** array;

  // import the .py file in which the to-call python function is located
  pName = PyUnicode_FromString("receiveSound");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if(pModule == NULL) printf("its null\n");

  if (pModule != NULL) {
    // Get the reference to the to-call python function and checks if its callable
    pFunc = PyObject_GetAttrString(pModule, "recordSound");
    if (pFunc && PyCallable_Check(pFunc)) {
      // set arguments you want to pass along to the python function
      pArgs = PyTuple_New(1);
      PyTuple_SetItem(pArgs, 0, PyLong_FromLong(LENGTH));

      // call the python function and return the numpy array in pValue
      pValue = PyObject_CallObject(pFunc, pArgs);
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
      //printf("input\n%"NPY_FLOAT_FMT"\n%"NPY_FLOAT_FMT"\n%"NPY_FLOAT_FMT"\n%"NPY_FLOAT_FMT"\n", array[0], array[5], array[10], array[12]);
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


  // release the thread safety
  PyGILState_Release(micState);
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
  * Receives an array from the speakerPlay function. This array is sent to a
  * python-extension which will play this array on the speaker. When done '0'
  * is returned
*/
int audioSend(npy_float64 output[]){
  // ensures the python thread safety for the garbage collector (GC)
  PyGILState_STATE speakerState;
  speakerState = PyGILState_Ensure();

  // init variables
  PyObject *pName, *pModule, *pFunc;
  PyObject *pValue;
  npy_intp dims2[2] = {LENGTH,1};
  // convert the, by c-compiler understandable, array to an array understandable by a python-compiler
  PyObject * pyArray = PyArray_SimpleNewFromData(2, dims2,NPY_FLOAT, output);
  if (pyArray == NULL){
    printf("Conversion failed\n");
  }

  // import the .py file in which the to-call python function is located
  pName = PyUnicode_FromString("sendSound");
  pModule = PyImport_Import(pName);
  Py_DECREF(pName);
  if(pModule == NULL) printf("its null\n");

  if (pModule != NULL) {
      // Get the reference to the to-call python function and checks if its callable
      pFunc = PyObject_GetAttrString(pModule, "playSound");
      if (pFunc && PyCallable_Check(pFunc)) {
        // set arguments you want to pass along to the python function
        PyObject* pArg = PyTuple_New(1);
        PyTuple_SetItem(pArg, 0, pyArray);

        // call the python function to play the sound by reading the array
        //pValue = PyObject_CallObject(pFunc, ptemp);
        pValue = PyObject_CallObject(pFunc, pArg);
        Py_DECREF(pArg);

        // check if array is played
        if (pValue != NULL) {
            Py_DECREF(pValue);
        }

        // if the array was not correctly read
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

  // release the thread safety for the GC
  PyGILState_Release(speakerState);
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
    *written2 = 3;
    pthread_mutex_t* mutex1r = args.mut1r;
    pthread_mutex_t* mutex2r = args.mut2r;

    // used as temporary storage of the recordings to write them away in the
    // correct place of recx[][]
    npy_float** tempArray;

  // loop through the 2 cycles of the reading for duration times
  for(int i = 0; i<duration; i++){
    // receive the recordings array
    tempArray = audioReceive();

    // write the input to the rec1[][] array
    pthread_mutex_lock(mutex1r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written1 != 0){
        pthread_cond_wait(args.cond22, mutex1r);
      }
      // loops through the rec array to put in new inputs. Each propeller has
      // its own collum in the rec array. Two propellers have a larger delay
      // than the other two. Those are inserted in a delayed position.
      for(int x = 0; x < LENGTH; x++){

        // if the delay between the two furthes propellers and the closest ones
        // is zero
        if(interpropellerDelay == 0){
          rec1[x][0] = tempArray[x][0];
          rec1[x][1] = tempArray[x][1];
          rec1[x][2] = tempArray[x][2];
          rec1[x][3] = tempArray[x][3];
        }
        else{
          // if the delayed position is larger than length it is passed onto the
          // next cycle to be used there. Here the passed on values are added
          if((LENGTH + x) < (LENGTH + interpropellerDelay)){
            rec1[x][2] = rec2[LENGTH + x][2];
            rec1[x][3] = rec2[LENGTH + x][3];
          }

          // add the first two propeller recordings
          rec1[x][0] = tempArray[x][0];
          rec1[x][1] = tempArray[x][1];

          // add the recordings of the two propellers further away
          rec1[x + interpropellerDelay][2] = tempArray[x][2];
          rec1[x + interpropellerDelay][3] = tempArray[x][3];
        }
        // insert the error microphone recording
        rec1[x][4] = tempArray[x][4];
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

    // receive the recordings array
    tempArray = audioReceive();

    // write the input array to the rec2[][] array
    pthread_mutex_lock(mutex2r);

      // wait till the previous NLMS update is done before writing new input
      // values to the recordings array
      if(*written2 != 0){
        pthread_cond_wait(args.cond12, mutex2r);
      }

      // loops through the rec array to put in new inputs. Each propeller has
      // its own collum in the rec array. Two propellers have a larger delay
      // than the other two. Those are inserted in a delayed position.
      for(int x = 0; x < LENGTH; x++){

        // if the delay between the two furthes propellers and the closest ones
        // is zero
        if(interpropellerDelay == 0){
          rec2[x][0] = tempArray[x][0];
          rec2[x][1] = tempArray[x][1];
          rec2[x][2] = tempArray[x][2];
          rec2[x][3] = tempArray[x][3];
        }
        else{
          // if the delayed position is larger than length it is passed onto the
          // next cycle to be used there. Here the passed on values are added
          if((LENGTH + x) < (LENGTH + interpropellerDelay)){
            rec2[x][2] = rec1[LENGTH + x][2];
            rec2[x][3] = rec1[LENGTH + x][3];
          }

          // add the first two propeller recordings
          rec2[x][0] = tempArray[x][0];
          rec2[x][1] = tempArray[x][1];

          // add the recordings of the two propellers further away
          rec2[x + interpropellerDelay][2] = tempArray[x][2];
          rec2[x + interpropellerDelay][3] = tempArray[x][3];
        }

        // insert the error microphone recording
        rec2[x][4] = tempArray[x][4];
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

      pthread_mutex_lock(mutex2r);
      if(*written2 != 3){
        pthread_cond_wait(args.cond23, mutex2r);
        *written2 = 0;
      }
      else{
        *written2 = 0;
      }
      pthread_mutex_unlock(mutex2r);

      *written1 = 2;

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

      // check if the previous output has been played by the speakerPlay function
      // before continiouing with a new recording
      pthread_mutex_lock(mutex1r);
      if(*written1 != 3){
        pthread_cond_wait(args.cond13, mutex1r);
        *written1 = 0;
      }
      else{
        *written1 = 0;
      }
      pthread_mutex_unlock(mutex1r);

      *written2 = 2;


      pthread_mutex_unlock(mutex2p);
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond22);

    // calls the update function to update the coefficients
    nlmsUpdate(insertPos, stepsize, coefs, prevInputs, prevError[LENGTH-1]);
  }

  printf("calc thread done\n");
  pthread_exit(NULL);
}

void* speakerPlay(void* pArgs){
  printf("in speakerPlay\n\n");

  // initialize all variables
  //separate pArgs into all its variables
  playArgs args = *(playArgs*)pArgs;
    int duration = args.duration;
    pthread_mutex_t* mutex1p = args.mut1p;
    pthread_mutex_t* mutex2p = args.mut2p;
    pthread_mutex_t* mutex1r = args.mut1r;
    pthread_mutex_t* mutex2r = args.mut2r;
    int* written1 = args.written1;
    int* written2 = args.written2;

    // used to time how long a function takes to execute
    time_t first, second;
    time_t etime = 0;

  for(int i = 0; i < duration; i++){
    // read the output from the playx[] array
    pthread_mutex_lock(mutex1r);
      // wait if there is no new output yet coming from the calc function
      if(*written1 != 2){
        pthread_cond_wait(args.cond12, mutex1r);
      }

      // when there is a new output, send this to the speaker
      audioSend(play1);

      first = clock();

      // if reading the playx[] array is done
      *written1 = 3;
    pthread_mutex_unlock(mutex1r);
    pthread_cond_broadcast(args.cond13);

    // read the output from the outputfile
    pthread_mutex_lock(mutex2r);
      // wait if there is no new output yet
      if(*written2 != 2){
        pthread_cond_wait(args.cond22, mutex2r);
      }

      second = clock();
      etime += second - first;

      // when there is a new output, send this to the speaker
      audioSend(play2);

      // if reading the playx[] array is done
      *written2 = 3;
    pthread_mutex_unlock(mutex2r);
    pthread_cond_broadcast(args.cond23);
  }

  printf("speaker duration: %d\telapsed time in betwee total: %ld\taverage time microsec: %ld\taverage lost samples: %f\n", duration, etime, etime/(duration), (double)etime/(duration*454));
  printf("Speaker thread done\n");
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
