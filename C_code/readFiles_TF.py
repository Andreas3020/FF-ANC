import numpy as np
import soundfile as sf

def recordSound():
    #reads the .wav files with
    # a length of 248886
    recArray1, fs = sf.read("Audio/dichtste.wav")
    recArray2, fs = sf.read("Audio/ver.wav")
    recArray3, fs = sf.read("Audio/verder.wav")
    recArray4, fs = sf.read("Audio/verste.wav")
    recArray5, fs = sf.read("Audio/error_speech.wav")

    recArray = np.stack((recArray1, recArray2, recArray3, recArray4, recArray5), axis = 1)

    print(np.shape(recArray))
    print(np.size(recArray))
    print(np.ndim(recArray))
    print(fs)

    print("--------------- FILES ARE READ ---------------")

    # return array for the microphone(s)
    return recArray
