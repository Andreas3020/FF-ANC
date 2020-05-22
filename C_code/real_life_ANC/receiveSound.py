import sounddevice as sd
import numpy as np
import soundfile as sf

fs = 44100 #should be 2205 but the microphone of my computer could not handle that
sd.default.samplerate = fs
sd.default.device = 0
sd.default.channels = 2

def recordSound(length=50):
    #records the sound
    #for testing purposes at te moment files are read, but should be changed to the line 20-21
    recArray1, fs = sf.read("Audio/dichtste.wav")
    recArray2, fs = sf.read("Audio/ver.wav")
    recArray3, fs = sf.read("Audio/verder.wav")
    recArray4, fs = sf.read("Audio/verste.wav")
    recArray5, fs = sf.read("Audio/error_speech.wav")
    recArray = np.stack((recArray1[0:length], recArray2[0:length], recArray3[0:length], recArray4[0:length], recArray5[0:length]), axis = 1)

    #recArray = sd.rec(int(length))
    #sd.wait()

    # return array for the microphone(s)
    return recArray
