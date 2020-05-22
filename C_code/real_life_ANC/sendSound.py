import sounddevice as sd
import numpy as np
fs = 2205
sd.default.samplerate = fs
sd.default.device = 8

def playSound(recArray):
    # set-up of speaker
    sd.default.channels = 1

    # play received array
    #at the moment commented out for testing purposes
    #sd.play(recArray)
    #sd.wait()
    return 1
