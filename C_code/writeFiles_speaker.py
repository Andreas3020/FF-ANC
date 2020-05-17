import soundfile as sf
fs = 8820

def writeArray(array):

    sf.write('results_TF/Speech/outTF_speech_0.5.wav', array, fs)
    print("--------------- FILES ARE WRITTEN ---------------")

    return 0
