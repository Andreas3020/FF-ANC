import soundfile as sf
fs = 8820

def writeArray(array):

    sf.write('result.wav', array, fs)
    print("--------------- FILES ARE WRITTEN ---------------")

    return 0
