import soundfile as sf
fs = 8820

def writeArray(array):

    sf.write('results_time/Speech/result_final_speech_4.5_15.wav', array, fs)
    print("--------------- FILES ARE WRITTEN ---------------")

    return 0
