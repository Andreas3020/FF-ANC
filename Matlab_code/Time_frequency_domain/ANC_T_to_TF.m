function[signal_STFT, L_time_frames, frequencies] = ANC_T_to_TF(signal, fs_pn)

    Number_of_fft = 1024;
    overlap_fft = Number_of_fft/2;  
    
    % It is a squared hann window -> leg in de thesis uit waarom
    hann_window = sqrt(hann(Number_of_fft,'periodic'));                     % analysis window
    bins = floor(Number_of_fft/2)+1;                                        % number of bins in onsided FFT 

    frequencies = 0:fs_pn/Number_of_fft:fs_pn/2;                            % fs_pn = sampling frequency 
    
    % Put signal in the STFT domain
    [signal_STFT, freq_vec] = calc_STFT(signal, fs_pn, hann_window,Number_of_fft, overlap_fft, 'onesided');
    
    [K_bins, L_time_frames] = size(signal_STFT);
end