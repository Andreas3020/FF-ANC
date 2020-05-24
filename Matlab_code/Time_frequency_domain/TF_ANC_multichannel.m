% -------------------------------------------------------------------------
% Thesis 2019 - 2020 : Active noise control for ego-noise suppression in an 
% unmanned aerial vehicle
% 
% Matlab Code : Active Noise control in Time-Frequency domain: multichannel
%
% Created on : 16 March 2020 
% 
% Authors : Andreas Dierickx & Shana Reynders 
%
% -------------------------------------------------------------------------
clc; close all; clear all;

%% Concatenate the different audio files
[y1, fs_n] = audioread('noise_only_closest.wav');
[y2] = audioread('noise_only_close.wav');
[y3] = audioread('noise_only_further.wav');
[y4] = audioread('noise_only_furthest.wav');
[error] = audioread('noise_only_error_mic.wav'); 

noise = horzcat(y1, y2, y3, y4);

%% Downsampling to lower sampling frequency 
factor = 5;
fs_down = fs_n/factor;

noise_ds = zeros(round(length(y1)/factor),4);                               % Initialisation of the downsampled noise
error_ds = zeros(round(length(error)/factor),1);                            % Initialisation of the downsampled error microphone

j = 1;
for i = 1:factor:length(noise(:,1))  
    noise_ds(j,:) = noise(i,:);
    error_ds (j,1) = error(i,1);
    j = j + 1;
end

fs_n = fs_down;
noise = noise_ds;
error = error_ds;
error_before = error;

%% Lowpass
load('lowpassB1000.mat', 'lpB1000');
noise = filter(lpB1000,noise);
error = filter(lpB1000,error);

%% Primary path transfer function
% Delay
H2_1 = round(44/factor);
H2_2 = round(49/factor);
H2_3 = round(53/factor);
H2_4 = round(62/factor);

error_part(:,1) = [zeros(H2_1,1); noise(1:end-H2_1,1)];
error_part(:,2) = [zeros(H2_2,1); noise(1:end-H2_2,2)];
error_part(:,3) = [zeros(H2_3,1); noise(1:end-H2_3,3)];
error_part(:,4) = [zeros(H2_4,1); noise(1:end-H2_4,4)];

error_total = error_part(:,1) + error_part(:,2) + error_part(:,3) + ...
    error_part(:,4);

%% Propeller noise characteristic
% -- PROPELLER 1 --
% Frequency respone
ydft = fft(noise(:,1));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 1'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

% Amplitude spectrum
y_freq = fft(noise(:,1));
L = length(noise(:,1));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 1'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

% Spectrogram
[noise_1_STFT, L_time_frames, frequencies] = ...
    ANC_T_to_TF(noise(:,1), fs_n);
figure; imagesc(1:L_time_frames, frequencies, ...
    mag2db(abs(noise_1_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), 
ylabel('Frequency, k (Hz)'); 
title('Spectrogram of ego-noise in propeller 1');

% -- PROPELLER 2 --
% Frequency respone
noise(:,2) = 3 * noise(:,2);
ydft = fft(noise(:,2));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 2'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

% Amplitude response 
y_freq = fft(noise(:,2));
L = length(noise(:,2));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 2'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

% Spectrogram 
[noise_2_STFT, L_time_frames, frequencies] = ...
    ANC_T_to_TF(noise(:,2), fs_n);
figure; imagesc(1:L_time_frames, frequencies, ...
    mag2db(abs(noise_2_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)');
title('Spectrogram of ego-noise in propeller 2');

% -- PROPELLER 2 --
% Frequency respone
ydft = fft(noise(:,3));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 3'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

% Amplitude response 
y_freq = fft(noise(:,3));
L = length(noise(:,3));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 3'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

% Spectrogram
[noise_3_STFT, L_time_frames, frequencies] = ...
    ANC_T_to_TF(noise(:,3), fs_n);
figure; imagesc(1:L_time_frames, frequencies, ...
    mag2db(abs(noise_3_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)');
title('Spectrogram of ego-noise in propeller 3');

% -- PROPELLER 4 --
% Frequency respone
ydft = fft(noise(:,4));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 4'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

% Amplitude response 
y_freq = fft(noise(:,4));
L = length(noise(:,4));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 4'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

% Spectrogram 
[noise_4_STFT, L_time_frames, frequencies] = ...
    ANC_T_to_TF(noise(:,4), fs_n);
figure; imagesc(1:L_time_frames, frequencies, ...
    mag2db(abs(noise_4_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)');
title('Spectrogram of ego-noise in propeller 3');

%% Speech implementation
[speech,fs] = audioread('speech_male_count.wav');
speech_ds = downsample(speech, factor);
speech_ds = speech_ds(1:length(noise))/100;

j = 1;
speech_noise = error;
for i = 1:length(error_before)-1
    speech_noise(i) = speech_noise(i) + speech_ds(j);
    j = j + 1;
    if j == length(speech_ds)
        j = 1;
    end
end
speech_ds_before = speech_ds;
speech_noise = lowpass(speech_noise, 1000, fs_n);
speech_ds = lowpass(speech_ds, 1000, fs_n);
%sound(speech_noise, fs_n);

%% SNR before ANC
k = 1;
overlap = 0.7;
frame_size = round(length(noise(:,1))/50);                                  % Calculate the SNR every 50 samples
for i = 1 : round(frame_size*(1-overlap)) : length(noise)-(frame_size-1)
    signal_power(k) = mean((speech_ds_before(i : i+(frame_size-1))).^2); 
    noise_power(k) = mean((error_before(i : i+(frame_size-1))).^2);
    k = k + 1;
end
SNR = pow2db(signal_power./noise_power);                                    % SNR calculation in dB 

figure; plot(SNR); xt = get(gca, 'XTick');                                  % Plot the SNR 
set(gca, 'XTick',xt, 'XTickLabel',round(xt/5.9));
set(gcf,'color','w'), set(gca,'Fontsize',15); 
xlabel('Time (s)', 'Fontsize',15'); ylabel('SNR (dB)','Fontsize',15);

%% Initialise variables
% NLMS algorithm coefficients
step_size = 0.1;

%% Initialise STFT variables
Number_of_fft = 20;                                                         % Number of fft's 
overlap_fft = Number_of_fft/2;                                              % 50% overlap

hann_window = sqrt(hann(Number_of_fft,'periodic'));                         % analysis window
bins = floor(Number_of_fft/2)+1;                                            % number of bins in onsided FFT 

frequencies = 0:fs_n/Number_of_fft:fs_n/2;                                  % fs_n = sampling frequency 

[noise_1_STFT] = calc_STFT(noise(:,1), fs_n, hann_window,Number_of_fft,...
    overlap_fft, 'onesided'); 
    
[K_bins, L_time_frames] = size(noise_1_STFT);                               % Determine number of frequency bins and time frames

K_end = 1;                                                                  % Number of bins used for calculation 

coef_length = 2;                                                            % Number of coefficients

% Filter coefficients
filter_coefficients_1 = -0.5 + rand(K_end,coef_length);
filter_coefficients_2 = -0.5 + rand(K_end,coef_length);
filter_coefficients_3 = -0.5 + rand(K_end,coef_length);
filter_coefficients_4 = -0.5 + rand(K_end,coef_length);

%% ANC Algorithm 
loudspeaker_STFT = zeros(K_bins, L_time_frames);                            % Initialese loudspeaker in Time-Frequency domain
noise_1_zero = [coef_length*Number_of_fft/2 ; noise(:,1)];
noise_2_zero = [coef_length*Number_of_fft/2 ; noise(:,2)];
noise_3_zero = [coef_length*Number_of_fft/2 ; noise(:,3)];
noise_4_zero = [coef_length*Number_of_fft/2 ; noise(:,4)];

i = 1;
j = 1;
tic
for l = coef_length : 1 : L_time_frames
    n1_STFT = calc_STFT([zeros(overlap_fft,1);noise_1_zero(i : i + ...      % Propeller noise to Time-Frequency domain
        (coef_length-1)*overlap_fft);zeros(overlap_fft,1)], fs_n,...
        hann_window,Number_of_fft, overlap_fft, 'onesided');
    n2_STFT = calc_STFT([zeros(overlap_fft,1);noise_2_zero(i : i + ...
        (coef_length-1)*overlap_fft);zeros(overlap_fft,1)], fs_n,...
        hann_window,Number_of_fft, overlap_fft, 'onesided');
    n3_STFT = calc_STFT([zeros(overlap_fft,1);noise_3_zero(i : i + ...
        (coef_length-1)*overlap_fft);zeros(overlap_fft,1)], fs_n,...
        hann_window,Number_of_fft, overlap_fft, 'onesided');
    n4_STFT = calc_STFT([zeros(overlap_fft,1);noise_4_zero(i : i + ...
        (coef_length-1)*overlap_fft);zeros(overlap_fft,1)], fs_n,...
        hann_window,Number_of_fft, overlap_fft, 'onesided');

    for k = 1 : K_end
        for z = 0 : 1                                                       % Calculation of output signal (Loudspeaker)
        loudspeaker_STFT(k,l:l+z) = filter_coefficients_1(k,z+1) * n1_STFT(k,z+1)' ...
                    + filter_coefficients_2(k,z+1) * n2_STFT(k,z+1)' ...
                    + filter_coefficients_3(k,z+1) * n3_STFT(k,z+1)' ...
                    + filter_coefficients_4(k,z+1) *  n4_STFT(k,z+1)';  
        end
    end
    
    loudspeaker_aux = calc_ISTFT([loudspeaker_STFT(:,l:l+1)],...
        hann_window, Number_of_fft,overlap_fft, 'onesided');
    loudspeaker(j:j+(overlap_fft-1),1) = ...
        loudspeaker_aux(overlap_fft:Number_of_fft-1); 
    
    error_mic_signal(j:j+(overlap_fft-1),1) =...                            % Calculation of error in time domain
        speech_noise(j:j+(overlap_fft-1),1) - loudspeaker(j:j+(overlap_fft-1),1);
        
    error_mic_STFT = calc_STFT([zeros(overlap_fft,1);... 
        error_mic_signal(j:j+(overlap_fft-1),1);zeros(overlap_fft,1)],...
    fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    
    j = j + overlap_fft;
    i = i + overlap_fft;
    
    % Filter weight update with NLMS
    for k = 1 : K_end
         norm1 = sum(n1_STFT(k,:).'.*n1_STFT(k,:)');
         norm2 = sum(n2_STFT(k,:).'.*n2_STFT(k,:)');
         norm3 = sum(n3_STFT(k,:).'.*n3_STFT(k,:)');
         norm4 = sum(n4_STFT(k,:).'.*n4_STFT(k,:)');
         
         p1(k,1:coef_length) = n1_STFT(k,:) / (norm1+1);
         p2(k,1:coef_length) = n2_STFT(k,:) / (norm2+1);
         p3(k,1:coef_length) = n3_STFT(k,:) / (norm3+1);
         p4(k,1:coef_length) = n4_STFT(k,:) / (norm4+1);
         
        f1(k,:) = step_size * p1(k,:) * error_mic_STFT(k,1);
        f2(k,:) = step_size * p2(k,:) * error_mic_STFT(k,1);
        f3(k,:) = step_size * p3(k,:) * error_mic_STFT(k,1);
        f4(k,:) = step_size * p4(k,:) * error_mic_STFT(k,1);
    end

    filter_coefficients_1 = filter_coefficients_1 + f1;
    filter_coefficients_2 = filter_coefficients_2 + f2;
    filter_coefficients_3 = filter_coefficients_3 + f3;
    filter_coefficients_4 = filter_coefficients_4 + f4;
end
toc

error_STFT_total = calc_STFT(error_mic_signal, fs_n, hann_window,...
    Number_of_fft, overlap_fft, 'onesided');

%% Frequency plot, amplitude plot and spectrogram of error microphone signal before ANC
microphone_signal_dft = fft(error);
n_end = length(microphone_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(microphone_signal_dft).^2/n_end);

figure; semilogx(f_end, power);                                             % Frequency response plot
title('Frequency response graph error microphone signal before ANC'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(error);
L = length(error);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); set(gca,'Fontsize',15);                                 % Amplitude response plot
title('Single-Sided Amplitude Spectrum of error microphone signal before ANC');
xlabel('Frequency (Hz)','FontSize',15); ylabel('Amplitude','FontSize',15); 
ylim([0 0.0003]);

[error_STFT, L_time_frames, frequencies] = ANC_T_to_TF(error, fs_n);

figure; imagesc(1:L_time_frames, frequencies, ...                           % Spectrogram 
    mag2db(abs(error_STFT(:,:,1))),[-65, 10]); 
colorbar; axis xy; xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',round(xt*512/8820));
set(gcf,'color','w'), set(gca,'Fontsize',15),
xlabel('Time (s)', 'Fontsize',15'), 
ylabel('Frequency, k (Hz)','Fontsize',15);

%% Frequency plot, amplitude plot and spectrogram of error microphone signal after ANC
microphone_signal_dft = fft(error_mic_signal);
n_end = length(microphone_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(microphone_signal_dft).^2/n_end);

figure; semilogx(f_end, power);                                             % Frequency response 
title('Frequency response graph error microphone signal'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(error_mic_signal);
L = length(error_mic_signal);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); set(gca,'Fontsize',15); xlabel('f (Hz)','FontSize',15); % Amplitude response 
title('Single-Sided Amplitude Spectrum of error microphone signal after ANC');
ylabel('|P1(f)|','FontSize',15); ylim([0 0.0003]);

figure; imagesc(1:L_time_frames, frequencies, ...                           % Spectrogram 
    mag2db(abs(error_STFT_total(:,:,1))), [-65, 10]); 
colorbar; axis xy; set(gcf,'color','w'), set(gca,'Fontsize',15);
xlabel('Time Frames, l','FontSize',15), 
ylabel('Frequency, k (Hz)','FontSize',15);
title('Spectrogram of Error microphone after ANC');

%% Frequency plot, amplitude plot and spectrogram of loudspeaker signal after ANC
loudspeaker_signal_dft = fft(loudspeaker);
n_end = length(loudspeaker_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(loudspeaker_signal_dft).^2/n_end);

figure; semilogx(f_end, power);                                             % Frequency response 
title('Frequency response graph loudspeaker signal'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(loudspeaker);
L = length(loudspeaker);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1);                                                         % Amplitude response 
title('Single-Sided Amplitude Spectrum of the loudspeaker'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

figure; imagesc(1:L_time_frames, frequencies, ...                           % Spectrogram 
    mag2db(abs(loudspeaker_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), 
ylabel('Frequency, k (Hz)'), title('Spectrogram of Loudspeaker noise');

%% Plot of output signal
figure; 
plot(error_mic_signal);
ylim([-0.1 0.1]); 
xlabel('Timeframes (s)', 'FontSize', 15); 
ylabel('Amplitude', 'FontSize', 15); title('Signal prediction using NLMS');

%% SNR after ANC
% Signal = e = s + n + l;
noise_only = error(1:length(loudspeaker)) - loudspeaker;
signal = error_mic_signal - noise_only;

figure; hold on 
subplot(1,3,1); plot(speech_ds, 'g'); title('signal'); ylim([-0.1 0.1])
subplot(1,3,2); plot(noise_only, 'r'); title('nois_only'); ylim([-0.1 0.1])
hold off

k = 1;
overlap = 0.7;
frame_size = round(length(noise(:,1))/50);
for i = coef_length : round(frame_size*(1-overlap)) : length(noise)-(frame_size-1)
    signal_power(k) = mean((signal(i : i+(frame_size-1))).^2); 
    noise_power(k) = mean((noise_only(i : i+(frame_size-1))).^2);
    k = k + 1;
end

SNR_after = pow2db((signal_power)./noise_power);                            % SNR calculation in dB

figure; hold on
plot(SNR); plot(SNR_after);                                                 % Plot SNR before and after ANC
xlabel('Time Frames', 'FontSize', 15); ylabel('SNR (dB)', 'FontSize', 15)
legend('before', 'after'); hold off