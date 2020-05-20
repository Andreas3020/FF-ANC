% -------------------------------------------------------------------------
% Thesis 2019 - 2020 : ANC of propeller noise in a microphone array mounted
% on an UAV. 
% 
% Matlab Code : Active Noise control in Time-Frequency domain
%
% Based on : Time-frequency-domain filtered-x LMS algorithm for active
% noise control by X.L. Tang and C.-M. Lee 
% 
% Created on : 16 March 2020 
% 
% Authors : Andreas Dierickx & Shana Reynders 
%
% -------------------------------------------------------------------------
clc; close all; clear all;

%% Concatenate the different audio files
[y1, fs_n] = audioread('noise_only_dichtste_5_03.wav');
[y2] = audioread('noise_only_ver_5_03.wav');
[y3] = audioread('noise_only_verder_5_03.wav');
[y4] = audioread('noise_only_verste_5_03.wav');
[error] = audioread('noise_only_mic_5_03.wav');

noise = horzcat(y1, y2, y3, y4);

%% Downsampling
factor = 20;
fs_down = fs_n/factor;
noise_ds = zeros(round(length(noise(:,1))/factor),4);
error_ds = zeros(round(length(error)/factor),1);
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
noise = lowpass(noise, 1000, fs_n);
error = lowpass(error, 1000, fs_n);

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

error_total = error_part(:,1) + error_part(:,2) + error_part(:,3) + error_part(:,4);

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
frame_size = round(length(noise(:,1))/50);
for i = 1 : round(frame_size*(1-overlap)) : length(noise)-(frame_size-1)
    signal_power(k) = mean((speech_ds_before(i : i+(frame_size-1))).^2); 
    noise_power(k) = mean((error_before(i : i+(frame_size-1))).^2);
    k = k + 1;
end
SNR = pow2db(signal_power./noise_power);
plot(SNR);

%% Initialise variables
% LMS algorithm coefficients
step_size = 0.1;

%% Initialise STFT variables
Number_of_fft = 20;                                     % Number of fft's 
overlap_fft = Number_of_fft/2;                          % 50% overlap

hann_window = sqrt(hann(Number_of_fft,'periodic'));     % analysis window
bins = floor(Number_of_fft/2)+1;                        % number of bins in onsided FFT 

frequencies = 0:fs_n/Number_of_fft:fs_n/2;            % fs_pn = sampling frequency 

[noise_1_STFT] = calc_STFT(noise(:,1), fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided'); 
[noise_2_STFT] = calc_STFT(noise(:,2), fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided'); 
[noise_3_STFT] = calc_STFT(noise(:,3), fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided'); 
[noise_4_STFT] = calc_STFT(noise(:,4), fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided'); 

[error_before_STFT, freq_vec] = calc_STFT(error, fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided'); 
    
[K_bins, L_time_frames] = size(noise_1_STFT);

coef_length = 2;

% Filter coefficients
filter_coefficients_1 = -0.5 + rand(K_bins,coef_length);
filter_coefficients_2 = -0.5 + rand(K_bins,coef_length);
filter_coefficients_3 = -0.5 + rand(K_bins,coef_length);
filter_coefficients_4 = -0.5 + rand(K_bins,coef_length);

% Figures
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_1_STFT(:,:))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of propeller noise');

figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(7*noise_2_STFT(:,:))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of propeller noise');
 
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_3_STFT(:,:))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of propeller noise');
 
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_4_STFT(:,:))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of propeller noise');

% Add zeros
noise_1_STFT_zero = [zeros(K_bins,coef_length-1) noise_1_STFT];
noise_2_STFT_zero = [zeros(K_bins,coef_length-1) noise_2_STFT];
noise_3_STFT_zero = [zeros(K_bins,coef_length-1) noise_3_STFT];
noise_4_STFT_zero = [zeros(K_bins,coef_length-1) noise_4_STFT];

%% ANC Algorithm 
loudspeaker_STFT = zeros(K_bins, L_time_frames);
noise_1_zero = [coef_length*Number_of_fft/2 ; noise(:,1)];
noise_2_zero = [coef_length*Number_of_fft/2 ; noise(:,2)];
noise_3_zero = [coef_length*Number_of_fft/2 ; noise(:,3)];
noise_4_zero = [coef_length*Number_of_fft/2 ; noise(:,4)];

i = 1;
j = 1;
tic
for l = coef_length : 1 : L_time_frames - 10
    n1_STFT = calc_STFT([zeros(overlap_fft,1);noise_1_zero(i : i + (coef_length-1)*Number_of_fft/2);zeros(overlap_fft,1)], fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    n2_STFT = calc_STFT([zeros(overlap_fft,1);noise_2_zero(i : i + (coef_length-1)*Number_of_fft/2);zeros(overlap_fft,1)], fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    n3_STFT = calc_STFT([zeros(overlap_fft,1);noise_3_zero(i : i + (coef_length-1)*Number_of_fft/2);zeros(overlap_fft,1)], fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    n4_STFT = calc_STFT([zeros(overlap_fft,1);noise_4_zero(i : i + (coef_length-1)*Number_of_fft/2);zeros(overlap_fft,1)], fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    
    for k = 1 : K_bins
        loudspeaker_STFT(k,l) = filter_coefficients_1(k,:) * n1_STFT(k,:)' ...
                    + filter_coefficients_2(k,:) * n2_STFT(k,:)' ...
                    + filter_coefficients_3(k,:) * n3_STFT(k,:)' ...
                    + filter_coefficients_4(k,:) *  n4_STFT(k,:)';       
    end
    loudspeaker_aux = calc_ISTFT([loudspeaker_STFT(:,l)], hann_window, Number_of_fft,overlap_fft, 'onesided');
    loudspeaker(j:j+(Number_of_fft-1),1) = loudspeaker_aux; 
    
    error_mic_signal(j:j+(Number_of_fft-1),1) = speech_noise(j:j+(Number_of_fft-1),1) - loudspeaker(j:j+(Number_of_fft-1),1);
        
    error_mic_STFT(:,1) = calc_STFT([error_mic_signal(j:j+(Number_of_fft-1),1)], fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');
    j = j + Number_of_fft/2;
    i = i + Number_of_fft/2;
           
    for k = 1 : K_bins
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
error_STFT_total = calc_STFT(error_mic_signal, fs_n, hann_window,Number_of_fft, overlap_fft, 'onesided');

%% Graphs
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_1_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of propeller noise');

figure; imagesc(1:L_time_frames,frequencies,mag2db(abs(error_before_STFT(:,:,1))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'); set(gca,'Fontsize',14); xlabel('Time (s)'), ylabel('Frequency (Hz)'),title('Error Microphone signal before ANC');

figure; imagesc(1:L_time_frames,frequencies,mag2db(abs(error_STFT_total(:,:,1))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'); set(gca,'Fontsize',14); xlabel('Time (s)'), ylabel('Frequency (Hz)'),title('Error Microphone signal after ANC');

figure; imagesc(1:L_time_frames,frequencies,mag2db(abs(loudspeaker_STFT(:,:,1))),[-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'); set(gca,'Fontsize',14); xlabel('Time (s)'), ylabel('Frequency (Hz)'),title('Loudspeaker signal');

%% Plots
% Original signal // Counter signal // Output signal 
figure; hold on; subplot(1,3,1); plot(-error, 'g');
subplot(1,3,2); plot(loudspeaker, 'r');
subplot(1,3,3); plot(error_mic_signal, 'b');  
hold off; xlabel('Signal samples in time'); ylabel('Signal value'); title('Signal prediction using NLMS');

figure
plot(error_mic_signal, 'b');
audiowrite('audio6.wav', error_mic_signal, fs_n)

[error_STFT, L_time_frames, frequencies] = ANC_time_to_TF(error, fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(error_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of Error microphone before ANC');

[error_2_STFT, L_time_frames, frequencies] = ANC_time_to_TF(error_mic_signal, fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(error_2_STFT(:,:,1))), [-65, 10]); 
colorbar; axis xy; set(gcf,'color','w'), set(gca,'Fontsize',15), xlabel('Time Frames, l', 'FontSize', 15), ylabel('Frequency, k (Hz)', 'FontSize', 15);

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
    %signal_power(k) = mean((speech_ds(i : i+(frame_size-1))).^2); 
    noise_power(k) = mean((noise_only(i : i+(frame_size-1))).^2);
    k = k + 1;
end

SNR_after = pow2db((signal_power)./noise_power);
%save('SNR2.mat', 'SNR_after');

figure
hold on
plot(SNR);
plot(SNR_after);
xlabel('Time Frames', 'FontSize', 15);
ylabel('SNR (dB)', 'FontSize', 15)
legend('before', 'after');
hold off

y_freq = fft(error_mic_signal(end-22000:end));
L = length(error_mic_signal(end-22000:end));
P5 = abs(y_freq/L);
P6 = P5(1:L/2+1);
P6(2:end-1) = 2*P6(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P6); set(gca,'Fontsize',15); xlabel('f (Hz)','FontSize',15); ylabel('|P1(f)|','FontSize',15); ylim([0 0.0003]);