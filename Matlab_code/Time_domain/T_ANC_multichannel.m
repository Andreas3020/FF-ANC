%% ------------------------------------------------------------------------
% Thesis 2019 - 2020 : ANC of propeller noise in a microphone array mounted
% on an UAV. 
% 
% Matlab Code : Active Noise control in Time domain
% 
% Created on : 09 February 2020 
% 
% Authors : Andreas Dierickx & Shana Reynders 
%
% -------------------------------------------------------------------------
clc; close all; clear all;

%% Read and Concatenate the different audio files
[y1, fs_n] = audioread('noise_only_dichtste_5_03.wav');
[y2] = audioread('noise_only_ver_5_03.wav');
[y3] = audioread('noise_only_verder_5_03.wav');
[y4] = audioread('noise_only_verste_5_03.wav');
[error] = audioread('noise_only_mic_5_03.wav'); 

noise = horzcat(y1, y2, y3, y4);

%% Downsampling
factor = 5;
fs_down = fs_n/factor;
noise_ds = zeros(round(length(y1)/factor),4);
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

error_noise = error_part(:,1) + error_part(:,2) + error_part(:,3) ...
    + error_part(:,4);

%% Propeller noise characteristic
% Propeller 1 
ydft = fft(noise(:,1));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 1'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(noise(:,1));
L = length(noise(:,1));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 1'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

[noise_1_STFT, L_time_frames, frequencies] = ANC_time_to_TF(noise(:,1), fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_1_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), 
ylabel('Frequency, k (Hz)'); 
title('Spectrogram of ego-noise in propeller 1');

% Propeller 2 
noise(:,2) = 3 * noise(:,2);
ydft = fft(noise(:,2));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 2'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(noise(:,2));
L = length(noise(:,2));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 2'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

[noise_2_STFT, L_time_frames, frequencies] = ANC_time_to_TF(noise(:,2), fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_2_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)'), title('Spectrogram of ego-noise in propeller 2');

% Propeller 3 
ydft = fft(noise(:,3));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 3'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(noise(:,3));
L = length(noise(:,3));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 3'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

[noise_3_STFT, L_time_frames, frequencies] = ANC_time_to_TF(noise(:,3), fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_3_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)'), title('Spectrogram of ego-noise in propeller 3');

% Propeller 4
ydft = fft(noise(:,4));
n = length(ydft);
f = (0:n-1)*(fs_n/n);
power = 10*log(abs(ydft).^2/n);

figure; semilogx(f, power); 
title('Frequency response graph of ego-noise in propeller 4'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(noise(:,4));
L = length(noise(:,4));
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); 
title('Single-Sided Amplitude Spectrum of ego-noise in propeller 4'); 
xlabel('f (Hz)'); ylabel('|P1(f)|')

[noise_4_STFT, L_time_frames, frequencies] = ANC_time_to_TF(noise(:,4), fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(noise_4_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l');
ylabel('Frequency, k (Hz)'), title('Spectrogram of ego-noise in propeller 3');

%% Speech implementation
[speech,fs] = audioread('speech_male_count.wav');
speech_ds = downsample(speech, factor);
speech_ds = speech_ds(1:length(noise))/120;
speech_ds_before = speech_ds;
speech_ds = filter(lpB1000,speech_ds);

j = 1;
speech_error = error;
speech_error_before = error_before;
for i = 1:length(error_before)-1
    speech_error(i) = speech_error(i) + speech_ds(j);
    speech_error_before(i) = speech_error_before(i) + speech_ds_before(j);
    j = j + 1;
    if j == length(speech_ds)
        j = 1;
    end
end
% sound(speech_error, fs_n);

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
figure;
plot(SNR);
xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',round(xt/5.9));
set(gcf,'color','w'), set(gca,'Fontsize',15); 
xlabel('Time (s)', 'Fontsize',15');
ylabel('SNR (dB)','Fontsize',15);

%% Initialise variables
% Signal comming out of the loudspeaker 
loudspeaker_sound = zeros(length(noise), 1);

% Signal catched by the error microphone -> this is the error ! 
error_microphone_signal = zeros(length(noise), 1);

% System parameters 
order = 13;
filter_coefficients_1 = -0.5 + rand(order,1);
filter_coefficients_2 = -0.5 + rand(order,1);
filter_coefficients_3 = -0.5 + rand(order,1);
filter_coefficients_4 = -0.5 + rand(order,1);

coef_length = length(filter_coefficients_1);

% Zero padded propeller_noise
error_zp = [zeros(coef_length,1);error];
speech_error_zp = [zeros(coef_length,1);speech_error];
error_noise_zp = [zeros(coef_length,1);error_noise];
noise_zp = [zeros(coef_length,4);noise];

% LMS algorithm coefficients
step_size = 0.8;

%% ANC Algorithm
% for loop running over all samples 
t = 1; % Determine the update rate 
tic
for i = coef_length:length(speech_error_zp)
    % Calculation of error_microphone_signal 
        % FIR filter the propeller_noise with filter coefficients 
            loudspeaker_sound(i) = filter_coefficients_1' * noise_zp((i-(coef_length-1)):i,1) ...
                + filter_coefficients_2' * noise_zp((i-(coef_length-1)):i,2) ...
                + filter_coefficients_3' * noise_zp((i-(coef_length-1)):i,3) ...
                + filter_coefficients_4' * noise_zp((i-(coef_length-1)):i,4);
    
        % Take the inverse of the loudspeaker_noise
            loudspeaker_sound(i) = loudspeaker_sound(i)*(-1);
            
        % Error_microphone signal is the sum of propeller and loudspeaker noise
            error_microphone_signal(i) = speech_error_zp(i) + loudspeaker_sound(i);
            %error_microphone_signal(i) = error_zp(i) + loudspeaker_sound(i);
        if t == 1
    % Filter coefficients update
        % 2nd norm of the propeller_noise         
            norm_1 = (sum(noise_zp((i-(coef_length-1)):i,1).*noise_zp((i-(coef_length-1)):i,1)));            
            norm_2 = (sum(noise_zp((i-(coef_length-1)):i,2).*noise_zp((i-(coef_length-1)):i,2)));
            norm_3 = (sum(noise_zp((i-(coef_length-1)):i,3).*noise_zp((i-(coef_length-1)):i,3)));
            norm_4 = (sum(noise_zp((i-(coef_length-1)):i,4).*noise_zp((i-(coef_length-1)):i,4)));
    
        % Update filter coefficients 
            p_1 = noise_zp((i-(coef_length-1)):i,1)/(norm_1+1);
            p_2 = noise_zp((i-(coef_length-1)):i,2)/(norm_2+1);
            p_3 = noise_zp((i-(coef_length-1)):i,3)/(norm_3+1);
            p_4 = noise_zp((i-(coef_length-1)):i,4)/(norm_4+1);
            
            f_1 = step_size * p_1 * error_microphone_signal(i);
            f_2 = step_size * p_2 * error_microphone_signal(i);
            f_3 = step_size * p_3 * error_microphone_signal(i);
            f_4 = step_size * p_4 * error_microphone_signal(i);
            
            filter_coefficients_1 = filter_coefficients_1 + f_1;            
            filter_coefficients_2 = filter_coefficients_2 + f_2;
            filter_coefficients_3 = filter_coefficients_3 + f_3;
            filter_coefficients_4 = filter_coefficients_4 + f_4;  
            
            t = 0;
        end
        t = t+1;
end
toc

%sound(error_microphone_signal*5,fs_n);

%% Spectogram and frequency plot microphone signal before the algorithm
microphone_signal_dft = fft(error);
n_end = length(microphone_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(microphone_signal_dft).^2/n_end);

figure; semilogx(f_end, power); 
title('Frequency response graph error microphone signal before ANC'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(error);
L = length(error);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

%figure; plot(f,P1); title('Single-Sided Amplitude Spectrum of error microphone signal before ANC'); xlabel('f (Hz)'); ylabel('|P1(f)|')
figure; plot(f,P1); set(gca,'Fontsize',15); 
xlabel('Frequency (Hz)','FontSize',15); ylabel('Amplitude','FontSize',15); 
ylim([0 0.0003]);

[error_STFT, L_time_frames, frequencies] = ANC_time_to_TF(error, fs_n);
figure; 
imagesc(1:L_time_frames, frequencies, mag2db(abs(error_STFT(:,:,1))), [-65, 10]); 
colorbar; axis xy; 
xt = get(gca, 'XTick');
set(gca, 'XTick',xt, 'XTickLabel',round(xt*512/8820));
set(gcf,'color','w'), set(gca,'Fontsize',15), 
xlabel('Time (s)', 'Fontsize',15'), 
ylabel('Frequency, k (Hz)','Fontsize',15);

%% Spectogram and frequency plot microphone signal after the algorithm
microphone_signal_dft = fft(error_microphone_signal);
n_end = length(microphone_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(microphone_signal_dft).^2/n_end);

figure; semilogx(f_end, power); 
title('Frequency response graph error microphone signal'); 
xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(error_microphone_signal);
L = length(error_microphone_signal);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

%figure; plot(f,P1); title('Single-Sided Amplitude Spectrum of error microphone signal after ANC'); xlabel('f (Hz)'); ylabel('|P1(f)|')
figure; plot(f,P1); set(gca,'Fontsize',15); xlabel('f (Hz)','FontSize',15); 
ylabel('|P1(f)|','FontSize',15); ylim([0 0.0003]);

[error_microphone_signal_STFT, L_time_frames, frequencies] = ANC_time_to_TF(error_microphone_signal, fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(error_microphone_signal_STFT(:,:,1))), [-65, 10]); 
colorbar; axis xy; set(gcf,'color','w'), set(gca,'Fontsize',15);
xlabel('Time Frames, l','FontSize',15), ylabel('Frequency, k (Hz)','FontSize',15);% title('Spectrogram of Error microphone after ANC');

%% Spectogram and frequency plot loudspeaker signal after the algorithm
loudspeaker_signal_dft = fft(loudspeaker_sound);
n_end = length(loudspeaker_signal_dft);
f_end = (0:n_end-1)*(fs_n/n_end);
power = 10*log(abs(loudspeaker_signal_dft).^2/n_end);

figure; semilogx(f_end, power); title('Frequency response graph loudspeaker signal'); xlabel('Frequency(Hz)'); ylabel('Power(dB)');

y_freq = fft(loudspeaker_sound);
L = length(loudspeaker_sound);
P2 = abs(y_freq/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);
f = fs_n*(0:(L/2))/L;

figure; plot(f,P1); title('Single-Sided Amplitude Spectrum of the loudspeaker'); xlabel('f (Hz)'); ylabel('|P1(f)|')

[loudspeaker_sound_STFT, L_time_frames, frequencies] = ANC_time_to_TF(loudspeaker_sound, fs_n);
figure; imagesc(1:L_time_frames, frequencies, mag2db(abs(loudspeaker_sound_STFT(:,:,1))), [-65, 20]); 
colorbar; axis xy; set(gcf,'color','w'), xlabel('Time Frames, l'), ylabel('Frequency, k (Hz)'), title('Spectrogram of Loudspeaker noise');

%% Plots
% Original signal // Counter signal // Output signal 
figure; 
%subplot(1,3,1); plot(-error_zp, 'g'); title('Ideal loudspeaker noise'); ylim([-0.1 0.1])
%subplot(1,3,2); plot(loudspeaker_sound, 'r'); title('Loudspeaker noise'); ylim([-0.1 0.1])
plot(error_microphone_signal);
ylim([-0.1 0.1]); 
xlabel('Timeframes (s)', 'FontSize', 15); ylabel('Amplitude', 'FontSize', 15); title('Signal prediction using NLMS');
audiowrite('audio2.wav', error_microphone_signal, fs_n)

%% SNR after ANC
% Signal = e = s + n + l;
%temp = lowpass(error_microphone_signal, 750, fs_n);
noise_only = error_zp + loudspeaker_sound;
signal = error_microphone_signal - noise_only;
%signal = temp - noise_only;

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
save('SNR2.mat', 'SNR_after');
figure
hold on
plot(SNR);
plot(SNR_after);
xlabel('Time Frames', 'FontSize', 15);
ylabel('SNR (dB)', 'FontSize', 15)
legend('before', 'after');
hold off

mean(SNR_after(end-100:end) - SNR(end-100:end))
%sound(error_microphone_signal*5, fs_n);
