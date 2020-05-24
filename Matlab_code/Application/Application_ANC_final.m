classdef Application_ANC_final < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                        matlab.ui.Figure
        TextArea                        matlab.ui.control.TextArea
        ParameterControlPanel           matlab.ui.container.Panel
        TimeFrequencydomainparametersLabel  matlab.ui.control.Label
        GeneralparametersLabel          matlab.ui.control.Label
        ChoosetheaudiofileLabel         matlab.ui.control.Label
        audiofile                       matlab.ui.control.DropDown
        SamplingrateDropDownLabel       matlab.ui.control.Label
        samplingrate                    matlab.ui.control.DropDown
        NumberoffiltercoefficientsSpinnerLabel  matlab.ui.control.Label
        filtercoef                      matlab.ui.control.Spinner
        WindowDropDownLabel             matlab.ui.control.Label
        window                          matlab.ui.control.DropDown
        NumberofFFTSpinnerLabel         matlab.ui.control.Label
        fft                             matlab.ui.control.Spinner
        StepsizeLabel                   matlab.ui.control.Label
        stepsize2                       matlab.ui.control.Spinner
        ErrorapproachbeforeANCDropDownLabel  matlab.ui.control.Label
        ErrorapproachbeforeANCDropDown  matlab.ui.control.DropDown
        sounderror2_2                   matlab.ui.control.Button
        SpeechDropDownLabel             matlab.ui.control.Label
        SpeechDropDown                  matlab.ui.control.DropDown
        sounderror2_3                   matlab.ui.control.Button
        sounderror2_4                   matlab.ui.control.Button
        AudiosetupLabel                 matlab.ui.control.Label
        DomainControlPanel              matlab.ui.container.Panel
        DomainDropDownLabel             matlab.ui.control.Label
        DomainDropDown                  matlab.ui.control.DropDown
        run                             matlab.ui.control.Button
        AdaptivefilterupdateDropDownLabel  matlab.ui.control.Label
        AdaptivefilterupdateDropDown    matlab.ui.control.DropDown
        SNRPanel                        matlab.ui.container.Panel
        snr                             matlab.ui.control.UIAxes
        SNRdifferenceEditFieldLabel_2   matlab.ui.control.Label
        DifferenceinSNRTextAreaLabel    matlab.ui.control.Label
        DifferenceinSNRTextArea         matlab.ui.control.TextArea
        EgonoiseinReferencemicrophonesPanel  matlab.ui.container.Panel
        TabGroup                        matlab.ui.container.TabGroup
        PlotTab                         matlab.ui.container.Tab
        p1                              matlab.ui.control.UIAxes
        SpectrogramTab                  matlab.ui.container.Tab
        sp1                             matlab.ui.control.UIAxes
        AmplitudespectrumTab            matlab.ui.container.Tab
        ap1                             matlab.ui.control.UIAxes
        FrequencyresponsTab             matlab.ui.container.Tab
        fp1                             matlab.ui.control.UIAxes
        EgonoiseinErrorMicrophonebeforeANCPanel  matlab.ui.container.Panel
        TabGroup_2                      matlab.ui.container.TabGroup
        PlotTab_2                       matlab.ui.container.Tab
        e                               matlab.ui.control.UIAxes
        SpectrogramTab_2                matlab.ui.container.Tab
        se                              matlab.ui.control.UIAxes
        AmplitudeTab_2                  matlab.ui.container.Tab
        ae                              matlab.ui.control.UIAxes
        FrequencyTap_2                  matlab.ui.container.Tab
        fe                              matlab.ui.control.UIAxes
        EgonoiseinErrorMicrophoneafterANCPanel_2  matlab.ui.container.Panel
        TabGroup_3                      matlab.ui.container.TabGroup
        PlotTab_3                       matlab.ui.container.Tab
        em                              matlab.ui.control.UIAxes
        SpectrogramTab_3                matlab.ui.container.Tab
        sem                             matlab.ui.control.UIAxes
        AmplitudespectrumTab_3          matlab.ui.container.Tab
        aem                             matlab.ui.control.UIAxes
        FrequencyresponsTab_3           matlab.ui.container.Tab
        fem                             matlab.ui.control.UIAxes
        NoisecomingfromloudspeakerPanel_2  matlab.ui.container.Panel
        TabGroup_4                      matlab.ui.container.TabGroup
        PlotTab_4                       matlab.ui.container.Tab
        l                               matlab.ui.control.UIAxes
        SpectrogramTab_4                matlab.ui.container.Tab
        sl                              matlab.ui.control.UIAxes
        AmplitudespectrumTab_4          matlab.ui.container.Tab
        al                              matlab.ui.control.UIAxes
        FrequencyresponsTab_4           matlab.ui.container.Tab
        fl                              matlab.ui.control.UIAxes
    end

    methods (Access = private)

        % Value changed function: audiofile
        function audiofileValueChanged(app, event)
            % Add the correct path 
            % addpath(genpath(fullfile('C:\Users\Shana Reynders\OneDrive\Thesis\Matlab_files\audio_files')));
%--------------------------------------------------------------------------------------------------------------------
            % Choose the audio files that you want to use 
            audio = app.audiofile.Value;
            
            if audio == "Noise only"
                [y1] = audioread('noise_only_closest.wav');
                [y2] = audioread('noise_only_close.wav');
                [y3] = audioread('noise_only_further.wav');
                [y4] = audioread('noise_only_furthest.wav');
                [errorsig] = audioread('noise_only_error_mic.wav'); 
            elseif audio == "Noise + speech counting"
                [y1] = audioread('count_dichtste_5_03.wav');
                [y2] = audioread('count_ver_5_03.wav');
                [y3] = audioread('count_verder_5_03.wav');
                [y4] = audioread('count_verste_5_03.wav');
                [errorsig] = audioread('count_mic_5_03.wav'); 
            end
            
%--------------------------------------------------------------------------------------------------------------------          
            % Concatenate all the files
            noise = horzcat(y1, y2, y3, y4);
            noise_original = y1 + y2 + y3 + y4;
            H2_1 = 44; H2_2 = 62; H2_3 = 53; H2_4 = 49;
            y1 = [zeros(H2_1,1); y1(1:end-H2_1)];
            y2 = [zeros(H2_2,1); y2(1:end-H2_2)];
            y3 = [zeros(H2_3,1); y3(1:end-H2_3)];
            y4 = [zeros(H2_4,1); y4(1:end-H2_4)];

            noise_error = y1 + y2 + y3 + y4;
%-------------------------------------------------------------------------------------------------------------------- 
            % Lowpass
            load('lowpassB1000.mat', 'lpB1000');
            noise = filter(lpB1000,noise);
            errorsig = filter(lpB1000,errorsig);
%-------------------------------------------------------------------------------------------------------------------- 
            % Pass on the data 
            setappdata(0, 'noise_original', noise_original);
            setappdata(0, 'noise', noise);
            setappdata(0, 'noise_error', noise_error);
            setappdata(0, 'errorsig', errorsig);
        end

        % Button pushed function: run
        function runButtonPushed(app, event)
            % Received data 
            number_fil_coef = getappdata(0,'number_of_filter');
            noise_total = getappdata(0,'prop_noise_total');
            noise = getappdata(0, 'noise_after_ds');
            errorsig = getappdata(0,'error');
            step_size = getappdata(0,'step_size');
            fs_pn = getappdata(0, 'fs_pn');
            numberoffft = getappdata(0, 'numberoffft');
            SNR_before = getappdata(0, 'SNR_before');
            error_only_down = getappdata(0, 'error_only_down');
%--------------------------------------------------------------------------------------------------------------------            
            % Read the domain 
            domain = app.DomainDropDown.Value;
%--------------------------------------------------------------------------------------------------------------------            
            % ANC algorithm 
            if domain == "Time domain" 
                [loudspeaker_noise, error_mic] = ANC_t_multichannel(number_fil_coef, noise, errorsig, step_size);
                
                % Spectrogram error
                [signal_error_STFT, L_time_frames, frequencies] = ANC_T_to_TF(error_mic, fs_pn);
                imagesc(app.sem, 1:L_time_frames, frequencies, mag2db(abs(signal_error_STFT(:,:,1))), [-65, 20]);
                
                % Spectrogram loudspeaker_noise
                [signal_loudspeaker_noise_STFT, L_time_frames, frequencies] = ANC_T_to_TF(loudspeaker_noise, fs_pn);
                imagesc(app.sl, 1:L_time_frames, frequencies, mag2db(abs(signal_loudspeaker_noise_STFT(:,:,1))), [-65, 20]);  
                   
            elseif domain == "Time-frequency domain" 
                [loudspeaker_noise, error_mic, loudspeaker_noise_STFT, error_STFT, L_time_frames, frequencies] = ANC_tf_multichannel(number_fil_coef,noise,errorsig, step_size, numberoffft, fs_pn)
               
                imagesc(app.sem, 1:L_time_frames, frequencies, mag2db(abs(error_STFT(:,:,1))), [-65, 20]);  
                imagesc(app.sl, 1:L_time_frames, frequencies, mag2db(abs(loudspeaker_noise_STFT(:,:,1))), [-65, 20]); 
            end
%--------------------------------------------------------------------------------------------------------------------             
            % Plot the error microphone signal 
            plot(app.em,error_mic);
            
            % amplitude plot of the error microphone
            microphone_signal_dft = fft(error_mic(end-fs_pn*3:end, :));
            L_error = length(error_mic(end-fs_pn*3:end, :));
            P2_error = abs(microphone_signal_dft/L_error);
            P1_error = P2_error(1:L_error/2+1);
            P1_error(2:end-1) = 2*P1_error(2:end-1);
            
            f_error = fs_pn*(0:(L_error/2))/L_error;
            
            plot(app.aem, f_error, P1_error);
            
            % Frequency
            ydft = fft(error_mic);
            n = length(ydft);
            f = (0:n-1)*(fs_pn/n);
            power = 10*log(abs(ydft).^2/n);
            
            semilogx(app.fem, f, power);
%--------------------------------------------------------------------------------------------------------------------            
            % Plot the loudspeaker_noise signal 
            plot(app.l,loudspeaker_noise);
         
            % Amplitude plot of the loudspeaker_noise signal 
            loudspeaker_noise_signal_dft = fft(loudspeaker_noise);
            L = length(loudspeaker_noise);
            P2 = abs(loudspeaker_noise_signal_dft/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);            
            f = fs_pn*(0:(L/2))/L;
            
            semilogx(app.al, f, P1);
            
            % Frequency
            ydft = fft(loudspeaker_noise);
            n = length(ydft);
            f = (0:n-1)*(fs_pn/n);
            power = 10*log(abs(ydft).^2/n);
            
            semilogx(app.fl, f, power);
%--------------------------------------------------------------------------------------------------------------------              
            % SNR after 
            cla(app.snr);
            
            if length(error_only_down) > length(loudspeaker_noise) 
                 noise_only = error_only_down(1:length(loudspeaker_noise)) - loudspeaker_noise;
            else
                 noise_only = [zeros(length(loudspeaker_noise)-length(error_only_down),1);error_only_down] + loudspeaker_noise;

            end
            signal = error_mic - noise_only;
            
            k = 1;
            overlap = 0.8;
            frame_size = round(length(noise(:,1))/50);
            for i = number_fil_coef : round(frame_size*(1-overlap)) : length(noise(:,1))-(frame_size-1)
                signal_power(k) = mean((signal(i : i+(frame_size-1))).^2);
                noise_power(k) = mean((noise_only(i : i+(frame_size-1))).^2);
                k = k + 1;
            end
            SNR_after = pow2db((signal_power)./noise_power);
                
            plot(app.snr, SNR_before);
            plot(app.snr, SNR_after);
                        
            mean_difference = mean(SNR_after(end-100:end) - SNR_before(end-100:end))   
            app.DifferenceinSNRTextArea.Value = num2str(mean_difference);

            %set(app.DifferenceinSNRTextArea, 'String', num2str(mean_difference));
%--------------------------------------------------------------------------------------------------------------------
            % Pass on the data 
            setappdata(0, 'error_microphone_signal', error_mic);
            setappdata(0, 'loudspeaker_noise', loudspeaker_noise);
        end

        % Value changed function: samplingrate
        function samplingrateValueChanged(app, event)
            % Received data
            noise = getappdata(0, 'noise');
            errorsig = getappdata(0, 'errorsignew');
            speech = getappdata(0, 'speech');
            error_only = getappdata(0, 'errorsig');
%--------------------------------------------------------------------------------------------------------------------            
            % Read the sampling rate  
            samplerate = app.samplingrate.Value;
            
            if samplerate == "4.41 kHz"
                factor = 10;
                fs_down = 4410;
            elseif samplerate == "22 kHz"
                factor = 2;
                fs_down = 22050;
            elseif samplerate == "2.2 kHz"
                factor = 20;
                fs_down = 2205;
            elseif samplerate == "8.82 kHz"
                factor = 5;
                fs_down = 8820;           
            elseif samplerate == "44.1 kHz"
                factor = 1;
                fs_pn = 44100;       
            end
            noise_down = zeros(round(length(noise)/factor),4);
            error_down = zeros(round(length(noise)/factor),1);
            speech_down = zeros(round(length(noise)/factor),1);
            error_only_down = zeros(round(length(noise)/factor),1);
            j = 1;
            for i = 1:factor:length(noise(:,1))  
                noise_down(j,:) = noise(i,:);
                error_down(j,1) = errorsig(i,1);
                speech_down(j,1) = speech(i,1);
                error_only_down(j,1) = error_only(i,1);
                j = j + 1;
            end
            fs_pn = fs_down;
            noise = noise_down;
            errorsig = error_down;
            speech = speech_down;
            error_only = error_only_down;
%--------------------------------------------------------------------------------------------------------------------            
            noise_total = noise(:,1) + noise(:,2) + noise(:,3) + noise(:,4);
%--------------------------------------------------------------------------------------------------------------------              
            % Plot the propeller noise 
            plot(app.p1,noise(:,1));
%--------------------------------------------------------------------------------------------------------------------              
            % Amplitude spectrum
            y_freq = fft(noise(:,1));
            L = length(noise(:,1));
            P2 = abs(y_freq/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);
            f = fs_pn*(0:(L/2))/L;
            
            plot(app.ap1, f, P1);
%--------------------------------------------------------------------------------------------------------------------             
            % noise graph
            ydft = fft(noise(:,1));
            n = length(ydft);
            f = (0:n-1)*(fs_pn/n);
            power = 10*log(abs(ydft).^2/n);
            
            semilogx(app.fp1, f, power);
%--------------------------------------------------------------------------------------------------------------------              
            % Spectrogram of the propeller noise 
            [signal_STFT, L_time_frames, frequencies] = ANC_T_to_TF(noise(:,1), fs_pn);
            imagesc(app.sp1, 1:L_time_frames, frequencies, mag2db(abs(signal_STFT(:,:,1))), [-65, 20]); 
            
%-------------------------------------------------------------------------------------------------------------------- 
            % Plot
            plot(app.e, errorsig); 
            
            % Spectrogram
            [signal_STFT, L_time_frames, frequencies] = ANC_T_to_TF(errorsig, fs_pn);
            imagesc(app.se, 1:L_time_frames, frequencies, mag2db(abs(signal_STFT(:,:,1))), [-65, 20]); 
            
            % Amplitude plot
            y_freq = fft(errorsig);
            L = length(errorsig);
            P2 = abs(y_freq/L);
            P1 = P2(1:L/2+1);
            P1(2:end-1) = 2*P1(2:end-1);
            f = fs_pn*(0:(L/2))/L;
            
            plot(app.ae, f, P1);
            
            % Frequency plot
            ydft = fft(errorsig);
            n = length(ydft);
            f = (0:n-1)*(fs_pn/n);
            power = 10*log(abs(ydft).^2/n);
            
            semilogx(app.fe, f, power);
            
%--------------------------------------------------------------------------------------------------------------------
            % SNR before 
            k = 1;
            overlap = 0.8;
            frame_size = round(length(noise(:,1))/50);
            for i = 1 : round(frame_size*(1-overlap)) : length(noise(:,1))-(frame_size-1)
                signal_power(k) = mean((speech(i : i+(frame_size-1))).^2); 
                noise_power(k) = mean((errorsig(i : i+(frame_size-1))).^2);
                k = k + 1;
            end
            SNR_before = pow2db(signal_power./noise_power);
%--------------------------------------------------------------------------------------------------------------------
            % Pass on the data
            setappdata(0,'prop_noise_total',noise_total);
            setappdata(0,'noise_after_ds', noise);
            setappdata(0,'error',errorsig);
            setappdata(0, 'fs_pn', fs_pn);
            setappdata(0, 'SNR_before', SNR_before);
            setappdata(0, 'error_only_down', error_only);
        end

        % Callback function
        function stepsizeValueChanged(app, event)
            step_size =  app.stepsize.Value;
            setappdata(0, 'step_size', step_size);
        end

        % Callback function
        function soundpropButtonPushed(app, event)
            fs_pn = getappdata(0, 'fs_pn');
            noise_total = getappdata(0,'prop_noise_total');
            
            sound(noise_total,fs_pn);
        end

        % Value changed function: filtercoef
        function filtercoefValueChanged(app, event)
            number_of_filter = app.filtercoef.Value;
            setappdata(0, 'number_of_filter', number_of_filter);
        end

        % Value changed function: fft
        function fftValueChanged(app, event)
            numberoffft = app.fft.Value;
            setappdata(0, 'numberoffft', numberoffft);
        end

        % Value changed function: window
        function windowValueChanged(app, event)
            hannwindow = app.window.Value;
            if hannwindow == "Hann window"
                %hann_window = sqrt(hann(numberoffft,'periodic'));
            end
            setappdata(0, 'window', hannwindow);
        end

        % Value changed function: stepsize2
        function stepsize2ValueChanged(app, event)
            step_size =  app.stepsize2.Value;
            setappdata(0, 'step_size', step_size);
        end

        % Callback function
        function soundloud2ButtonPushed2(app, event)
            fs_pn = getappdata(0, 'fs_pn');
            loudspeaker_noise = getappdata(0,'loudspeaker_noise');
            
            sound(loudspeaker_noise,fs_pn);
        end

        % Callback function
        function sounderror2ButtonPushed(app, event)
            fs_pn = getappdata(0, 'fs_pn');
            error_microphone_signal = getappdata(0,'error_microphone_signal');
            
            sound(error_microphone_signal,fs_pn);
        end

        % Button pushed function: sounderror2_2
        function sounderror2_2ButtonPushed(app, event)
            fs_pn = getappdata(0, 'fs_pn');
            errorsig = getappdata(0, 'errorsignew');
            
            sound(errorsig, fs_pn);
        end

        % Value changed function: ErrorapproachbeforeANCDropDown
        function ErrorapproachbeforeANCDropDownValueChanged(app, event)
            approach = app.ErrorapproachbeforeANCDropDown.Value;
            
            noise_error = getappdata(0, 'noise_error');
            errorsig = getappdata(0, 'errorsig');
            speech = getappdata(0, 'speech');
            fs_pn = getappdata(0, 'fs_pn');
            
            if approach == "Real recording"
                speech_noise = errorsig;
            else
                speech_noise = noise_error;
            end
            
            j = 1;
            for i = 1:length(errorsig)-1
                speech_noise(i) = speech_noise(i) + speech(j);
                j = j + 1;
                if j == length(speech)
                    j = 1;
                end
            end
            speech_noise = lowpass(speech_noise, 1000, fs_pn);
            %speech_ds = lowpass(speech_ds, 1000, fs_n);
            
            errorsig_new = speech_noise; 
%-------------------------------------------------------------------------------------------------------------------- 
            
            setappdata(0, 'errorsignew', errorsig_new);
        end

        % Value changed function: SpeechDropDown
        function SpeechDropDownValueChanged(app, event)
            speech_audio = app.SpeechDropDown.Value;

            if speech_audio == "Counting"
                [speech] = audioread('speech_male_count.wav');
            elseif speech_audio == "No speech"
                noise_error = getappdata(0, 'noise_error');
                [speech] = zeros(length(noise_error),1);
            end
            
            setappdata(0,'speech', speech/100);
        end

        % Button pushed function: sounderror2_4
        function sounderror2_4ButtonPushed(app, event)
            noise = getappdata(0, 'noise_original');
            
            sound(noise, 44100);
        end

        % Button pushed function: sounderror2_3
        function sounderror2_3ButtonPushed(app, event)
            speech = getappdata(0, 'speech');
            
            sound(speech, 44100);
        end
    end

    % App initialization and construction
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure
            app.UIFigure = uifigure;
            app.UIFigure.Color = [1 0.9882 0.9882];
            app.UIFigure.Position = [100 100 1371 866];
            app.UIFigure.Name = 'UI Figure';

            % Create TextArea
            app.TextArea = uitextarea(app.UIFigure);
            app.TextArea.HorizontalAlignment = 'center';
            app.TextArea.FontSize = 33;
            app.TextArea.FontWeight = 'bold';
            app.TextArea.FontColor = [1 1 1];
            app.TextArea.BackgroundColor = [0.4314 0.0118 0.0902];
            app.TextArea.Position = [0 823 1372 44];
            app.TextArea.Value = {'ANC Algorithm'};

            % Create ParameterControlPanel
            app.ParameterControlPanel = uipanel(app.UIFigure);
            app.ParameterControlPanel.ForegroundColor = [0.1608 0.2196 0.3412];
            app.ParameterControlPanel.TitlePosition = 'centertop';
            app.ParameterControlPanel.Title = 'Parameter Control';
            app.ParameterControlPanel.BackgroundColor = [1 1 1];
            app.ParameterControlPanel.FontWeight = 'bold';
            app.ParameterControlPanel.FontSize = 18;
            app.ParameterControlPanel.Position = [14 431 413 385];

            % Create TimeFrequencydomainparametersLabel
            app.TimeFrequencydomainparametersLabel = uilabel(app.ParameterControlPanel);
            app.TimeFrequencydomainparametersLabel.BackgroundColor = [1 0.9098 0.9294];
            app.TimeFrequencydomainparametersLabel.FontSize = 16;
            app.TimeFrequencydomainparametersLabel.FontWeight = 'bold';
            app.TimeFrequencydomainparametersLabel.FontColor = [0.149 0.149 0.149];
            app.TimeFrequencydomainparametersLabel.Position = [9 73 390 22];
            app.TimeFrequencydomainparametersLabel.Text = 'Time-Frequency domain parameters ';

            % Create GeneralparametersLabel
            app.GeneralparametersLabel = uilabel(app.ParameterControlPanel);
            app.GeneralparametersLabel.BackgroundColor = [1 0.9098 0.9294];
            app.GeneralparametersLabel.FontSize = 16;
            app.GeneralparametersLabel.FontWeight = 'bold';
            app.GeneralparametersLabel.FontColor = [0.149 0.149 0.149];
            app.GeneralparametersLabel.Position = [12 192 390 22];
            app.GeneralparametersLabel.Text = 'General parameters';

            % Create ChoosetheaudiofileLabel
            app.ChoosetheaudiofileLabel = uilabel(app.ParameterControlPanel);
            app.ChoosetheaudiofileLabel.Position = [24 293 125 22];
            app.ChoosetheaudiofileLabel.Text = 'Choose the audio file :';

            % Create audiofile
            app.audiofile = uidropdown(app.ParameterControlPanel);
            app.audiofile.Items = {' ', 'Noise only', 'Noise + speech counting'};
            app.audiofile.ValueChangedFcn = createCallbackFcn(app, @audiofileValueChanged, true);
            app.audiofile.Position = [175 293 136 22];
            app.audiofile.Value = ' ';

            % Create SamplingrateDropDownLabel
            app.SamplingrateDropDownLabel = uilabel(app.ParameterControlPanel);
            app.SamplingrateDropDownLabel.Position = [24 161 125 22];
            app.SamplingrateDropDownLabel.Text = 'Sampling rate :';

            % Create samplingrate
            app.samplingrate = uidropdown(app.ParameterControlPanel);
            app.samplingrate.Items = {'44.1 kHz', '22 kHz', '8.82 kHz', '4.41 kHz', '2.2 kHz'};
            app.samplingrate.ValueChangedFcn = createCallbackFcn(app, @samplingrateValueChanged, true);
            app.samplingrate.Position = [175 161 225 22];
            app.samplingrate.Value = '44.1 kHz';

            % Create NumberoffiltercoefficientsSpinnerLabel
            app.NumberoffiltercoefficientsSpinnerLabel = uilabel(app.ParameterControlPanel);
            app.NumberoffiltercoefficientsSpinnerLabel.Position = [24 132 151 22];
            app.NumberoffiltercoefficientsSpinnerLabel.Text = 'Number of filter coefficients';

            % Create filtercoef
            app.filtercoef = uispinner(app.ParameterControlPanel);
            app.filtercoef.Limits = [0 Inf];
            app.filtercoef.ValueChangedFcn = createCallbackFcn(app, @filtercoefValueChanged, true);
            app.filtercoef.HorizontalAlignment = 'left';
            app.filtercoef.Position = [174 132 225 22];

            % Create WindowDropDownLabel
            app.WindowDropDownLabel = uilabel(app.ParameterControlPanel);
            app.WindowDropDownLabel.Position = [23 14 48 22];
            app.WindowDropDownLabel.Text = 'Window';

            % Create window
            app.window = uidropdown(app.ParameterControlPanel);
            app.window.Items = {' ', 'Hann window'};
            app.window.ValueChangedFcn = createCallbackFcn(app, @windowValueChanged, true);
            app.window.Position = [173 14 225 22];
            app.window.Value = ' ';

            % Create NumberofFFTSpinnerLabel
            app.NumberofFFTSpinnerLabel = uilabel(app.ParameterControlPanel);
            app.NumberofFFTSpinnerLabel.Position = [23 43 87 22];
            app.NumberofFFTSpinnerLabel.Text = 'Number of FFT';

            % Create fft
            app.fft = uispinner(app.ParameterControlPanel);
            app.fft.Step = 2;
            app.fft.Limits = [2 1024];
            app.fft.ValueChangedFcn = createCallbackFcn(app, @fftValueChanged, true);
            app.fft.HorizontalAlignment = 'left';
            app.fft.Position = [173 43 225 22];
            app.fft.Value = 2;

            % Create StepsizeLabel
            app.StepsizeLabel = uilabel(app.ParameterControlPanel);
            app.StepsizeLabel.Position = [24 103 58 22];
            app.StepsizeLabel.Text = 'Step size ';

            % Create stepsize2
            app.stepsize2 = uispinner(app.ParameterControlPanel);
            app.stepsize2.Step = 0.05;
            app.stepsize2.Limits = [0 5];
            app.stepsize2.ValueChangedFcn = createCallbackFcn(app, @stepsize2ValueChanged, true);
            app.stepsize2.HorizontalAlignment = 'left';
            app.stepsize2.Position = [174 103 225 22];

            % Create ErrorapproachbeforeANCDropDownLabel
            app.ErrorapproachbeforeANCDropDownLabel = uilabel(app.ParameterControlPanel);
            app.ErrorapproachbeforeANCDropDownLabel.Position = [23 225 151 22];
            app.ErrorapproachbeforeANCDropDownLabel.Text = 'Error approach before ANC';

            % Create ErrorapproachbeforeANCDropDown
            app.ErrorapproachbeforeANCDropDown = uidropdown(app.ParameterControlPanel);
            app.ErrorapproachbeforeANCDropDown.Items = {'', 'Real recording', 'Theoretical sum'};
            app.ErrorapproachbeforeANCDropDown.ValueChangedFcn = createCallbackFcn(app, @ErrorapproachbeforeANCDropDownValueChanged, true);
            app.ErrorapproachbeforeANCDropDown.Position = [176 225 135 22];
            app.ErrorapproachbeforeANCDropDown.Value = '';

            % Create sounderror2_2
            app.sounderror2_2 = uibutton(app.ParameterControlPanel, 'push');
            app.sounderror2_2.ButtonPushedFcn = createCallbackFcn(app, @sounderror2_2ButtonPushed, true);
            app.sounderror2_2.BackgroundColor = [0.4314 0 0.0902];
            app.sounderror2_2.FontSize = 18;
            app.sounderror2_2.FontColor = [1 1 1];
            app.sounderror2_2.Position = [316 221 82 30];
            app.sounderror2_2.Text = 'Sound';

            % Create SpeechDropDownLabel
            app.SpeechDropDownLabel = uilabel(app.ParameterControlPanel);
            app.SpeechDropDownLabel.Position = [24 259 46 22];
            app.SpeechDropDownLabel.Text = 'Speech';

            % Create SpeechDropDown
            app.SpeechDropDown = uidropdown(app.ParameterControlPanel);
            app.SpeechDropDown.Items = {' ', 'Counting', 'No speech'};
            app.SpeechDropDown.ValueChangedFcn = createCallbackFcn(app, @SpeechDropDownValueChanged, true);
            app.SpeechDropDown.Position = [176 259 135 22];
            app.SpeechDropDown.Value = ' ';

            % Create sounderror2_3
            app.sounderror2_3 = uibutton(app.ParameterControlPanel, 'push');
            app.sounderror2_3.ButtonPushedFcn = createCallbackFcn(app, @sounderror2_3ButtonPushed, true);
            app.sounderror2_3.BackgroundColor = [0.4314 0 0.0902];
            app.sounderror2_3.FontSize = 18;
            app.sounderror2_3.FontColor = [1 1 1];
            app.sounderror2_3.Position = [316 255 82 30];
            app.sounderror2_3.Text = 'Sound';

            % Create sounderror2_4
            app.sounderror2_4 = uibutton(app.ParameterControlPanel, 'push');
            app.sounderror2_4.ButtonPushedFcn = createCallbackFcn(app, @sounderror2_4ButtonPushed, true);
            app.sounderror2_4.BackgroundColor = [0.4314 0 0.0902];
            app.sounderror2_4.FontSize = 18;
            app.sounderror2_4.FontColor = [1 1 1];
            app.sounderror2_4.Position = [316 289 82 30];
            app.sounderror2_4.Text = 'Sound';

            % Create AudiosetupLabel
            app.AudiosetupLabel = uilabel(app.ParameterControlPanel);
            app.AudiosetupLabel.BackgroundColor = [1 0.9098 0.9294];
            app.AudiosetupLabel.FontSize = 16;
            app.AudiosetupLabel.FontWeight = 'bold';
            app.AudiosetupLabel.FontColor = [0.149 0.149 0.149];
            app.AudiosetupLabel.Position = [10 325 390 22];
            app.AudiosetupLabel.Text = 'Audio set-up';

            % Create DomainControlPanel
            app.DomainControlPanel = uipanel(app.UIFigure);
            app.DomainControlPanel.ForegroundColor = [0.1608 0.2196 0.3412];
            app.DomainControlPanel.TitlePosition = 'centertop';
            app.DomainControlPanel.Title = 'Domain Control';
            app.DomainControlPanel.BackgroundColor = [1 1 1];
            app.DomainControlPanel.FontWeight = 'bold';
            app.DomainControlPanel.FontSize = 18;
            app.DomainControlPanel.Position = [14 302 413 119];

            % Create DomainDropDownLabel
            app.DomainDropDownLabel = uilabel(app.DomainControlPanel);
            app.DomainDropDownLabel.Position = [7 65 76 22];
            app.DomainDropDownLabel.Text = 'Domain';

            % Create DomainDropDown
            app.DomainDropDown = uidropdown(app.DomainControlPanel);
            app.DomainDropDown.Items = {'Time domain', 'Time-frequency domain'};
            app.DomainDropDown.Position = [174 65 225 21];
            app.DomainDropDown.Value = 'Time domain';

            % Create run
            app.run = uibutton(app.DomainControlPanel, 'push');
            app.run.ButtonPushedFcn = createCallbackFcn(app, @runButtonPushed, true);
            app.run.BackgroundColor = [0.7686 0.0078 0.149];
            app.run.FontSize = 18;
            app.run.FontColor = [1 1 1];
            app.run.Position = [9 8 390 25];
            app.run.Text = 'RUN';

            % Create AdaptivefilterupdateDropDownLabel
            app.AdaptivefilterupdateDropDownLabel = uilabel(app.DomainControlPanel);
            app.AdaptivefilterupdateDropDownLabel.Position = [7 36 118 22];
            app.AdaptivefilterupdateDropDownLabel.Text = 'Adaptive filter update';

            % Create AdaptivefilterupdateDropDown
            app.AdaptivefilterupdateDropDown = uidropdown(app.DomainControlPanel);
            app.AdaptivefilterupdateDropDown.Items = {'NLMS', 'LMS', 'RLS', 'Kalman', ''};
            app.AdaptivefilterupdateDropDown.Position = [173 37 225 22];
            app.AdaptivefilterupdateDropDown.Value = 'NLMS';

            % Create SNRPanel
            app.SNRPanel = uipanel(app.UIFigure);
            app.SNRPanel.TitlePosition = 'centertop';
            app.SNRPanel.Title = 'SNR';
            app.SNRPanel.BackgroundColor = [1 1 1];
            app.SNRPanel.FontWeight = 'bold';
            app.SNRPanel.FontSize = 18;
            app.SNRPanel.Position = [14 12 413 281];

            % Create snr
            app.snr = uiaxes(app.SNRPanel);
            title(app.snr, 'SNR')
            xlabel(app.snr, 'Samples')
            ylabel(app.snr, 'SNR (dB)')
            app.snr.ColorOrder = [0 0.4471 0.7412;0.851 0.3255 0.098;0.9294 0.6941 0.1255;0.4941 0.1843 0.5569;0.4667 0.6745 0.1882;0.302 0.7451 0.9333;0.6353 0.0784 0.1843];
            app.snr.NextPlot = 'add';
            app.snr.BackgroundColor = [1 0.9882 0.9882];
            app.snr.Position = [12 36 387 212];

            % Create SNRdifferenceEditFieldLabel_2
            app.SNRdifferenceEditFieldLabel_2 = uilabel(app.SNRPanel);
            app.SNRdifferenceEditFieldLabel_2.HorizontalAlignment = 'right';
            app.SNRdifferenceEditFieldLabel_2.Position = [372 11 25 22];
            app.SNRdifferenceEditFieldLabel_2.Text = 'dB';

            % Create DifferenceinSNRTextAreaLabel
            app.DifferenceinSNRTextAreaLabel = uilabel(app.SNRPanel);
            app.DifferenceinSNRTextAreaLabel.HorizontalAlignment = 'right';
            app.DifferenceinSNRTextAreaLabel.Position = [109 10 162 22];
            app.DifferenceinSNRTextAreaLabel.Text = 'Difference in SNR';

            % Create DifferenceinSNRTextArea
            app.DifferenceinSNRTextArea = uitextarea(app.SNRPanel);
            app.DifferenceinSNRTextArea.Position = [281 8 92 26];

            % Create EgonoiseinReferencemicrophonesPanel
            app.EgonoiseinReferencemicrophonesPanel = uipanel(app.UIFigure);
            app.EgonoiseinReferencemicrophonesPanel.TitlePosition = 'centertop';
            app.EgonoiseinReferencemicrophonesPanel.Title = 'Ego-noise in Reference microphones';
            app.EgonoiseinReferencemicrophonesPanel.BackgroundColor = [1 1 1];
            app.EgonoiseinReferencemicrophonesPanel.FontWeight = 'bold';
            app.EgonoiseinReferencemicrophonesPanel.FontSize = 18;
            app.EgonoiseinReferencemicrophonesPanel.Position = [437 420 458 396];

            % Create TabGroup
            app.TabGroup = uitabgroup(app.EgonoiseinReferencemicrophonesPanel);
            app.TabGroup.Position = [8 10 443 347];

            % Create PlotTab
            app.PlotTab = uitab(app.TabGroup);
            app.PlotTab.Title = 'Plot';
            app.PlotTab.BackgroundColor = [1 0.9882 0.9882];
            app.PlotTab.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create p1
            app.p1 = uiaxes(app.PlotTab);
            title(app.p1, 'Ego-noise of propeller 1')
            xlabel(app.p1, 'Samples in time')
            ylabel(app.p1, 'Amplitude')
            app.p1.BackgroundColor = [1 0.9882 0.9882];
            app.p1.Position = [23 14 405 295];

            % Create SpectrogramTab
            app.SpectrogramTab = uitab(app.TabGroup);
            app.SpectrogramTab.Title = 'Spectrogram';
            app.SpectrogramTab.BackgroundColor = [1 0.9882 0.9882];
            app.SpectrogramTab.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create sp1
            app.sp1 = uiaxes(app.SpectrogramTab);
            title(app.sp1, 'Spectrogram of propeller 1')
            xlabel(app.sp1, 'Time Frames ')
            ylabel(app.sp1, 'Frequency bins')
            app.sp1.BackgroundColor = [1 0.9882 0.9882];
            app.sp1.Position = [23 15 405 295];

            % Create AmplitudespectrumTab
            app.AmplitudespectrumTab = uitab(app.TabGroup);
            app.AmplitudespectrumTab.Title = 'Amplitude spectrum';
            app.AmplitudespectrumTab.BackgroundColor = [1 0.9882 0.9882];
            app.AmplitudespectrumTab.ForegroundColor = [0.149 0.149 0.149];

            % Create ap1
            app.ap1 = uiaxes(app.AmplitudespectrumTab);
            title(app.ap1, 'Amplitude spectrum of propeller 1')
            xlabel(app.ap1, 'frequency (Hz)')
            ylabel(app.ap1, '|P(f)|')
            app.ap1.BackgroundColor = [1 0.9882 0.9882];
            app.ap1.Position = [23 15 405 295];

            % Create FrequencyresponsTab
            app.FrequencyresponsTab = uitab(app.TabGroup);
            app.FrequencyresponsTab.Title = 'Frequency respons';
            app.FrequencyresponsTab.BackgroundColor = [1 0.9882 0.9882];

            % Create fp1
            app.fp1 = uiaxes(app.FrequencyresponsTab);
            title(app.fp1, 'Frequency response of propeller 1')
            xlabel(app.fp1, 'frequency (Hz)')
            ylabel(app.fp1, 'Power (dB)')
            app.fp1.BackgroundColor = [1 0.9882 0.9882];
            app.fp1.Position = [23 15 405 295];

            % Create EgonoiseinErrorMicrophonebeforeANCPanel
            app.EgonoiseinErrorMicrophonebeforeANCPanel = uipanel(app.UIFigure);
            app.EgonoiseinErrorMicrophonebeforeANCPanel.TitlePosition = 'centertop';
            app.EgonoiseinErrorMicrophonebeforeANCPanel.Title = 'Ego-noise in Error Microphone before ANC';
            app.EgonoiseinErrorMicrophonebeforeANCPanel.BackgroundColor = [1 1 1];
            app.EgonoiseinErrorMicrophonebeforeANCPanel.FontWeight = 'bold';
            app.EgonoiseinErrorMicrophonebeforeANCPanel.FontSize = 18;
            app.EgonoiseinErrorMicrophonebeforeANCPanel.Position = [437 12 458 396];

            % Create TabGroup_2
            app.TabGroup_2 = uitabgroup(app.EgonoiseinErrorMicrophonebeforeANCPanel);
            app.TabGroup_2.Position = [8 13 443 347];

            % Create PlotTab_2
            app.PlotTab_2 = uitab(app.TabGroup_2);
            app.PlotTab_2.Title = 'Plot';
            app.PlotTab_2.BackgroundColor = [1 0.9882 0.9882];
            app.PlotTab_2.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create e
            app.e = uiaxes(app.PlotTab_2);
            title(app.e, 'Error microphone before ANC')
            xlabel(app.e, 'Samples in time')
            ylabel(app.e, 'Amplitude')
            app.e.YLim = [-0.2 0.2];
            app.e.BackgroundColor = [1 0.9882 0.9882];
            app.e.Position = [23 15 405 295];

            % Create SpectrogramTab_2
            app.SpectrogramTab_2 = uitab(app.TabGroup_2);
            app.SpectrogramTab_2.Title = 'Spectrogram';
            app.SpectrogramTab_2.BackgroundColor = [1 0.9882 0.9882];
            app.SpectrogramTab_2.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create se
            app.se = uiaxes(app.SpectrogramTab_2);
            title(app.se, 'Spectrogram error microphone before ANC')
            xlabel(app.se, 'Time frames')
            ylabel(app.se, 'Frequency bins')
            app.se.BackgroundColor = [1 0.9882 0.9882];
            app.se.Position = [23 15 405 295];

            % Create AmplitudeTab_2
            app.AmplitudeTab_2 = uitab(app.TabGroup_2);
            app.AmplitudeTab_2.Title = 'Amplitude spectrum';
            app.AmplitudeTab_2.BackgroundColor = [1 0.9882 0.9882];
            app.AmplitudeTab_2.ForegroundColor = [0.149 0.149 0.149];

            % Create ae
            app.ae = uiaxes(app.AmplitudeTab_2);
            title(app.ae, 'Amplitude spectrum of error before ANC')
            xlabel(app.ae, 'frequency (Hz)')
            ylabel(app.ae, '|P(f)|')
            app.ae.BackgroundColor = [1 0.9882 0.9882];
            app.ae.Position = [23 15 405 295];

            % Create FrequencyTap_2
            app.FrequencyTap_2 = uitab(app.TabGroup_2);
            app.FrequencyTap_2.Title = 'Frequency respons';
            app.FrequencyTap_2.BackgroundColor = [1 0.9882 0.9882];

            % Create fe
            app.fe = uiaxes(app.FrequencyTap_2);
            title(app.fe, 'Frequency response error before ANC')
            xlabel(app.fe, 'frequency (Hz)')
            ylabel(app.fe, 'Power (dB)')
            app.fe.BackgroundColor = [1 0.9882 0.9882];
            app.fe.Position = [23 15 405 295];

            % Create EgonoiseinErrorMicrophoneafterANCPanel_2
            app.EgonoiseinErrorMicrophoneafterANCPanel_2 = uipanel(app.UIFigure);
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.TitlePosition = 'centertop';
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.Title = 'Ego-noise in Error Microphone after ANC';
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.BackgroundColor = [1 1 1];
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.FontWeight = 'bold';
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.FontSize = 18;
            app.EgonoiseinErrorMicrophoneafterANCPanel_2.Position = [904 12 458 396];

            % Create TabGroup_3
            app.TabGroup_3 = uitabgroup(app.EgonoiseinErrorMicrophoneafterANCPanel_2);
            app.TabGroup_3.Position = [8 13 443 347];

            % Create PlotTab_3
            app.PlotTab_3 = uitab(app.TabGroup_3);
            app.PlotTab_3.Title = 'Plot';
            app.PlotTab_3.BackgroundColor = [1 0.9882 0.9882];
            app.PlotTab_3.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create em
            app.em = uiaxes(app.PlotTab_3);
            title(app.em, 'Error microphone after ANC')
            xlabel(app.em, 'Samples in time')
            ylabel(app.em, 'Amplitude')
            app.em.BackgroundColor = [1 0.9882 0.9882];
            app.em.Position = [25 15 398 295];

            % Create SpectrogramTab_3
            app.SpectrogramTab_3 = uitab(app.TabGroup_3);
            app.SpectrogramTab_3.Title = 'Spectrogram';
            app.SpectrogramTab_3.BackgroundColor = [1 0.9882 0.9882];
            app.SpectrogramTab_3.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create sem
            app.sem = uiaxes(app.SpectrogramTab_3);
            title(app.sem, 'Spectrogram error microphone after ANC')
            xlabel(app.sem, 'Time frames')
            ylabel(app.sem, 'Frequency bins')
            app.sem.BackgroundColor = [1 0.9882 0.9882];
            app.sem.Position = [25 15 398 295];

            % Create AmplitudespectrumTab_3
            app.AmplitudespectrumTab_3 = uitab(app.TabGroup_3);
            app.AmplitudespectrumTab_3.Title = 'Amplitude spectrum';
            app.AmplitudespectrumTab_3.BackgroundColor = [1 0.9882 0.9882];
            app.AmplitudespectrumTab_3.ForegroundColor = [0.149 0.149 0.149];

            % Create aem
            app.aem = uiaxes(app.AmplitudespectrumTab_3);
            title(app.aem, 'Amplitue spectrum of error after ANC')
            xlabel(app.aem, 'frequency (Hz)')
            ylabel(app.aem, '|P(f)|')
            app.aem.BackgroundColor = [1 0.9882 0.9882];
            app.aem.Position = [25 15 398 295];

            % Create FrequencyresponsTab_3
            app.FrequencyresponsTab_3 = uitab(app.TabGroup_3);
            app.FrequencyresponsTab_3.Title = 'Frequency respons';
            app.FrequencyresponsTab_3.BackgroundColor = [1 0.9882 0.9882];

            % Create fem
            app.fem = uiaxes(app.FrequencyresponsTab_3);
            title(app.fem, 'Frequency response error after ANC')
            xlabel(app.fem, 'Frequency (Hz)')
            ylabel(app.fem, 'Power (dB)')
            app.fem.YLim = [-0.2 0.2];
            app.fem.BackgroundColor = [1 0.9882 0.9882];
            app.fem.Position = [25 15 398 295];

            % Create NoisecomingfromloudspeakerPanel_2
            app.NoisecomingfromloudspeakerPanel_2 = uipanel(app.UIFigure);
            app.NoisecomingfromloudspeakerPanel_2.TitlePosition = 'centertop';
            app.NoisecomingfromloudspeakerPanel_2.Title = 'Noise coming from loudspeaker';
            app.NoisecomingfromloudspeakerPanel_2.BackgroundColor = [1 1 1];
            app.NoisecomingfromloudspeakerPanel_2.FontWeight = 'bold';
            app.NoisecomingfromloudspeakerPanel_2.FontSize = 18;
            app.NoisecomingfromloudspeakerPanel_2.Position = [904 420 458 396];

            % Create TabGroup_4
            app.TabGroup_4 = uitabgroup(app.NoisecomingfromloudspeakerPanel_2);
            app.TabGroup_4.Position = [8 13 443 347];

            % Create PlotTab_4
            app.PlotTab_4 = uitab(app.TabGroup_4);
            app.PlotTab_4.Title = 'Plot';
            app.PlotTab_4.BackgroundColor = [1 0.9882 0.9882];
            app.PlotTab_4.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create l
            app.l = uiaxes(app.PlotTab_4);
            title(app.l, 'Ego-noise of loudspeaker')
            xlabel(app.l, 'Samples in time')
            ylabel(app.l, 'Amplitude')
            app.l.BackgroundColor = [1 0.9882 0.9882];
            app.l.Position = [25 15 398 295];

            % Create SpectrogramTab_4
            app.SpectrogramTab_4 = uitab(app.TabGroup_4);
            app.SpectrogramTab_4.Title = 'Spectrogram';
            app.SpectrogramTab_4.BackgroundColor = [1 0.9882 0.9882];
            app.SpectrogramTab_4.ForegroundColor = [0.0118 0.0118 0.3294];

            % Create sl
            app.sl = uiaxes(app.SpectrogramTab_4);
            title(app.sl, 'Spectrogram loudspeaker')
            xlabel(app.sl, 'Time frames')
            ylabel(app.sl, 'Frequency bins')
            app.sl.BackgroundColor = [1 0.9882 0.9882];
            app.sl.Position = [25 15 398 295];

            % Create AmplitudespectrumTab_4
            app.AmplitudespectrumTab_4 = uitab(app.TabGroup_4);
            app.AmplitudespectrumTab_4.Title = 'Amplitude spectrum';
            app.AmplitudespectrumTab_4.BackgroundColor = [1 0.9882 0.9882];
            app.AmplitudespectrumTab_4.ForegroundColor = [0.149 0.149 0.149];

            % Create al
            app.al = uiaxes(app.AmplitudespectrumTab_4);
            title(app.al, 'Amplitude spectrum of loudspeaker')
            xlabel(app.al, 'frequency (Hz)')
            ylabel(app.al, '|P(f)|')
            app.al.BackgroundColor = [1 0.9882 0.9882];
            app.al.Position = [25 15 398 295];

            % Create FrequencyresponsTab_4
            app.FrequencyresponsTab_4 = uitab(app.TabGroup_4);
            app.FrequencyresponsTab_4.Title = 'Frequency respons';
            app.FrequencyresponsTab_4.BackgroundColor = [1 0.9882 0.9882];

            % Create fl
            app.fl = uiaxes(app.FrequencyresponsTab_4);
            title(app.fl, 'Frequency response of loudspeaker')
            xlabel(app.fl, 'frequency (Hz)')
            ylabel(app.fl, 'Power (dB)')
            app.fl.YLim = [-0.2 0.2];
            app.fl.BackgroundColor = [1 0.9882 0.9882];
            app.fl.Position = [25 15 398 295];
        end
    end

    methods (Access = public)

        % Construct app
        function app = Application_ANC_final

            % Create and configure components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end