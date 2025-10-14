%--------------------------------------------------------------------------
% Liquid parameters Estimation
%
%    Author:     Simone Soprani
%    Email:      simone.soprani2@unibo.it
%    Date:       February 2025
%--------------------------------------------------------------------------
clear all
close all
clc

%% import 
ws_path = fullfile('..', '..');
% odes_path = fullfile(ws_path,"Sloshing_model", "odes");
% add folder and subfolders
addpath("odes");

%% import bag

bag_folder = fullfile(pwd,"bagfiles");
date = "290425";
file = "y_noRot.bag";

bag_path = fullfile(bag_folder, date, file);
bag = rosbag(bag_path);

fs = 500; % Sampling frequency (Hz)
dt = 1/fs; % Time step in seconds

motion_end = 2.5;
% motion_end = 8.0;

%% Import force readings
topic = select(bag, 'Topic', '/atift_sensor/data');
msgs = readMessages(topic);

% store all forces
fx = cellfun(@(m) m.Linear.X, msgs);
fy = cellfun(@(m) m.Linear.Y, msgs);
fz = cellfun(@(m) m.Linear.Z, msgs);
% store all moments
mx = cellfun(@(m) m.Angular.X, msgs);
my = cellfun(@(m) m.Angular.Y, msgs);
mz = cellfun(@(m) m.Angular.Z, msgs);


% Create time vector based on sampling rate
t = 0:dt:(length(fx)-1)*dt;

% Number of data points
N = length(fx);

%% Filter signal
fc = 8;   % Cutoff frequency (Hz)
order = 4; % Filter order

% Design a low-pass Butterworth filter
[b, a] = butter(order, fc / (fs / 2), 'low');
% [b, a] = butter(order, [2.4/ (fs / 2), fc / (fs / 2)], 'bandpass');

% Apply zero-phase filtering using filtfilt
fx_filt = filtfilt(b, a, fx);
fy_filt = filtfilt(b, a, fy);
fz_filt = filtfilt(b, a, fz);

mx_filt = filtfilt(b, a, mx);
my_filt = filtfilt(b, a, my);
mz_filt = filtfilt(b, a, mz);

% fy = fy_filt;

figure;
hold on;
plot(t, fy, 'r', 'DisplayName', 'Original fy');
plot(t, fy_filt, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered fy');
xlabel('Time (s)');
ylabel('Force (N)');
title('Low-Pass Filtering of fy');
legend;
grid on;


%% Damping ratio (logarithmic decrement)
% % Isolate settling phase
fy_filt_set = fy_filt(motion_end/dt:end); 
t_set = 0:dt:(length(fy_filt_set)-1)*dt;
% Find Peaks
[peaks1, locs1] = findpeaks(fy_filt_set, 'MinPeakHeight', max(fy_filt_set)/10);
% 
% % figure;
% % hold on;
% % plot(t_set, fy_filt_set, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered fy');
% % xlabel('Time (s)');
% % ylabel('Force (N)');
% % title('Low-Pass Filtering of fy after motion end');
% % legend;
% % grid on;
% 
% % figure;
% % hold on;
% % plot(t_set, log(fy_filt_set), 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered fy');
% % xlabel('Time (s)');
% % ylabel('Force (N)');
% % title('Low-Pass Filtering of fy after motion end');
% % legend;
% % grid on;
% 
% 
% % Choose Peaks Separated by N Cycles
% % osc = 26; % Choose a separation of 5 oscillations
% osc = length(peaks1)-1;
% if length(peaks1) > osc
%     A_i = peaks1(1); % First identified peak
%     % A_j = peaks1(osc+1); % Peak after N cycles
%     A_j = peaks1(osc); % Peak after N cycles
% 
% 
%     % Compute Logarithmic Decrement
%     delta = (1/osc) * log(A_i / A_j);
% 
%     % Compute Damping Ratio
%     zeta_dr = delta / sqrt(4 * pi^2 + delta^2);
% 
%     % Display Results
%     % fprintf('Estimated damping ratio (logarithmic decrement): %f\n', zeta_dr);
% else
%     warning('Not enough peaks detected for logarithmic decrement calculation.');
% 
% end
% 
% % figure;
% % hold on;
% % plot(t_set, fy_filt_set, 'b', 'LineWidth', 1.5, 'DisplayName', 'Filtered fy');
% % plot(t_set, exp(-4*0.028571*t_set - 0.8))
% % xlabel('Time (s)');
% % ylabel('Force (N)');
% % title('Low-Pass Filtering of fy after motion end');
% % legend;
% % grid on;

%% Damping ratio (peak interpolation)
log_fy = log(abs(fy_filt_set)); % Ensure non-negative values

% Find Every Other Peak
[peaks, locs] = findpeaks(log_fy); % Find log peaks
locs = locs(2:2:end);  % Take every other peak
peaks = peaks(2:2:end);

% Convert indices to time values
t_locs = t_set(locs); 

% Linear Fit to Extract Damping Coefficient
p = polyfit(t_locs, peaks, 1); % Linear interpolation in time domain
damping_coeff = -p(1); % Slope of the line

% Check by Plotting the Fit on Log(fy_filt)
log_fit = polyval(p, t_locs); % Get fitted values

figure;
hold on;
plot(t_locs, peaks, 'ro', 'MarkerSize', 6, 'DisplayName', 'Selected Peaks');
plot(t_locs, log_fit, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Linear Fit');
plot(t_set, log_fy, 'LineWidth', 1.5, 'DisplayName', 'log(Fy)')
xlabel('Time (s)');
ylabel('log(fy\_filt)');
title('Log of Filtered Force with Linear Fit');
legend;
grid on;

% Plot Exponential Decay on Original Signal
exp_fit = exp(polyval(p, t_set)); % Convert back to original scale

figure;
hold on;
plot(t_set, fy_filt_set, 'k-', 'DisplayName', 'Original Signal');
plot(t_set, exp_fit, 'r--', 'LineWidth', 1.5, 'DisplayName', 'Exponential Fit');
plot(t_set, exp(p(1)*t_set +p(2))-0.034, 'g--', 'LineWidth', 1.5, 'DisplayName', 'Exponential Fit');
xlabel('Time (s)');
ylabel('Force (N)');
title('Original Signal with Exponential Decay Fit');
legend;
grid on;

% Display Result
fprintf('Estimated damping coefficient: %f\n', damping_coeff);


%% Perform FFT on the force signal fx
% FFT transforms the signal from the time domain (force vs. time) 
% into the frequency domain (amplitude vs. frequency), 
% revealing dominant frequencies in the signal.
% Returns complex-valued vector containing both magnitude and phase
% information for each frequency component.
% length of F_fft is the same as fx (N points)
F_fft = fft(fy); 


% FFT output is symmetric because the input signal is real-valued.
% The second half of F_fft contains redundant negative frequencies (a mirror image of the first half).
% Retain only the positive half of the frequency spectrum (since FFT is symmetric)
% FFT output consists of N frequency components.
% first N/2+1 values correspond to frequencies from 0 Hz to the Nyquist frequency (half the sampling rate).
F_fft = F_fft(1:N/2+1);

%% Generate the frequency axis
% 0:N/2 → Generates an index vector from 0 to N/2
% fs / N → Frequency resolution (smallest step in the frequency axis)
% Multiplying them gives actual frequency values in Hz
% If fs = 500 Hz and N = 1000, the frequency axis ranges from 0 Hz to 250 Hz in steps of 0.5 Hz.
freq = (0:N/2) * fs / N;

%% Compute the magnitude spectrum
% abs value (magnitude) of the complex FFT output, normalized to make it independent of the signal length.
% Without normalization, the FFT output depends on N, which distorts amplitude comparisons
F_magnitude = abs(F_fft) / N;

%% scale magnitude spectrum
% The FFT outputs the full amplitude across both positive and negative frequencies.
% Since we kept only the positive half, we need to double the magnitude (except DC and Nyquist).
% Skips:
% 1st value (F_magnitude(1)) → The DC component (0 Hz), which is not duplicated.
% Last value (F_magnitude(end)) → The Nyquist frequency, which is also not duplicated.
F_magnitude(2:end-1) = 2 * F_magnitude(2:end-1);


%% Plot values in time
f = figure();
set(f,'renderer','Painters');
hold on
plot(t-2.5,fx, 'LineWidth',2.5);
plot(t-2.5,fy,'LineWidth',2.5);
plot(t-2.5,fz,'LineWidth',2.5);
xlabel('Time (s)','FontSize', 20, 'FontWeight','bold', 'Interpreter', 'latex');
ylabel('Force (N)','FontSize', 20, 'FontWeight','bold', 'Interpreter', 'latex');
title('Force Signal','FontSize', 20, 'FontWeight','bold', 'Interpreter', 'latex');
legend('$F_x$', '$F_y$', '$F_z$','Interpreter', 'Latex','FontSize',40)
xlim([0 4.5])
ylim([-6 6])
grid on;
set(gca, 'FontSize', 40, 'TickLabelInterpreter', 'latex');
set(gcf, 'Units', 'Normalized', 'OuterPosition', [0 0 1 1]);
pos = get(f,'Position');
set(f,'Units','Normalized');
set(f,'PaperOrientation','landscape','PaperPositionMode','manual','PaperUnits','centimeters','PaperSize',[40, 20])

save_name = strcat('y','.pdf');
print('-dpdf', '-fillpage', save_name)







figure;
plot(freq, F_magnitude);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('FFT of Force Signal Fy');
grid on;

%% Find dominant frequencies
[peaks, locs] = findpeaks(F_magnitude, 'MinPeakHeight', max(F_magnitude)/5);
dominant_freqs = freq(locs);
disp('Dominant frequencies (Hz):');
disp(dominant_freqs);

%% After motion is finished
ix = motion_end/dt;
fy_2 = fy(ix:end); 

t_set = 0:dt:(length(fy_2)-1)*dt;
N = length(fy_2); % Number of points
F_fft = fft(fy_2); % Compute FFT
F_fft = F_fft(floor(1:N/2+1)); % Keep positive frequencies
freq = (0:N/2) * fs / N; % Frequency axis

% Compute the magnitude spectrum
F_magnitude = abs(F_fft) / N;
F_magnitude(2:end-1) = 2 * F_magnitude(2:end-1); % Correct scaling

figure;
hold on
plot(t_set,fy_2);
% plot(t,fy);
xlabel('Time (s)');
ylabel('Force (N)');
title('Force Signal after motion end');
grid on;

figure;
plot(freq, F_magnitude);
xlabel('Frequency (Hz)');
ylabel('Magnitude');
title('FFT of Force Signal after motion end');
grid on;

[peaks, locs] = findpeaks(F_magnitude, 'MinPeakHeight', max(F_magnitude)/15);
dominant_freqs = freq(locs);
disp('Dominant frequencies (Hz):');
disp(dominant_freqs);

%% Compare estimated wn with theoretical one
R = 0.049;
h = 0.08;
% [~, ~, ~, ~, ~, zeta, ~, ~, ~, ~, ~, ~, ~, ~, wn] = ParametersP(R, h);
[g, rho, m_tot, V, csi11, zeta, ms, ks, cs, as, ls, Ls, J, k, wn] = ParametersP(R, h);


% w = 2pi/T = 2pi*f
wn_est = 2*pi*dominant_freqs(1);

fprintf('Estimated w: %f.\n', wn_est);
fprintf('Theoretical w: %f.\n', wn);

% zeta_dr =  0.5*zeta_dr/wn_est;
zeta_est = damping_coeff/wn_est;

% fprintf('Estimated damping ratio (logarithmic decrement): %f\n', zeta_dr);
fprintf('Estimated damping ratio  (interpolated peaks): %f\n', zeta_est);

%% Compare estimated zita with theoretical one
% Compute estimated damping ratio using the Half-Power Bandwidth Method

% Find the peak frequency
[~, peak_idx] = max(F_magnitude);
fn = freq(peak_idx); % Dominant frequency (natural frequency)

% Half-power magnitude (3dB drop)
half_power_mag = max(F_magnitude) / sqrt(2);

% Find the closest lower and higher frequencies where magnitude ≈ half_power_mag
low_idx = find(F_magnitude(1:peak_idx) <= half_power_mag, 1, 'last'); % Left side of peak
high_idx = find(F_magnitude(peak_idx:end) <= half_power_mag, 1, 'first') + peak_idx - 1; % Right side of peak

% Ensure valid indices were found
if isempty(low_idx) || isempty(high_idx)
    error('Could not find valid -3dB points. Check FFT resolution.');
end

% Extract the -3dB frequencies
f_low = freq(low_idx);
f_high = freq(high_idx);

% Compute damping ratio
zeta_hp = (f_high - f_low) / (2 * fn);

zeta_hp = 0.5*zeta_hp/wn_est;

% Display results
fprintf('Estimated damping ratio (half-power method): %f\n', zeta_hp);
fprintf('Theoretical damping ratio: %f\n', zeta);

%% Model prediction
kin_path = fullfile(ws_path,"Kinematics");
addpath(genpath(kin_path));
FIR_path = fullfile(ws_path,"Control","Algebraic_Control","Slosh","Single_container","utils");
addpath(genpath(FIR_path));

path_type   = 'y';
dim_type    = '2D';
Ts = 1;
robot       = create_SmartSix();

[rEx, rEdx, rEddx, ~, ~, rEy, rEdy, rEddy, ~, ~, rEz, rEdz, rEddz, ~, ~, ~, ~, ~, ~, Te, time, n] = FIR_traj(path_type, dim_type, 0, robot, Ts);

time = 0:dt:(length(fy_filt)/2)*dt;  % This is the expanded time vector based on fy's length
rEddx = [rEddx, zeros(length(time) - length(rEddx), 1)'];
rEddy = [rEddy, zeros(length(time) - length(rEddy), 1)'];
rEddz = [rEddz, zeros(length(time) - length(rEddz), 1)'];
Te = time(end);
 

tspan = [0 2*Te];
% S0 = [0 0 0 0 0 0 0];
S0 = [0 0 0 0];
thdd_zero = zeros(1,length(rEddx));

%%NL 
% [tPNL_L,sPNL_L] = ode45(@(t,s)odeSchoenP(t,s,ls,k,(2*0.028571)/wn_est,ms,time,rEddx,rEddy,rEddz,thdd_zero,J,g,as,wn_est,'NL'), tspan, S0);
[tPNL_L,sPNL_L] = ode45(@(t,s)odeP(t,s,ls,k,zeta_est,ms,time,rEddx,rEddy,rEddz,J,g,as,wn_est,'NL'), tspan, S0);


etaPNL_L = zeros(length(tPNL_L),1);
gammaPNL_L = (h*ms)/(rho*V) * 1/tanh(csi11*h/R);
for i=1:length(tPNL_L)
    etaPNL_L(i) = gammaPNL_L*2*((csi11^2 - 1)*(1-cos(sPNL_L(i,1))*cos(sPNL_L(i,2))))^0.5;
end

%% Plot
Delta_start_index = 34;
% fs = 60; % Framerate experiment video
end_time = Te;
% [~, fmax_index] = max(fy_filt);
[~, fmax_index] = max(fy);

[~, Smax_index] = max(etaPNL_L);
Speak_time = tPNL_L(Smax_index);

start_index = round(fmax_index - Speak_time*fs) + Delta_start_index;
fy_cut = fy_filt(start_index+1:start_index+(end_time*fs));
% fy_cut = fy(start_index+1:start_index+(end_time*fs));
t_cut = t(start_index+1:start_index+(end_time*fs)) - t(start_index+1);



figure
hold on
grid on
plot(tPNL_L, etaPNL_L*100,'LineWidth',1.5);
plot(t_cut, rEddy(1:end_time*fs),'LineWidth',1.5);
plot(t_cut,2.5*abs(fy_cut),'LineWidth',1.5)
% line([Te Te],[0 Ylim],'Color','k','LineStyle','--','LineWidth',1,'HandleVisibility','off')
xlabel('t [s]', 'Interpreter', 'latex');
ylabel('$\overline {\eta}$ [mm]','Interpreter', 'latex');
xlim([0 end_time])
lg2 = legend('$\mathrm{PNL_L}$ Model','Fy','Location', 'north', 'interpreter', 'latex');
lg2.NumColumns = 2; % Adjust the number of columns as needed
currentPosition = lg2.Position; % [x, y, width, height]
lg2.Position(2) = currentPosition(2) + 0.09;
set(gca, 'TickLabelInterpreter', 'latex');
