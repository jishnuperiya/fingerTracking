%% Housekeeping
 
addpath('ximu_matlab_library');	% include x-IMU MATLAB library
addpath('quaternion_library');	% include quatenrion library
close all;                     	% close all figures
clear;                         	% clear all variables
clc;                          	% clear the command terminal

%% parameters (CB)
cmpBias = 1;  %switch on/off bias compensation
gravity = 9.80961; % g in Gaimersheim
maxGyr4Standstill = 1.2; %[deg/s]
maxDGyr4Standstill = 30; %[deg/s²]
maxDAcc4Standstill = 1.25; %[m/s³]
time4init = 1; %[s]
 
%% Import data
%1) save the glove raw data into a struct
glove_data = readRawData();

%2) define a hand modell
HandModell = defineHandModell();
% xIMUdata = xIMUdataClass('LoggedData/LoggedData');

% samplePeriod = 1/256;
samplePeriod = 1 /50;
time = glove_data.Timestamp; 
gyrX = glove_data.Hand.IMU(:,ImuSignals.GyroX);
gyrY = glove_data.Hand.IMU(:,ImuSignals.GyroY);
gyrZ = glove_data.Hand.IMU(:,ImuSignals.GyroZ);
accX = glove_data.Hand.IMU(:,ImuSignals.AccX)/gravity;
accY = glove_data.Hand.IMU(:,ImuSignals.AccY)/gravity;
accZ = glove_data.Hand.IMU(:,ImuSignals.AccZ)/gravity;

% Bias handling (CB)
if(cmpBias)
    bias_gyrX = -0.2356;
    bias_gyrY = -0.7323;
    bias_gyrZ = 0.4945;
    gyrX = gyrX - bias_gyrX;
    gyrY = gyrY - bias_gyrY;
    gyrZ = gyrZ - bias_gyrZ;
end

acc = [accX accY accZ];
gyr = [gyrX gyrY gyrZ];
  
% Plot
figure('NumberTitle', 'off', 'Name', 'Gyroscope');
hold on;
plot(gyr(:,1), 'r');
plot(gyr(:,2), 'g');
plot(gyr(:,3), 'b');
xlabel('sample');
ylabel('dps');
title('Gyroscope');
legend('X', 'Y', 'Z');

figure('NumberTitle', 'off', 'Name', 'Accelerometer');
hold on;
plot(acc(:,1), 'r');
plot(acc(:,2), 'g');
plot(acc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Accelerometer');
legend('X', 'Y', 'Z');

%% initialization (CB)
standstill = zeros(length(gyr),1);
for i=2:length(gyr)
    if(abs(gyrX(i)) < maxGyr4Standstill && abs(gyrY(i)) < maxGyr4Standstill && abs(gyrZ(i)) < maxGyr4Standstill...
    && abs(gyrX(i) - gyrX(i-1))/samplePeriod < maxDGyr4Standstill && abs(gyrY(i) - gyrY(i-1))/samplePeriod < maxDGyr4Standstill && abs(gyrZ(i) - gyrZ(i-1))/samplePeriod < maxDGyr4Standstill ...
    && abs(accX(i) - accX(i-1))/samplePeriod < maxDAcc4Standstill && abs(accY(i) - accY(i-1))/samplePeriod < maxDAcc4Standstill && abs(accZ(i) - accZ(i-1))/samplePeriod < maxDAcc4Standstill)
        standstill(i) = 1;
    end
end

ntime4init = floor(time4init/samplePeriod);
standstillperiod = 0;
gyrMean = [0 0 0];
accMean = [0 0 0];
stationaryAlignmentIndex = 0;
for i=2:length(gyr)
    if(standstill(i))
        standstillperiod = standstillperiod + 1;
        gyrMean = gyrMean + gyr(i,:);
        accMean = accMean + acc(i,:);
        if(standstillperiod >= ntime4init)
            gyrMean = gyrMean / ntime4init;
            accMean = accMean / ntime4init;
            stationaryAlignmentIndex = i;
            break
        end
    else
        standstillperiod = 0;
        gyrMean = [0 0 0];
        accMean = [0 0 0];
    end
end
% if(accMean(3) > 1 && cmpBias)
%     acc(:,3) = acc(:,3) - (accMean(3) - 1);
%     accMean(3) = 1;
% end
phi0 = atan2(accMean(2),accMean(3));
theta0 = asin(-accMean(1)/norm(accMean));

%% Process data through AHRS algorithm (calculate orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/

R = zeros(3,3,length(gyr));     % rotation matrix describing sensor relative to Earth

ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
% % initialize with stationary alignment (CB)
% if(cmpBias)
%     ahrs.Quaternion = Eul2Qnb(phi0, theta0, 0);
% end

for i = 1:length(gyr)
    ahrs.UpdateIMU(gyr(i,:) * (pi/180), acc(i,:));	% gyroscope units must be radians
    R(:,:,i) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
end
%% Calculate 'tilt-compensated' accelerometer

tcAcc = zeros(size(acc));  % accelerometer in Earth frame

for i = 1:length(acc)
    tcAcc(i,:) = R(:,:,i) * acc(i,:)';
end

% Plot
figure('NumberTitle', 'off', 'Name', '''Tilt-Compensated'' accelerometer');
hold on;
plot(tcAcc(:,1), 'r');
plot(tcAcc(:,2), 'g');
plot(tcAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('''Tilt-compensated'' accelerometer');
legend('X', 'Y', 'Z');

%% Calculate angles (CB)
phi = zeros(length(gyr),1);
theta = zeros(length(gyr),1);
psi = zeros(length(gyr),1);
phi(:,1) = atan2(R(3,2,:),R(3,3,:));
theta(:,1) = atan2(-R(3,1,:),sqrt(R(3,2,:).*R(3,2,:) + R(3,3,:).*R(3,3,:)));
psi(:,1) = atan2(R(2,1,:),R(1,1,:));

% Plot
figure('NumberTitle', 'off', 'Name', 'Orientation Angles');
hold on;
plot(phi, 'r');
plot(theta, 'g');
plot(psi, 'b');
xlabel('sample');
ylabel('rad');
title('Orientation Angles');
legend('X', 'Y', 'Z');

%% Calculate linear acceleration in Earth frame (subtracting gravity)

linAcc = tcAcc - [zeros(length(tcAcc), 1), zeros(length(tcAcc), 1), ones(length(tcAcc), 1)]; % substracting gravity. since the accelarations are normalized with g, subtracting 1 is similar to subtracting gravity
linAcc = linAcc * gravity;     % convert from 'g' to m/s/s<

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Acceleration');
hold on;
plot(linAcc(:,1), 'r');
plot(linAcc(:,2), 'g');
plot(linAcc(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear acceleration');
legend('X', 'Y', 'Z');

%% Calculate linear velocity (integrate acceleartion)

linVel = zeros(size(linAcc));

for i = 2:length(linAcc)
    linVel(i,:) = linVel(i-1,:) + linAcc(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Velocity');
hold on;
plot(linVel(:,1), 'r');
plot(linVel(:,2), 'g');
plot(linVel(:,3), 'b');
xlabel('sample');
ylabel('m/s');
title('Linear velocity');
legend('X', 'Y', 'Z');

% High-pass filter linear velocity to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelHP = filtfilt(b, a, linVel);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelHP(:,1), 'r');
plot(linVelHP(:,2), 'g');
plot(linVelHP(:,3), 'b');
xlabel('sample');
ylabel('m/s');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');

%% Calculate linear position (integrate velocity)
% 
 linPos = zeros(size(linVelHP));


for i = 2:length(linVelHP)
    linPos(i,:) = linPos(i-1,:) + linVelHP(i,:) * samplePeriod;
end

% Plot
figure('NumberTitle', 'off', 'Name', 'Linear Position');
hold on;
plot(linPos(:,1), 'r');
plot(linPos(:,2), 'g');
plot(linPos(:,3), 'b');
xlabel('sample');
ylabel('g');
title('Linear position');
legend('X', 'Y', 'Z');

%% High-pass filter linear position to remove drift

order = 1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linPosHP = filtfilt(b, a, linPos);

% Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
hold on;
plot(linPosHP(:,1), 'r');
plot(linPosHP(:,2), 'g');
plot(linPosHP(:,3), 'b');
xlabel('sample');
ylabel('g');
title('High-pass filtered linear position');
legend('X', 'Y', 'Z');

%% Play animation

SamplePlotFreq = 2;
% linPosHP = zeros(size(linPosHP))
SixDOFanimation(linPosHP, R, ...
                'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                'Position', [9 39 1280 720], ...
                'AxisLength', 0.1, 'ShowArrowHead', false, ...
                'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
 
%% End of script