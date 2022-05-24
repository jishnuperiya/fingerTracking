function [linVel,linPos,gyr,acc,RotationMatrixArray3D] = calcHandPositionAndOrientation(acc,gyr,linVel_prev,linPos_prev,gyr_prev,acc_prev,current_frame,ahrs,max_frames)
                                                      
%% parameters (CB)
cmpBias = 1;  %switch on/off bias compensation
gravity = 9.80961; % g in Gaimersheim
maxGyr4Standstill = 1.2; %[deg/s]
maxDGyr4Standstill = 30; %[deg/s²]
maxDAcc4Standstill = 1.25; %[m/s³]
time4init = 1; %[s]
samplePeriod = 1 /50;

% current imu signals
gyrX = gyr(1);
gyrY = gyr(2);
gyrZ = gyr(3);
accX = acc(1);
accY = acc(2);
accZ = acc(3);

% previous imu signals
gyrX_prev = gyr_prev(1);
gyrY_prev = gyr_prev(2);
gyrZ_prev = gyr_prev(3);
accX_prev = acc_prev(1);
accY_prev = acc_prev(2);
accZ_prev = acc_prev(3);


%2) define a hand modell
HandModell = defineHandModell();
xIMUdata = xIMUdataClass('LoggedData/LoggedData');

%% initialization (CB)

standstill = 0;

if(abs(gyrX) < maxGyr4Standstill && abs(gyrY) < maxGyr4Standstill && abs(gyrZ) < maxGyr4Standstill...
&& abs(gyrX - gyrX_prev)/samplePeriod < maxDGyr4Standstill && abs(gyrY - gyrY_prev)/samplePeriod < maxDGyr4Standstill && abs(gyrZ - gyrZ_prev)/samplePeriod < maxDGyr4Standstill ...
&& abs(accX - accX_prev)/samplePeriod < maxDAcc4Standstill && abs(accY - accY_prev)/samplePeriod < maxDAcc4Standstill && abs(accZ - accZ_prev)/samplePeriod < maxDAcc4Standstill)
    standstill = 1;
end


ntime4init = floor(time4init/samplePeriod);
standstillperiod = 0;

persistent gyrMean;
if isempty(gyrMean)
    gyrMean = [0 0 0];
end
persistent accMean;
if isempty(accMean)
    accMean = [0 0 0];
end

stationaryAlignmentIndex = 0;

if standstill
    standstillperiod = standstillperiod + 1;
    gyrMean = gyrMean + gyr;
    accMean = accMean + acc;
    if(standstillperiod >= ntime4init)
        gyrMean = gyrMean / ntime4init;
        accMean = accMean / ntime4init;
        stationaryAlignmentIndex = current_frame;        
    end
else
    standstillperiod = 0;
    gyrMean = [0 0 0];
    accMean = [0 0 0];
end



% disp("gyro info:");
% disp(gyr);
% disp(gyrMean);
% disp("accel info:");
% disp(acc);
% disp(accMean);

% if stationaryAlignmentIndex >0
%     disp("stationaryAlignmentIndex:",stationaryAlignmentIndex);
% end


%% Process data through AHRS algorithm (calculate orientation)
% See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
persistent RotationMatrix_array;
if isempty(RotationMatrix_array)
RotationMatrix_array =[];
end
R = zeros(3,3);

% % initialize with stationary alignment (CB)
% if(cmpBias)
%     ahrs.Quaternion = Eul2Qnb(phi0, theta0, 0);
% end

ahrs.UpdateIMU(gyr * (pi/180), acc);	% gyroscope units must be radians
R(:,:) = quatern2rotMat(ahrs.Quaternion)';    % transpose because ahrs provides Earth relative to sensor
RotationMatrix_array = [RotationMatrix_array;R];



tcAcc = R * acc';
linAcc = tcAcc' - [ 0 0 1]; % substracting gravity. since the accelarations are normalized with g, subtracting 1 is similar to subtracting gravity
linAcc = linAcc * gravity;     % convert from 'g' to m/s/s<

persistent linAccArray;
if isempty(linAccArray)
linAccArray=[];
end
linAccArray=[linAccArray;linAcc];

%% calc veloctity
if current_frame ==1
linVel = [0 0 0];
else
linVel = linVel_prev + linAcc * samplePeriod;
end
persistent linVelArray;
if isempty(linVelArray)
linVelArray=[];
end
linVelArray=[linVelArray;linVel];
%% High-pass filter linear velocity to remove drift

if size(linVelArray,1)>4
order =1;
filtCutOff = 0.1;
[b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
linVelArrayHP = filtfilt(b, a, linVelArray);
end
%% calc pos
% %old

if size(linVelArray,1)<5
linPos = linPos_prev + linVel* samplePeriod; % linVelArray(end,:)--> current vel
else
linPos = linPos_prev + linVelArrayHP(end,:)* samplePeriod; % linVelArray(end,:)--> current vel
end 
% %new
% persistent linPosArray;
% if isempty(linPosArray)
%     linPosArray = zeros(770,3);
% end   
%     
% if current_frame>3
% linPosArray(current_frame,:) = linPosArray(current_frame-1,:) + linVelArrayHP(current_frame,:) * samplePeriod;
% else
% linPos = [0 0 0];    
% end 
% linPos = [ 0 0 0]; % delete later
% if size(linVelArray,1)<2
%  linPos = linPos_prev + linVelArray(end,:)* samplePeriod; % linVelArray(end,:)--> current vel
% else
%  linPos = linPos_prev + (linVelArray(end,:)*0.8 + linVelArray(end,:)*0.2)*samplePeriod;
% % end 
persistent linPosArray;
if isempty(linPosArray)    
linPosArray=[];
end
% if current_frame>3
linPosArray=[linPosArray;linPos];
% end
%% High-pass filter linear position to remove drift
% if size(linPosArray,1)>3
% order = 1;
% filtCutOff = 0.1;
% [b, a] = butter(order, (2*filtCutOff)/(1/samplePeriod), 'high');
% linPosArrayHP = filtfilt(b, a, linPosArray);
% end
% disp(linPosArray);


% NaN("vel array: ");
% disp(linVelArray);
% disp("pos array: ");
% disp(linPosArray);



%% changing the rotation matrix from 2d to 3d matrix for visualization
depth=size(RotationMatrix_array,1)/ size(RotationMatrix_array,2);
RotationMatrixArray3D=permute(reshape(RotationMatrix_array, 3, depth, 3), [1 3 2]);
disp(current_frame)
    %% play animation at the end of the measurement
    if current_frame ==max_frames
        % Plot
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linPosArray(:,1), 'r');
plot(linPosArray(:,2), 'g');
plot(linPosArray(:,3), 'b');
xlabel('sample');
ylabel('m');
title('linear pos');
legend('X', 'Y', 'Z');
% 
% % Plot
% figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Position');
% hold on;
% plot(linPosArrayHP(:,1), 'r');
% plot(linPosArrayHP(:,2), 'g');
% plot(linPosArrayHP(:,3), 'b');
% xlabel('sample');
% ylabel('g');
% title('High-pass filtered linear position');
% legend('X', 'Y', 'Z');
% 
figure('NumberTitle', 'off', 'Name', 'High-pass filtered Linear Velocity');
hold on;
plot(linVelArrayHP(:,1), 'r');
plot(linVelArrayHP(:,2), 'g');
plot(linVelArrayHP(:,3), 'b');
xlabel('sample');
ylabel('m/s');
title('High-pass filtered linear velocity');
legend('X', 'Y', 'Z');
     
    SamplePlotFreq = 2;
    % linPosHP = zeros(size(linPosHP))
    SixDOFanimation(linPosArrayHP,RotationMatrixArray3D, ...
                    'SamplePlotFreq', SamplePlotFreq, 'Trail', 'Off', ...
                    'Position', [9 39 1280 720], ...
                    'AxisLength', 0.1, 'ShowArrowHead', false, ...
                    'Xlabel', 'X (m)', 'Ylabel', 'Y (m)', 'Zlabel', 'Z (m)', 'ShowLegend', false, 'Title', 'Unfiltered',...
                    'CreateAVI', false, 'AVIfileNameEnum', false, 'AVIfps', ((1/samplePeriod) / SamplePlotFreq));            
% 
%     %% End of script
    end
end

