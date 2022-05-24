%%read data and initialilze
clear all; clc;
glove_data = readRawData();

bias_gyrX = -0.2356;
bias_gyrY = -0.7323;
bias_gyrZ = 0.4945;
samplePeriod = 1 /50;
gravity = 9.80961; 

time = glove_data.Timestamp; 
gyrX = glove_data.Hand.IMU(:,ImuSignals.GyroX)-bias_gyrX;
gyrY = glove_data.Hand.IMU(:,ImuSignals.GyroY)-bias_gyrY;
gyrZ = glove_data.Hand.IMU(:,ImuSignals.GyroZ)-bias_gyrZ;
accX = glove_data.Hand.IMU(:,ImuSignals.AccX)/gravity;
accY = glove_data.Hand.IMU(:,ImuSignals.AccY)/gravity;
accZ = glove_data.Hand.IMU(:,ImuSignals.AccZ)/gravity; 

linVel_prev=[0 0 0];
linPos_prev=0;
acc_prev=[accX(1) accY(1) accZ(1)];
gyr_prev=[gyrX(1) gyrY(1) gyrZ(1)];
ahrs = MahonyAHRS('SamplePeriod', samplePeriod, 'Kp', 1);
%% loop
for i = 1:size(time,1)
    max_frames=size(time,1);
      
    acc = [accX(i) accY(i) accZ(i)];
    gyr = [gyrX(i) gyrY(i) gyrZ(i)];
    
    [linVel_prev,linPos_prev,gyr_prev,acc_prev,RotationMatrixArray3D]=calcHandPositionAndOrientation(acc,gyr,linVel_prev,linPos_prev,gyr_prev,acc_prev,i,ahrs,max_frames);
   
end   
