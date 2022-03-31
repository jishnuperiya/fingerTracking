function glove_data = readRawData()

%read csv
[file, path] = uigetfile('*.csv',...
    'Select One Measurement File', ...
    'MultiSelect', 'off');
DataFile = fullfile(path, file);

opts = detectImportOptions(DataFile);
Data = readtable(DataFile, opts);

% save it into struct
glove_data.Timestamp =table2array(Data(:,1));
glove_data.Hand.IMU= table2array(Data(:,2:7));

glove_data.IndexFinger.MR0= table2array(Data(:,8));
glove_data.IndexFinger.IMU0= table2array(Data(:,9:14));
glove_data.IndexFinger.MR1= table2array(Data(:,15));
glove_data.IndexFinger.IMU1= table2array(Data(:,16:21));
glove_data.IndexFinger.MR2= table2array(Data(:,22));
glove_data.IndexFinger.IMU2= table2array(Data(:,23:28));

glove_data.MiddleFinger.MR0= table2array(Data(:,29));
glove_data.MiddleFinger.IMU0= table2array(Data(:,30:35));
glove_data.MiddleFinger.MR1= table2array(Data(:,36));
glove_data.MiddleFinger.IMU1= table2array(Data(:,37:42));
glove_data.MiddleFinger.MR2= table2array(Data(:,43));
glove_data.MiddleFinger.IMU2= table2array(Data(:,44:49));

end

