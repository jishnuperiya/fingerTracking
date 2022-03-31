function FingerTipAcc = getFingerTipAcc(glove_data)

FingerTipAcc.IndexFinger = glove_data.IndexFinger.IMU2(:,ImuSignals.AccZ);

FingerTipAcc.MiddleFinger = glove_data.MiddleFinger.IMU2(:,ImuSignals.AccZ);

FingerTipAcc.RingFinger = glove_data.RingFinger.IMU2(:,ImuSignals.AccZ);

FingerTipAcc.LittleFinger = glove_data.LittleFinger.IMU2(:,ImuSignals.AccZ);


end