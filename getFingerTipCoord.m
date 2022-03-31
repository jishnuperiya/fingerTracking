
function FingerTipCoord = getFingerTipCoord(HandModell,glove_data)




% x = HandModell.IndexFinger.PP* cos(?_1) + HandModell.IndexFinger.MP* cos(?_12) + HandModell.IndexFinger.DP * cos(?_123) 
% y = HandModell.IndexFinger.PP* sin(?_1) + HandModell.IndexFinger.MP* sin(?_12) + HandModell.IndexFinger.DP * sin(?_123) 

FingerTipCoord.IndexFinger.x = HandModell.IndexFinger.PP* cos(deg2rad(glove_data.IndexFinger.MR0)) + HandModell.IndexFinger.MP* cos(deg2rad(glove_data.IndexFinger.MR1)) + HandModell.IndexFinger.DP * cos(deg2rad(glove_data.IndexFinger.MR2)); 
FingerTipCoord.IndexFinger.y = HandModell.IndexFinger.PP* sin(deg2rad(glove_data.IndexFinger.MR0)) + HandModell.IndexFinger.MP* sin(deg2rad(glove_data.IndexFinger.MR1)) + HandModell.IndexFinger.DP * sin(deg2rad(glove_data.IndexFinger.MR2)); 

FingerTipCoord.MiddleFinger.x = HandModell.MiddleFinger.PP* cos(deg2rad(glove_data.MiddleFinger.MR0)) + HandModell.MiddleFinger.MP* cos(deg2rad(glove_data.MiddleFinger.MR1)) + HandModell.MiddleFinger.DP * cos(deg2rad(glove_data.MiddleFinger.MR2)); 
FingerTipCoord.MiddleFinger.y = HandModell.MiddleFinger.PP* sin(deg2rad(glove_data.MiddleFinger.MR0)) + HandModell.MiddleFinger.MP* sin(deg2rad(glove_data.MiddleFinger.MR1)) + HandModell.MiddleFinger.DP * sin(deg2rad(glove_data.MiddleFinger.MR2)); 

FingerTipCoord.RingFinger.x = HandModell.RingFinger.PP* cos(deg2rad(glove_data.RingFinger.MR0)) + HandModell.RingFinger.MP* cos(deg2rad(glove_data.RingFinger.MR1)) + HandModell.RingFinger.DP * cos(deg2rad(glove_data.RingFinger.MR2)); 
FingerTipCoord.RingFinger.y = HandModell.RingFinger.PP* sin(deg2rad(glove_data.RingFinger.MR0)) + HandModell.RingFinger.MP* sin(deg2rad(glove_data.RingFinger.MR1)) + HandModell.RingFinger.DP * sin(deg2rad(glove_data.RingFinger.MR2)); 

FingerTipCoord.LittleFinger.x = HandModell.LittleFinger.PP* cos(deg2rad(glove_data.LittleFinger.MR0)) + HandModell.LittleFinger.MP* cos(deg2rad(glove_data.LittleFinger.MR1)) + HandModell.LittleFinger.DP * cos(deg2rad(glove_data.LittleFinger.MR2)); 
FingerTipCoord.LittleFinger.y = HandModell.LittleFinger.PP* sin(deg2rad(glove_data.LittleFinger.MR0)) + HandModell.LittleFinger.MP* sin(deg2rad(glove_data.LittleFinger.MR1)) + HandModell.LittleFinger.DP * sin(deg2rad(glove_data.LittleFinger.MR2)); 

FingerTipCoord.Thumb.x = HandModell.Thumb.PP* cos(deg2rad(glove_data.Thumb.MR0)) + HandModell.Thumb.DP * cos(deg2rad(glove_data.Thumb.MR1)); 
FingerTipCoord.Thumb.y = HandModell.Thumb.PP* sin(deg2rad(glove_data.Thumb.MR0)) + HandModell.Thumb.DP * sin(deg2rad(glove_data.Thumb.MR1)); 

end






