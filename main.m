
%1) save the glove raw data into a struct
glove_data = readRawData();

%2) define a hand modell
HandModell = defineHandModell();

%3) get (x,y) coordinate of tip of all 5 fingers
FingerTipCoord = getFingerTipCoord(HandModell,glove_data);

%4) get finger tip accelaration of all 5 fingers
FingerTipAcc = getFingerTipAcc(glove_data);

