function q = Eul2Qnb(angleX, angleY, angleZ)
    cPhi2 = cos(0.5*angleX); cThe2 = cos(0.5*angleY); cPsi2 = cos(0.5*angleZ);
    sPhi2 = sin(0.5*angleX); sThe2 = sin(0.5*angleY); sPsi2 = sin(0.5*angleZ);
    q0 = cPhi2*cThe2*cPsi2 + sPhi2*sThe2*sPsi2;
    q1 = sPhi2*cThe2*cPsi2 - cPhi2*sThe2*sPsi2;
    q2 = cPhi2*sThe2*cPsi2 + sPhi2*cThe2*sPsi2;
    q3 = cPhi2*cThe2*sPsi2 - sPhi2*sThe2*cPsi2;

    q = [q0 q1 q2 q3];
end

