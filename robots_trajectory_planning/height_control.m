function y = height_control(desired_height)

global F_Leg_Length B_Leg_Length Body_Par
% F_Leg_Length = [1.4, 27.5, 23];
% B_Leg_Length = [1.4, 29,   28];
% Body_Par = [ 26.8534, 13.5, -6.5;...
%     26.8534, -13.5, -6.5;...
%     -25.6466, 16.5,  0;...
%     -25.6466,-16.5, 0];
num = 2500;
add = abs( (desired_height -sum(B_Leg_Length))/num);
LF_Pos = [Body_Par(1,1), Body_Par(1,2), Body_Par(1,3) - sum(F_Leg_Length) ];
RF_Pos = [Body_Par(2,1), Body_Par(2,2), Body_Par(2,3) - sum(F_Leg_Length) ];
LB_Pos = [Body_Par(3,1), Body_Par(3,2), Body_Par(3,3) - sum(B_Leg_Length) ];
RB_Pos = [Body_Par(4,1), Body_Par(4,2), Body_Par(4,3) - sum(B_Leg_Length) ];
y = zeros(num,12);
for i=1:num
    LF_Pos(3) = LF_Pos(3) + add;
    RF_Pos(3) = RF_Pos(3) + add;
    LB_Pos(3) = LB_Pos(3) + add;
    RB_Pos(3) = RB_Pos(3) + add;
    
    y(i,:) = [reverse_kinematics( LF_Pos,1),reverse_kinematics( RF_Pos,2),...
        reverse_kinematics( LB_Pos,3),reverse_kinematics( RB_Pos,4)];
    
end

end

