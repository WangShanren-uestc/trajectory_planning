function y = get_angle_through_position( LF_Foot_Pos,RF_Foot_Pos,LB_Foot_Pos,RB_Foot_Pos )


global F_Leg_Length B_Leg_Length Body_Par  

LF = get_angle(LF_Foot_Pos,Body_Par(1,:),F_Leg_Length,1);
RF = get_angle(RF_Foot_Pos,Body_Par(2,:),F_Leg_Length,1);
LB = get_angle(LB_Foot_Pos,Body_Par(3,:),B_Leg_Length,-1);
RB = get_angle(RB_Foot_Pos,Body_Par(4,:),B_Leg_Length,-1);
y = [ LF,RF,LB,RB];


end

function y = get_angle(Foot_Pos,Offset,Leg_Length,sgn)

theta_zero = atan( (Offset(2) - Foot_Pos(2)) / (Foot_Pos(3) - Offset(3) ) );

delta = Foot_Pos(1) - Offset(1);
kexi = Leg_Length(1) + Foot_Pos(3) * cos(theta_zero) - Offset(3) * cos(theta_zero) - Foot_Pos(2) * sin(theta_zero) + Offset(2) * sin(theta_zero);

theta_two = sgn * acos( (delta^2 + kexi^2 - Leg_Length(2)^2 - Leg_Length(3)^2)/(2 * Leg_Length(2) * Leg_Length(3) ) );

phi = delta + Leg_Length(3) * sin(theta_two);
if phi == 0
    phi = phi + 0.00000001;
end
theta_one = 2 * atan( ( kexi + sqrt(kexi^2 - phi * (Leg_Length(3) * sin(theta_two) - delta) ) ) / phi );

y = [-1 * theta_zero,theta_one,theta_two];

end
