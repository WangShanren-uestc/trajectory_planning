function pos = foot_troj_plan_dual_circle(point_desired,pos_init,height)
global N;


Steps_Length = sqrt( (point_desired(1) - pos_init(1))^2 + (point_desired(2) - pos_init(2))^2);


    centre = Steps_Length/2 - Leg_Length * sin(deg2rad(Initial_Angle + Leg_Swing_Max));
    for i = 1:N
        feet_h = get_dual_heart_line_height ( Steps_Length,height);
        [p_foot(1),p_foot(2)] = dual_heart_line(Steps_Length,feet_h,centre);
        
    end
    

    
end