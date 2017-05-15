function y = find_next_desired_point( Leg_Order)
global F_Leg_Length B_Leg_Length Body_Par Height
Steps = -15;
switch Leg_Order
    case 1
        bound = get_bound_circle(F_Leg_Length,Body_Par(1,:));       
        y =[bound(1) + Steps, bound(2), -1 * Height];
    case 2
        bound = get_bound_circle(F_Leg_Length,Body_Par(2,:));      
        y =[bound(1) + Steps , bound(2) , -1 * Height];
    case 3
        bound = get_bound_circle(B_Leg_Length,Body_Par(3,:));    
        y =[bound(1) + Steps , bound(2) , -1 * Height];
    case 4
        bound = get_bound_circle(B_Leg_Length,Body_Par(4,:));      
        y =[bound(1) + Steps , bound(2) , -1 * Height];
    otherwise
        error('Error! unexpected leg order')
end

end
function y = get_bound_circle(Leg_Length,Offset)
global   Cog Height
centre_point = Cog + Offset;
length = sum(Leg_Length);
r  = sqrt(length^2 - Height^2);
if isreal(r)
    y = [centre_point(1), centre_point(2), r];
else
    error('Error! leg height out of range')
end
end

