
function [sys, x0, str, ts] = cog_move_test( t, x, u, flag )
%------  Initialize model parameter;
global F_Leg_Length B_Leg_Length Body_Par Stable_Magin_Min Cog Order Height Count angle_pub  Height_Adjust_Flag cog_offset
global next_foot_point troj Swing_Flag RF_Swing_Flag LB_Swing_Flag RB_Swing_Flag Count_LF Count_RB Count_RF Count_LB A_Adj B_Adj  C_Adj  D_Adj Delay
Height = 45;

Stable_Magin_Min = 8; %3cm
F_Leg_Length = [1.4, 27.5, 23];
B_Leg_Length = [1.4, 29,   28];
Body_Par = [ 26.8534, 13.5, -6.5;...
    26.8534, -13.5, -6.5;...
    -25.6466, 16.5,  0;...
    -25.6466,-16.5, 0];

switch flag
    case 0
        Height_Adjust_Flag=0;
        Swing_Flag = 0;
        RF_Swing_Flag = 0;
        LB_Swing_Flag = 0;
        RB_Swing_Flag = 0;
        
        Count_LF = 0;
        Count_RF = 0;
        Count_LB = 0;
        Count_RB = 0;
        Order = 3;
        Count = 0;
        Delay = 0;
        cog_offset = [0,0];
        next_foot_point = [0,0,0];
        troj = zeros(40,3);
        Cog = [0, 0, 0];
        angle_pub = zeros(8000,12);
        angle_pub = height_control(Height);
        [sys,x0,str,ts]=mdlInitializeSizes;
    case 3
        sys=mdlOutputs(t,x,u);     
    case {1,2,4,9}
        sys=[];
    otherwise
        error('Simulink:blocks:unhandledFlag.');
end
end

function [sys,x0,str,ts]=mdlInitializeSizes
sizes = simsizes;
sizes.NumContStates  = 0;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 12;
sizes.NumInputs      = 16;
sizes.DirFeedthrough = 1;
sizes.NumSampleTimes = 1;   % at least one sample time is needed
sys = simsizes(sizes);
x0  = [];
str = [];
ts  = [0.01,0];
end

% function sys=mdlGetTimeOfNextVarHit(t,x,u)
% sampleTime = 0.1;    %  Example, set the next hit to be one second later.
% sys = t + sampleTime;
% end


function sys=mdlOutputs(t,x,u)
global  Cog Order Height Swing_Flag Count angle_pub Height_Adjust_Flag
global next_foot_point troj  A_Adj B_Adj  C_Adj  D_Adj N cog_offset
global Delay

if Delay<1000
    Delay = Delay + 1;
    sys(1:12) = u(1:12);
else
    
    if max( floor(100*u(1:12)')-floor(100*angle_pub(size(angle_pub, 1),:) ))==0 && Height_Adjust_Flag==0
        Height_Adjust_Flag = 1;
        Count = 0;
    end
    %------init height control----only for adams----%
    if Height_Adjust_Flag==0
        Count = Count + 1;
        if Count <= size(angle_pub, 1)
            sys(1:12) = angle_pub(Count,:);
            return
        else
            sys(1:12) = angle_pub(size(angle_pub, 1),:);
            return
        end
    end
    
    
    
    if Height_Adjust_Flag==1
        angle = [u(1),u(2),u(3);u(4),u(5),u(6);u(7),u(8),u(9);u(10),u(11),u(12)];
        [LF,RF,LB,RB] = forward_kinematics(angle ) ;
        switch Order
            case 1               
                    Order = 4;
                    sys(1:12) = u(1:12)';                 
            case 2               
                    Order = 3;
                    sys(1:12) = u(1:12)';                   
            case 3
                if Count==0
                    next_foot_point = find_next_desired_point(Order);
                    Count = Count + 1;
                end
                cog_offset  = get_cross_cog_margin( LF, RF, next_foot_point, RB, Order );
                if ~isequal( cog_offset, [0, 0] )                    
                    [A_Adj, B_Adj, C_Adj, D_Adj] = cog_adjust_single( LF, RF, LB, RB, cog_offset );
                    sys(1:12) = get_angle_through_position( A_Adj, B_Adj, C_Adj, D_Adj );
                    return                    
                else
                    Count = 0;
                    Order = 1;
                    sys(1:12) = u(1:12)';
                    return
                end
            case 4                
                if Count ==0
                    next_foot_point = find_next_desired_point(Order);
                    Count = Count + 1;
                end
                cog_offset  = get_cross_cog_margin( LF, RF, LB, next_foot_point, Order );
                if ~isequal( cog_offset, [0, 0] )                    
                    [A_Adj, B_Adj, C_Adj, D_Adj] = cog_adjust_single( LF, RF, LB, RB, cog_offset );                    
                    sys(1:12) = get_angle_through_position( A_Adj, B_Adj, C_Adj, D_Adj );
                    return
                else
                    Count = 0;
                    Order = 2;
                    sys(1:12) = u(1:12)';
                    return
                end
        end
    end
end
end



