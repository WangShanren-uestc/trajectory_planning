
function [sys, x0, str, ts] = main_process_module( t, x, u, flag )
%------  Initialize model parameter;
global F_Leg_Length B_Leg_Length Body_Par Stable_Magin_Min Cog Order Height Count angle_pub  Height_Adjust_Flag cog_offset
global next_foot_point troj Swing_Flag RF_Swing_Flag LB_Swing_Flag RB_Swing_Flag Count_LF Count_RB Count_RF Count_LB A_Adj B_Adj  C_Adj  D_Adj Delay
Height = 45;

Stable_Magin_Min = 8; %3cm
F_Leg_Length = [1.4, 27.5, 23];
B_Leg_Length = [1.4, 29,   28];
Body_Par = [ 27.1129441624, 13.5, -6.5;...
    27.1129441624, -13.5, -6.5;...
    -25.3870558376, 16.5,  0;...
    -25.3870558376,-16.5, 0];

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
        %     case 4
        %         sys=mdlGetTimeOfNextVarHit(t,x,u);
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
ts  = [0.1 0];
end

% function sys=mdlGetTimeOfNextVarHit(t,x,u)
% sampleTime = 0.1;    %  Example, set the next hit to be one second later.
% sys = t + sampleTime;
% end


function sys=mdlOutputs(t,x,u)
global  Cog Order Height Swing_Flag Count angle_pub Height_Adjust_Flag
global next_foot_point troj  A_Adj B_Adj  C_Adj  D_Adj N cog_offset
global Delay



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
    C_Force = [u(13),u(14),u(15),u(16)];
    [LF,RF,LB,RB] = forward_kinematics(angle ); 
    switch Order
        case 1
            if Swing_Flag==0%adjust cog
                Swing_Flag = 1;
                next_foot_point = find_next_desired_point(Order);
                troj = foot_troj_plan_circle(next_foot_point,LF,Height/10);
                sys(1:12) = u(1:12)';
                return
            else % swing leg
                Count = Count +1;
                if ~isequal(troj,[0,0,0])
                    if Count <=size(troj,1)
                        sys(1:3) = reverse_kinematics( troj(Count,:),Order) ;
                        sys(4:12) = u(4:12)';
                        return
                    else
                        if max(abs(round(troj(size(troj,1),:)) -  round(LF)))<=0.5  ||  C_Force(1)>50
                            Count = 0;
                            Order = 4;
                            Swing_Flag = 0;
                            sys(1:12) = u(1:12)';
                            return
                        else
                            sys(1:3) = reverse_kinematics( troj(size(troj,1),:),Order) ;
                            sys(4:12) = u(4:12)';
                            return
                        end
                    end
                else
                    Count = 0;
                    Order = 4;
                    Swing_Flag = 0;
                    sys(1:12) = u(1:12)';
                    return
                end
            end
        case 2
            if Swing_Flag==0%adjust cog
                Swing_Flag = 1;
                next_foot_point = find_next_desired_point(Order);
                troj = foot_troj_plan_circle(next_foot_point,RF,Height/10);
                sys(1:12) = u(1:12)';
                return                
            else % swing leg
                Count = Count +1;
                if ~isequal(troj,[0,0,0])
                    if Count <=size(troj,1)
                        sys(1:3) = u(1:3)';
                        sys(4:6) = reverse_kinematics( troj(Count,:),Order) ;
                        sys(7:12) = u(7:12)';
                        return
                    else
                        if max(abs(round(troj(size(troj,1),:)) -  round(RF)))<=0.5 ||  C_Force(2)>50
                            Count = 0;
                            Order = 3;
                            Swing_Flag = 0;
                            sys(1:12) = u(1:12)';
                            return
                        else
                            sys(1:3) = u(1:3)';
                            sys(4:6) = reverse_kinematics( troj(size(troj,1),:),Order) ;
                            sys(7:12) = u(7:12)';
                            return
                        end
                    end
                else
                    Count = 0;
                    Order = 3;
                    Swing_Flag = 0;
                    sys(1:12) = u(1:12);
                    return
                end
            end
        case 3
            if Swing_Flag==0%adjust cog
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
                    Swing_Flag = 1;
                    troj = foot_troj_plan_circle(next_foot_point,LB,Height/10);
                    sys(1:12) = u(1:12)';
                    return
                end
            else % swing leg
                Count = Count +1;
                if ~isequal(troj,[0,0,0])
                    if Count <=size(troj,1)
                        sys(1:6) = u(1:6)';
                        sys(7:9) = reverse_kinematics( troj(Count,:),Order) ;
                        sys(10:12) = u(10:12)';
                        return
                    else
                        if max(abs(round(troj(size(troj,1),:)) -  round(LB)))<=0.5 ||  C_Force(3)>50
                            Count = 0;
                            Order = 1;
                            Swing_Flag = 0;
                            sys(1:12) = u(1:12)';
                            return
                        else
                            sys(1:6) = u(1:6)';
                            sys(7:9) = reverse_kinematics( troj(size(troj,1),:),Order) ;
                            sys(10:12) = u(10:12)';
                            return
                        end
                    end
                else
                    Count = 0;
                    Order = 1;
                    Swing_Flag = 0;
                    sys(1:12) = u(1:12)';
                    return
                end
            end
        case 4
            if Swing_Flag==0%adjust cog
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
                    Swing_Flag = 1;
                    troj = foot_troj_plan_circle(next_foot_point,RB,Height/10);
                    sys(1:12) = u(1:12)';
                    return
                end
            else % swing leg
                Count = Count +1;
                if ~isequal(troj,[0,0,0])
                    if Count <=size(troj,1)
                        sys(1:9) = u(1:9)';
                        sys(10:12) = reverse_kinematics( troj(Count,:),Order) ;
                        return
                    else
                        if max(abs(round(troj(size(troj,1),:)) -  round(RB)))<=0.5 ||  C_Force(4)>50
                            Count = 0;
                            Order = 2;
                            Swing_Flag = 0;
                            sys(1:12) = u(1:12)';
                            return
                        else
                            sys(1:9) = u(1:9)';
                            sys(10:12) = reverse_kinematics( troj(size(troj,1),:),Order) ;
                            return
                        end
                    end
                else
                    Count = 0;
                    Order = 2;
                    Swing_Flag = 0;
                    sys(1:12) = u(1:12)';
                    return
                end
            end
    end
end
end



