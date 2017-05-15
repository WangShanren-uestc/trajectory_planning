%输入 u：1~12为关节角度，13~16为地面接触力,16~20为标志位

%Steps:
%------  Initialize model parameter;
%------  get angle array and contact force from model;
%------  forward kinematics;
%------  decide leg swing order;
%------  calculate cog stable margin and cog offset;
%------  adjust to optimal cog position;
%------  plan foot trajectory;
%------  inverse kinematics and swing;
%------  get contact force values and stop swing;

%------  Initialize model parameter;
global F_Leg_Length B_Leg_Length Body_Par Stable_Magin_Min Cog Order Height Swing_Flag
Order = 1;
Cog = [0, 0, 0];

Height = 45;
Stable_Magin_Min = 3; %3cm
F_Leg_Length = [1.4, 27.5, 23];
B_Leg_Length = [1.4, 29,   28];
Body_Par = [ 26.8534, 13.5, -6.5;...
    26.8534, -13.5, -6.5;...
    -25.6466, 16.5,  0;...
    -25.6466,-16.5, 0];

Swing_Flag = 0;
%--------ROS--------%
pub = rospublisher('/Designed_AngleArray', 'std_msgs/Float32MultiArray');
AngleArray_Pub = rosmessage(pub);
% sub = rossubscriber('/Designed_AngleArray');
% AngleArray_Sub = receive(sub,10)

%------init height control----only for adams----%
angle_pub = height_control(Height);
while(1)
    for i = 1 : size(angle_pub, 1)
        AngleArray_Pub.Data(1:12) = angle_pub(i,:);        
        send(pub,AngleArray_Pub);        
    end
end


%
% count = 0;
% while(count<10000)
%     count = count + 1;
%     %------  get information from model;
%     angle = reshape(u(1:12),4,3);
%     contact_force = u(13:16);
%
%     %------  forward kinematics;
%     [LF,RF,LB,RB] = forward_kinematics( angle);
%     %------  decide leg swing order;
%     four_leg_on_ground = isequal(4,sum( is_on_ground(contact_force)));
%
%     %****************************STATE MACHINE*******************************%
%     %************calculate cog stable margin and cog offset********************%
%     %******************** swing order 1-4-2-3-1*********************************%
%     switch Order
%         case 1
%             %****************************************************************************%
%             %****************** before swing , adjust cog first***************************%
%             %****************************************************************************%
%             cog_offset  = get_cog_margin( RF, LB, RB, Cog );
%             if (~isequal(cog_offset, [0, 0]) ) && four_leg_on_ground && Swing_Flag ~=1
%                 [A_Adj, B_Adj, C_Adj, D_Adj] = cog_adjust( LF, RF, LB, RB, cog_offset );
%                 angle_pub = get_angle_through_position( A_Adj, B_Adj, C_Adj, D_Adj );
%                 ros.pub(angle_pub);%need to send to robot
%                 return
%                 %**************************************************************************%
%                 %****************** after adjust cog ,swing********************************%
%                 %**************************************************************************%
%             elseif isequal(cog_offset, [0, 0]) && four_leg_on_ground && Swing_Flag ~=1
%                 next_foot_point = find_next_desired_point( Order);
%                 troj = foot_troj_plan(next_foot_point,LF,Height/10);
%                 angle_pub = zeros(size(troj,1),3);
%                 for i=1:size(troj,1)
%                     angle_pub(i,:) = reverse_kinematics( troj(i,:),Order) ;
%                 end
%                 ros.pub(angle_pub);%need to send to robot
%                 Swing_Flag = 1;
%                 return
%                 %**************************************************************************%
%                 %****************** Order to next swing leg*******************************%
%                 %**************************************************************************%
%             else
%                 four_leg_on_ground = isequal(4,sum( is_on_ground(u(13:16))));
%                 if four_leg_on_ground
%                     Order = 4;
%                     Swing_Flag = 0;
%                 end
%             end
%
%         case 2
%             cog_offset  = get_cog_margin( LF,LB,RB,Cog );
%
%         case 3
%             cog_offset  = get_cog_margin( LF,RF,RB,Cog );
%
%         case 4
%             cog_offset  = get_cog_margin( LF,RF,LB,Cog );
%
%         otherwise
%             error('Error! unexpected walk order flag')
%     end
%
%
% end







