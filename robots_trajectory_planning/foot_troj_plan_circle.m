function pos = foot_troj_plan_circle(point_desired,pos_init,height)

if max(abs(round(point_desired(1:2) -  round(pos_init(1:2)))))<=1
    pos = [0,0,0];
    return
end
x_add = point_desired(1) - pos_init(1);
y_add = point_desired(2) - pos_init(2);
sgn_x = sign(x_add);
sgn_y = sign(y_add);
if x_add==0
    alpha = pi/2;
else    
    alpha = atan((y_add/x_add));   
end

%建立椭圆方程
a = sqrt(x_add^2 + y_add^2) / 2;
b = height;
num = 3000 ;
pos = zeros(num,3);
theta = pi;
theta_add = pi / num;
for i=1:num
    theta = theta - theta_add;
    x = a * cos(theta) + a;
    y = b * sin(theta);
    pos(i,:) = [sgn_x * x * cos(alpha) + pos_init(1), sgn_y * x * sin(alpha) + pos_init(2), pos_init(3) + y];
end
end