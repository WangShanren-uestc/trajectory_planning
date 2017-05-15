function pos = foot_troj_plan(point_desired,pos_init,height)

if isequal(point_desired,pos_init)
    pos = [0,0,0];
    return
end
num = 1000;
x_add = (point_desired(1) - pos_init(1)) / 5 / num;
y_add = (point_desired(2) - pos_init(2)) / 5/ num;
z_up = height/num;
z_down = height/num;
for i=1:num   
    pos(i,1) = pos_init(1);
    pos(i,2) = pos_init(2);
    pos(i,3) = pos_init(3) + z_up * i;   
end
for i=num+1:6*num  
    pos(i,1) = pos_init(1) + x_add * (i-100);
    pos(i,2) = pos_init(2) + y_add * (i-100);
    pos(i,3) = pos(i-1,3) ;   
end
for i=6*num+1:7*num 
    pos(i,1) = pos(i-1,1) ;
    pos(i,2) = pos(i-1,2) ;
    pos(i,3) = pos(i-1,3) - z_down;   
end

% plot(x,y,'r.'),hold on
% axis equal
end