function y = get_cross_point(A_start,A_end,B_start,B_end)
% line 1
if A_start(1) == A_end(1)% 竖线
    flag_one = 1;
elseif A_start(2) == A_end(2)%横线
    flag_one = 2;
else
    k1 = (A_start(2) - A_end(2) )/(A_start(1) - A_end(1));
    b1 = A_start(2) - k1 * A_start(1);
    flag_one = 3;
end
%line 2
if B_start(1) == B_end(1)% 竖线
    flag_two = 1;
elseif B_start(2) == B_end(2)%横线
    flag_two = 2;
else
    k2 = (B_start(2) - B_end(2) )/(B_start(1) - B_end(1));
    b2 = B_start(2) - k2 * B_start(1);
    flag_two = 3;
end
switch flag_one
    case 1
        switch flag_two
            case 1
                error('Error! foot piont cross error in get_cross_cog_margin')
            case 2
                y = [A_start(1), B_start(2)];
            case 3
                y = [A_start(1), k2 * A_start(1) + b2];                
        end
    case 2
        switch flag_two
            case 1
                y = [A_start(2), B_start(1)];
            case 2
                error('Error! foot piont cross error in get_cross_cog_margin')
            case 3
                y = [(A_start(2) - b2) / k2, A_start(2)];
        end        
    case 3
        switch flag_two
            case 1
                y = [B_start(1), k1 * B_start(1) + b1];
            case 2
                y = [(B_start(2) - b1) / k1, B_start(2)];
            case 3
                if k1==k2
                    error('Error! foot piont cross error in get_cross_cog_margin')
                else
                    y = [ (b1-b2)/(k2-k1), k1 * (b1-b2)/(k2-k1) + b1 ];
                end
        end
end
end