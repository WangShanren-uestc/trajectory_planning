function y = get_cross_cog_margin( LF_Pos, RF_Pos, LB_Pos, RB_Pos ,Order)

%return exact how cog shoule move and towards

global Stable_Magin_Min Cog Height
P = Cog;
switch Order
    case 1
        A = [get_cross_point(LF_Pos, RB_Pos, RF_Pos, LB_Pos), -1 * Height ];
        B = RF_Pos;
        C = RB_Pos;
    case 2
        A = [get_cross_point(LF_Pos, RB_Pos, RF_Pos, LB_Pos), -1 * Height ];
        B = LF_Pos;
        C = LB_Pos;
    case 3
        A = [get_cross_point(LF_Pos, RB_Pos, RF_Pos, LB_Pos), -1 * Height ];
        B = RF_Pos;
        C = RB_Pos;
    case 4
        A = [get_cross_point(LF_Pos, RB_Pos, RF_Pos, LB_Pos), -1 * Height ];
        B = LF_Pos;
        C = LB_Pos;
end

optimal_cog = get_triangle_innerheart( A,B,C);
Stable_Magin_Min = min(cal_p2line_dist(A,B,optimal_cog),10);
a = same_side(A,B,C,P);
b = same_side(B,C,A,P);
c = same_side(C,A,B,P);

if a==1 && b==1 && c==1
    L_AB=cal_p2line_dist(A,B,P);
    L_AC=cal_p2line_dist(A,C,P);
    L_BC=cal_p2line_dist(C,B,P);

    if min(floor([L_AB,L_AC,L_BC])) == max(floor([L_AB,L_AC,L_BC]))
        y = [0,0];
        return
    end
    if min([L_AB,L_AC,L_BC]) - Stable_Magin_Min >=0
        y = [0,0];
        return
    else
        y = [optimal_cog(1) - P(1), optimal_cog(2) - P(2)];
        return
    end
else
    y = [optimal_cog(1) - P(1), optimal_cog(2) - P(2)];
    return
end
end

%-----------------------------------------------------%
function y = same_side(A,B,C,P)

ab = B - A;
ac = C - A;
ap = P - A;
if length(ab)<=2
    ab = [ab,0];
    ac = [ac,0];
    ap = [ap,0];
end

if length(ab)>3 || length(ac)>3 || length(ap)>3
    error('Error! the length of vector for cross multiple must be under 3')
end
v1 = cross(ab,ac);
v2 = cross(ab,ap);

if dot(v1,v2)>0
    y = 1;
else
    y = 0;
end
end

function y = cal_p2line_dist(A,B,P)

if A(1) ==B(1)
    y = abs(P(1) - A(1));
    return
else
    k = (A(2) - B(2))  / (A(1) - B(1));
    b = A(2) - k * A(1);
    y = abs( k * P(1) + b - P(2) )/sqrt(k^2 + 1^2);
end

end
