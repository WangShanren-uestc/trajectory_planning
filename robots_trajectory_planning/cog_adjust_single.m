function [A_Adj,B_Adj,C_Adj,D_Adj] = cog_adjust_single( A,B,C,D,offset_vector )

%  normalization
offset = offset_vector/sqrt(offset_vector(1)^2 + offset_vector(2)^2);

x_addition =  offset(1)/10;
y_addition =  offset(2)/10;

A_Adj = [A(1) - x_addition , A(2) - y_addition ,A(3)] ;
B_Adj = [B(1) - x_addition , B(2) - y_addition ,B(3)] ;
C_Adj = [C(1) - x_addition , C(2) - y_addition ,C(3)] ;
D_Adj = [D(1) - x_addition , D(2) - y_addition ,D(3)] ;
