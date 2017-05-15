function [A_Adj,B_Adj,C_Adj,D_Adj] = cog_adjust( A,B,C,D,offset_vector )

global N
N=5000;
A_Adj = zeros(N,3);
B_Adj = zeros(N,3);
C_Adj = zeros(N,3);
D_Adj = zeros(N,3);
x_addition =  offset_vector(1)/N;
y_addition =  offset_vector(2)/N;

for i=1:N
    A_Adj(i,:) = [A(1) - x_addition * i, A(2) - y_addition * i ,A(3)] ;
    B_Adj(i,:) = [B(1) - x_addition * i, B(2) - y_addition * i ,B(3)] ;
    C_Adj(i,:) = [C(1) - x_addition * i, C(2) - y_addition * i ,C(3)] ;
    D_Adj(i,:) = [D(1) - x_addition * i, D(2) - y_addition * i ,D(3)] ;
end

