function y = get_triangle_innerheart( A,B,C)

a = sqrt( (B(1) - C(1))^2 + (B(2) - C(2))^2 );
b = sqrt( (A(1) - C(1))^2 + (A(2) - C(2))^2 );
c = sqrt( (B(1) - A(1))^2 + (B(2) - A(2))^2 );

y(1) = ( a * A(1) + b * B(1) + c * C(1) ) / (a + b + c) ;
y(2) = ( a * A(2) + b * B(2) + c * C(2) ) / (a + b + c) ;

end

