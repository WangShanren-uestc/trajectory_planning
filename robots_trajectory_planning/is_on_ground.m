function [ a,b,c,d ] = is_on_ground( contact_force )

if abs(contact_force(1))>0
    a = 1;
else
    a = 0;
end
if abs(contact_force(2))>0
    b = 1;
else
    b = 0;
end
if abs(contact_force(3))>0
    c = 1;
else
    c = 0;
end
if abs(contact_force(4))>0
    d = 1;
else
    d = 0;
end


