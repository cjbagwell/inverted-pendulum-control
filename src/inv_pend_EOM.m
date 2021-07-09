% JORDAN BAGWELL
% JEFFREY MAYS
% NICHOLAS SCIORTINO

function dx = inv_pend_EOM(x, m1, m2, L, g, b, u)

ST = sin(x(3));
CT = cos(x(3));

dx(1,1) = x(2);
dx(2,1) = (4*u - 4*b*x(2) + 2*L*m2*ST*x(4)^2 + 3*g*m2*CT*ST)/(4*m1 + 4*m2 - 3*m2*CT^2)*x(4);
dx(3,1) = x(4);
dx(4,1) = -(3*(2*u*CT + 2*g*m1*ST + 2*g*m2*ST - 2*b*CT*x(3) + L*m2*CT*ST*x(4)^2))/(L*(4*m1 + 4*m2 - 3*m2*CT^2));

