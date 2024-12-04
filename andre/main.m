syms q1 q2 q3 q4 L
% theta alpha a d

(-cos(q1+q3)+L*sin(q1)*cos(q4))^2 + (-sin(q1+q3)+L*cos(q1)*cos(q4))^2

matrix = [
    q1,0,0,0;
    0,pi/2,0,q2;
    0,0,0,q3;
    q4,0,L,0;
];

%T = DH_HTM(matrix, "r")

%T=simplify(matrix,'Steps',20);

%syms alpha beta gamma

%eul2rotm([alpha, beta, gamma], "ZYX")
