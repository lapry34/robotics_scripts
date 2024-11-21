syms q1 q2 q3 a1 a3
% theta alpha a d
matrix = [
    q1, -pi/2, a1, 0;
    0, pi/2, 0, q2;
    q3, 0, a3, 0
];

T = DH_HTM(matrix, "r");
w_T_0 = [
    0.5, -sqrt(3)/2, 0, 1;
    sqrt(3)/2, 0.5, 0, 3;
    0,0,1,0;
    0,0,0,1;
]
w_T_0 * T
%T=simplify(matrix,'Steps',20);

%syms alpha beta gamma

%eul2rotm([alpha, beta, gamma], "ZYX")
