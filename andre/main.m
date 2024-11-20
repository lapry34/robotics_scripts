syms q1 q2

matrix = [
  -pi/2, 0.2, 0, q1;
  pi/2, 0, q2, pi
];

DH_HTM(matrix, "r");
T=simplify(matrix,'Steps',20);

syms alpha beta gamma

eul2rotm([alpha, beta, gamma], "ZYX")
