clc;
clear all

l_1 = 1;
p_x = 2;
p_y = 1;
a = pi/6;

c_a = cos(a);
s_a = sin(a);

det = (p_x*c_a + p_y*s_a)^2 - (p_x^2 + p_y^2 - l_1^2);

q_31 = p_x*c_a + p_y*s_a + sqrt(det);
q_32 = p_x*c_a + p_y*s_a - sqrt(det);



q_11 = atan2(p_y -q_31*s_a, p_x - q_31*c_a);
q_12 = atan2(p_y -q_32*s_a, p_x - q_32*c_a);

q_21 = a - q_11;
q_22 = a - q_12;

fprintf("First solution: \n");
fprintf("q_1 = %f\n", q_11);
fprintf("q_2 = %f\n", q_21);
fprintf("q_3 = %f\n", q_31);

fprintf("Second solution: \n");
fprintf("q_1 = %f\n", q_12);
fprintf("q_2 = %f\n", q_22);
fprintf("q_3 = %f\n", q_32);
