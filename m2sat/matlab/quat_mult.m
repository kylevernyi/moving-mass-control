function Q3 = quat_mult(Q1,Q2)

q10 = Q1(1); q11 = Q1(2); q12 = Q1(3); q13 = Q1(4);


Q1_mult = [q10 -q11 -q12 -q13;
           q11  q10  q13 -q12;
           q12 -q13  q10  q11;
           q13  q12 -q11  q10];

Q3 = Q1_mult*Q2;

end