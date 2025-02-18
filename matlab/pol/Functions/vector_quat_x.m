function V_q = vector_quat_x(V)

V_q = [0   V(3)  -V(2)  V(1);
    -V(3)    0    V(1)  V(2); 
     V(2) -V(1)     0   V(3); 
    -V(1) -V(2)  -V(3)    0];

end