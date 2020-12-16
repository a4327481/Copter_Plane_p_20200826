syms delta alpha beta gamma
syms delta0 alpha0 beta0 gamma0
syms delta1 alpha1 beta1 gamma1 
syms cos_delta sin_delta
q0=[cos(delta0) sin(delta0)*cos(alpha0) sin(delta0)*cos(beta0) sin(delta0)*cos(gamma0)];
q1=[cos(delta1) sin(delta1)*cos(alpha1) sin(delta1)*cos(beta1) sin(delta1)*cos(gamma1)];
q1_inv=[cos(delta1) -sin(delta1)*cos(alpha1) -sin(delta1)*cos(beta1) -sin(delta1)*cos(gamma1)];
q=quatmultiply_m(q0 , q1);
q_s=subs(q,[cos(delta0)*cos(delta1),sin(delta0)*sin(delta1)],[cos_delta,sin_delta])
q0a=quatmultiply_m(q1_inv,quatmultiply_m(q0 , q1))