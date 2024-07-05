function [gz1_gz2] = gz_div(q1,q2,q1p,q2p)
  
determinante = (J + M_2*l_bi^2 + 2*M_2*l_bi*C_x + I_z + I_x*sin(q2)^2)*I_x - (M_2*l_bi*C_z*cos(q2))^2;
gz1_gz2 = [Ix;-M_2*l_bi*C_z*cos(q2)]/determinante;

end

