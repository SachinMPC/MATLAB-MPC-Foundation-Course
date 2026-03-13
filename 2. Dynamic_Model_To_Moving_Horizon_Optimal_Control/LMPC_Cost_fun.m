function obj_fn = LMPC_Cost_fun( Uf)

global phy gam n_st n_ip n_pred Wx Wu xk

zk = xk ;
obj_fn = 0 ;

for j = 1 : n_pred
    c1 = (j-1) * n_ip+1; c2 = j * n_ip ;
    uk = Uf(c1:c2) ;
    zkplus1 = phy * zk + gam * uk ;
    obj_fn = obj_fn + zkplus1' * Wx * zkplus1 ;
    obj_fn = obj_fn + uk' * Wu * uk ;
    zk = zkplus1 ;
end

end 