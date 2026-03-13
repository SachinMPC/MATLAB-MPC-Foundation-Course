function mpc = Load_BlackBox_Model (operating_pt, model_type)
if operating_pt==1
    if (model_type==1)
        load QTank_MP_GreyBox_OE_SS.mat
    elseif (model_type==2)
        load QTank_MP_GreyBox_INV_SS.mat
    elseif (model_type == 3)
        load QTank_MP_BlackBox_OE_SS.mat
    else
        load QTank_MP_BlackBox_INV_SS_6.mat
    end
else
    if model_type==1
        load QTank_NMP_GreyBox_OE_SS.mat
    elseif model_type==2
        load QTank_NMP_GreyBox_INV_SS.mat
    elseif model_type==3
        load QTank_NMP_BlackBox_OE_SS.mat
    else
        load QTank_NMP_BlackBox_INV_SS_6.mat
    end
end
mpc.phy=idmod.phy;
mpc.gam=idmod.gam;
mpc.C_mat=idmod.C;
mpc.n_st = length(mpc.phy) ;
[mpc.n_op, mpc.n_st ] = size(mpc.C_mat)  ; 
[mpc.n_st, mpc.n_ip] = size(mpc.gam) ;
mpc.L_mat=idmod.L_mat;
end