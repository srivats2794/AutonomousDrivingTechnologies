function steer_ang = Steer_Control_Function(ml,bl,mr,br,z_upper,z_lower,pic_width)

    %Finding Co-ordinates
    z_lf = z_upper;
    x_lf = (z_lf - bl)/ml;
    z_rf = z_upper;
    x_rf = (z_rf - br)/mr;
    z_ln = z_lower;
    x_ln = (z_ln - bl)/ml;
    z_rn = z_lower;
    x_rn = (z_rn - br)/mr;
    
    x_cn = 0.5*(x_ln + x_rn);
    x_cf = 0.5*(x_lf + x_rf);
    z_cn = 0.5*(z_ln + z_rn);
    z_cf = 0.5*(z_lf + z_rf);
    
%     plot([x_cn x_cf],[z_cn z_cf],'--b','LineWidth',2);
    
    deviation_ang = atan((x_cf - x_cn)/(z_cf - z_cn));
    deviation_distance = x_cf - 0.5*pic_width;
    
    k1 = 1/(pic_width*0.5);% try2: 0.75);% try1: 0.5);
    k2 = 1/120;%try2: 90;% try1: 100;
    
    y = 0.5 + (k1*deviation_distance + k2*deviation_ang);
    
    if (y>0.9)
        steer_ang = 0.9;
    elseif(y<0.1)
        steer_ang = 0.1;
    else
        steer_ang = y;
    end
              
end

    
    
    