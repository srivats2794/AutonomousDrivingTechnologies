function [steer_ang,deviation_distance,deviation_angle] = Team_11_Steer_Control_Function(ml,bl,mr,br,z_upper,z_lower,pic_width)
% This function takes in the slope and intercepts of the lanes from the
% lane recognition function, finds the center lane and calculates the
% desired steering input.
% 4/10/18
% Team 11

    %Finding Co-ordinates
    z_lf = z_upper;
    x_lf = (z_lf - bl)/ml;
    z_rf = z_upper;
    x_rf = (z_rf - br)/mr;
    z_ln = z_lower;
    x_ln = (z_ln - bl)/ml;
    z_rn = z_lower;
    x_rn = (z_rn - br)/mr;
    
    % find center
    x_cn = 0.5*(x_ln + x_rn);
    x_cf = 0.5*(x_lf + x_rf);
    z_cn = 0.5*(z_ln + z_rn);
    z_cf = 0.5*(z_lf + z_rf);
    
    % uncomment to plot center line
    plot([x_cn x_cf],[z_cn z_cf],'--b','LineWidth',2);
    
    % calculate error
    deviation_ang = atan((x_cf - x_cn)/(z_cf - z_cn));
    deviation_distance = x_cf - 0.5*pic_width;
    
    % define gaine
    k1 = 1/(pic_width*0.5);% try2: 0.75);% try1: 0.5);
    k2 = 1/120;%try2: 90;% try1: 100;
    
    % calculate steering input
    y = 0.5 + (k1*deviation_distance + k2*deviation_ang);
    
    % bound steering input
    if (y>0.9)
        steer_ang = 0.9;
    elseif(y<0.1)
        steer_ang = 0.1;
    else
        steer_ang = y;
    end
              
end

    
    
    