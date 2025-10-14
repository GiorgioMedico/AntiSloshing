function sigma_int = errore_integrale_esaurimento(eta_mod,eta_exp,time_mod,time_exp,Tin,Tfin,normalized)

ind10_mod = time_mod > Tin & time_mod < Tfin;
interval_mod = find(ind10_mod);
time_mod_motion = time_mod(interval_mod);
eta_mod_motion = eta_mod(interval_mod);

ind10_exp = time_exp > Tin & time_exp < Tfin;
interval_exp = find(ind10_exp);
time_exp_res = time_exp(interval_exp);
eta_exp_res = eta_exp(interval_exp);

ind20_exp = time_exp > 0 & time_exp < Tin;
interval_exp = find(ind20_exp);
time_exp_motion = time_exp(interval_exp);
eta_exp_motion = eta_exp(interval_exp);

int_mod = trapz(time_mod_motion,eta_mod_motion);
int_exp_res = trapz(time_exp_res,eta_exp_res);
int_exp_motion = trapz(time_exp_motion,eta_exp_motion);

if normalized == 2
    % sigma_int = 100*(int_mod - int_exp)/(Tfin-Tin)/max(eta_exp_motion);
    sigma_int = 100*(int_mod - int_exp_res)/int_exp_res;
elseif normalized == 1
    sigma_int = 100*((int_mod - int_exp_res)/(Tfin-Tin))/(int_exp_motion/Tin);
else
    sigma_int = (int_mod - int_exp_res)/(Tfin-Tin);
end

end