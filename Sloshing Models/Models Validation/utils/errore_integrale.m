function sigma_int = errore_integrale(eta_mod,eta_exp,time_mod,time_exp,Te,normalized)

int_mod = time_mod<=Te;
time_mod_motion = time_mod(int_mod);
eta_mod_motion = eta_mod(int_mod);

int_exp = time_exp<=Te;
time_exp_motion = time_exp(int_exp);
eta_exp_motion = eta_exp(int_exp);

int_mod = trapz(time_mod_motion,eta_mod_motion);
int_exp = trapz(time_exp_motion,eta_exp_motion);

if normalized
    % sigma_int = 100*(int_mod - int_exp)/Te/max(eta_exp_motion);
    sigma_int = 100*(int_mod - int_exp)/int_exp;
else
    sigma_int = (int_mod - int_exp)/Te;
end

end