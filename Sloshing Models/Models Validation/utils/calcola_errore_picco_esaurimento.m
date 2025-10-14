function err_max = calcola_errore_picco_esaurimento(Altezza_conf,Altezza_rif,time_conf,time_rif,time_fin)

Altezza_conf_rest = Altezza_conf(time_conf>=time_fin);
Altezza_rif_rest = Altezza_rif(time_rif>=time_fin);

err_max = (max(Altezza_conf_rest) - max(Altezza_rif_rest))*100/max(Altezza_rif_rest);

end

