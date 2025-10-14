function y=calcola_errore_picco(A,B,denominatore,type)
[model,model_index]=max(A);
[preonlab,preonlab_index]=max(B);

if denominatore==0
    if type==1
        y=(model-preonlab)*100/preonlab;
    elseif type==2
        y=(A(preonlab_index)-preonlab)*100/preonlab;
    elseif type==3
        y=(model-B(model_index))*100/B(model_index);
    end
else
    y=model-preonlab;
end

end