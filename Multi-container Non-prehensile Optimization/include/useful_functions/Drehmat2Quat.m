function [ alpha ] = Drehmat2Quat( A )
%#codegen

lim = 1e-10;%1e-15;

% Aufgrund von numerischen Effekten wird separat überprüft, ob der Term
% unter der Wurzel < 0 ist und in diesem Fall auf 0 gesetzt

%% skalarer Anteil
tmp = A(1, 1) + A(2, 2) + A(3, 3) + 1;
if(tmp <= 0)
    eta = 0;
else
    eta = 1/2*sqrt(tmp);
end

%% vektorieller Anteil

% Betrag
eps = zeros(3, 1);
tmp = A(1, 1) - A(2, 2) - A(3, 3) + 1;

if(tmp <= 0)
    eps(1) = 0;
else
    if(A(3, 2) - A(2, 3) >= 0)
        eps(1) = 1/2*sqrt(tmp);
    else
        eps(1) = -1/2*sqrt(tmp);
    end
end
tmp = -A(1, 1) + A(2, 2) - A(3, 3) + 1;
if(tmp <= 0)
    eps(2) = 0;
else
    if(A(1, 3) - A(3, 1) >= 0)
        eps(2) = 1/2*sqrt(tmp);
    else
        eps(2) = -1/2*sqrt(tmp);
    end
end
tmp = -A(1, 1) - A(2, 2) + A(3, 3) + 1;
if(tmp <= 0)
    eps(3) = 0;
else
    if(A(2, 1) - A(1, 2) >= 0)
        eps(3) = 1/2*sqrt(tmp);
    else
        eps(3) = -1/2*sqrt(tmp);
    end
end

% Wenn bestimmte Nebendiagonalterme 0 ergeben, dann sind die Vorzeichen von
% q2, q3 und q4 mit obigen Formeln nicht definiert!

if(A(3, 2) - A(2, 3) < lim || A(1, 3) - A(3, 1) < lim || A(2, 1) - A(1, 2) < lim)
    vecL = ones(1, 3);
    vecR = ones(3, 1);
    minRes = 1e10;
    indRes = 0;
    % Bestimmung der korrekten Vorzeichen
    lst = [ 1,  1,  1;
           -1,  1,  1;
           -1, -1,  1;
           -1,  1, -1;
           -1, -1, -1;
            1, -1,  1;
            1,  1, -1;
            1, -1, -1];
    for i = 1:length(lst)
        % Kontrolle von allen möglichen Kombinationen
        res = vecL*(abs(Quat2Drehmat([eta; eps.*(lst(i, :).')]) - A))*vecR;
        if(res < minRes)
            minRes = res;
            indRes = i;
        end
    end
    % correct signs are determined
    eps = eps.*(lst(indRes, :).');
    
end



alpha = [eta; eps];

end

