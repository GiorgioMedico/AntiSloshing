function f = scalObjFuncAccIndex(alphan_,k_eps,k_sigma,objFuncType)

index = objFuncAccIndex(alphan_);

eps = index(1,:);
sigma = index(2,:);

N = length(eps);

if strcmp(objFuncType,'abs_value')
    f = k_eps*sum(abs(eps))/N + k_sigma*sum(abs(sigma))/N;
elseif strcmp(objFuncType,'wSign_value')
    f = k_eps*sum(eps)/N + k_sigma*sum(sigma)/N;
end

end