function P = prodexp(V,th,n)

P = eye(4);
for i = 1:n
    
    P = P*SE3exp(V(:,i),th(i));
      
end



end