function f = objFuncAccIndexXW(XW,m1,k1,zita1,h1,J,g,k_,h,R,EOMtype,N,erased_path,Te_array,gammaNL1,str,k_eps,k_sigma,objFuncType)

x = XW(1);
w = XW(2);

f = objFuncAccIndexCropped(x,w,m1,k1,zita1,h1,J,g,k_,h,R,EOMtype,N,erased_path,Te_array,gammaNL1,str,k_eps,k_sigma,objFuncType);

end