function xkplus1 = rk4_singlestep(f,dt,tk,xk)

f1 = f(tk,xk);
f2 = f(tk+dt/2,xk+(dt/2)*f1);
f3 = f(tk+dt/2,xk+(dt/2)*f2);
f4 = f(tk+dt,xk+dt*f3);

xkplus1 = xk + (dt/6)*(f1 + 2*f2+ 2*f3 + f4);

end