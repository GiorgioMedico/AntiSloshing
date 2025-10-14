# 🛠️ CasADi Opti Cheat Sheet

This document summarizes key techniques and best practices for working with CasADi's `Opti` stack for optimization.

---

## 📌 Parameters

```matlab
P = opti.parameter(p_dim, N+1);
opti.set_value(P, 0.1);
opti.solve();
opti.set_value(P, 0.2);
opti.solve();
```
- You can set parameter values multiple times and re-solve.
- After the first solve, subsequent solves are typically faster.
---
## 📏 Scaling
Scaling decision variables helps improve solver convergence.
Multiply each variable (can be done for both state and input variables) by a factor specifying the magnitude of that variable.
Eg. km and m, multiply the first by 0.001 or the latter by 1000 to normalize

**Before**:
```matlab
X = opti.variable(x_dim, N+1);
```
**After (normalized variables)**:
```matlab
scaling_factors = repmat([1; 5; 40; 1; 1; 1; 5; 5; 5; 40; 40; 40], 1, N+1);
X = scaling_factors .* opti.variable(x_dim, N+1);
```
---
## ♻️ Warm Start
Re-using previous solutions as initial guesses speeds up convergence:

```matlab
sol1 = opti.solve();
opti.set_initial(sol1.value_variables());  % Warm start
sol2 = opti.solve();
```
Compare iteration counts:
```matlab
sol1.stats.iter_count
sol2.stats.iter_count
```
---
## 🔍 Debugging
View Initial Guess
```matlab
sol.value(X, opti.initial())         % Initial guess of variable X
sol.value(s_o, opti.initial())       % Initial guess of s
sol.value(x^2 + 10*y, opti.initial())% Expression using initial guess
```
Works even if problem doesn't converge:

```matlab
opti.debug.value(x, opti.initial())
```

### 🧩 Opti.debug
```matlab
opti.debug()
```
Provides diagnostic tools:

- Instance number: number of times solve has been called
- Number of variables:
```matlab
opti.variable % nx = actual num of variables
```
- Number of parameters:
```matlab
opti.parameter % np = actual num of parameters
```
- Number of constraints: 
```matlab
opti.subject_to % nc = actual num of constraints
```

It can show the value of a variable after `opti.solve()` has been called, and it is available even if it fails or the process is stopped
```matlab
opti.debug.value(var)                     % Shows value of variable
```
If problem does not converge you can see which constraint is violated (i=num of constraint / total num of constraints) at which iteration (at nonzero …)
```matlab
opti.debug.show_infeasibilities()         % Violated constraints
opti.debug.show_infeasibilities(1e-5)     % Show those > 1e-5
```
In case the solver reports NaN/Inf at a certain location, you may find out which constraint or variable is to blame by looking at its description: 
```matlab
opti.debug.x_describe(index)             % Describe variable
opti.debug.g_describe(index)             % Describe constraint
```
where index is the number of constraint the compiler tells you is violated
### ⚠️ Note on Square Roots
>
>The derivative of sqrt(x) → ∞ as x → 0. This can cause instability since IPOPT uses gradient information. Be cautious when including square roots in constraints.
>To check if a jacobian is at NaN, eg. if a certain constraint starts at infeasible conditions
>```matlab
>opti.debug.value(jacobian(nonSlip,X),opti.initial())
>```
>
---
## 🔁 Callback
Recursively call a function each iteration, useful to plot evolution of a variable.
Use a callback function to track variable evolution across iterations:

```matlab
figure();
opti.callback(@(i) plot(opti.debug.value(s_o)));
opti.solve();
```
---
Or for 3D plotting
```matlab
figure(1)
opti.callback(@(i) plot3(opti.debug.value(1000*X(1,:)),opti.debug.value(1000*X(2,:)),opti.debug.value(1000*X(3,:))));
axis equal
hold on
```
---
## 📈 Stats & Iterations
```matlab
opti.stats()                   % Overview of stats
opti.stats().iter_count       % Number of iterations
opti.stats().iterations       % Detailed iteration log
```
---
## 🎯 Objective Evolution
You can check the decrease of the objective function 
```matlab
figure();
semilogy(sol.stats.iterations.obj, "LineWidth", 2);
title("Objective")
xlabel("Iteration Number")
ylabel("Objective value")
grid on 
```
---
## 📉 Feasibility
You can check primal and dual feasibility and how they evolve
```matlab
figure;
semilogy(sol.stats.iterations.inf_du, "LineWidth", 2);
hold on;
semilogy(sol.stats.iterations.inf_pr, "LineWidth", 2);
xlabel("Iteration Number");
legend("Dual feasibility", "Primal feasibility");
```
---
### 🧩 Sparsity
You can also check the sparsity of the constraints
```matlab
figure;
spy(sol.value(jacobian(opti.g, opti.x)));
title("Sparsity of the constraint Jacobian");
xlabel("Decision variables");
ylabel("Constraints");
grid on;
```
---
## 🧠 References

- [IPOPT Documentation](https://coin-or.github.io/Ipopt/)
- [Slipping vs. Tipping](https://mechanicsmap.psu.edu/websites/7_friction/7-2_slipping_vs_tipping/slippingvstipping.html)
- [Tipping Point](https://openoregon.pressbooks.pub/bodyphysics2ed/chapter/tipping-point/)
- [Opti Bounds, Constraints and Infeasibilities](https://groups.google.com/g/casadi-users/c/hX6bTw6lCSw)
- [Casadi Wiki](https://github.com/casadi/casadi/wiki/Onboarding-Guide)
- [Casadi NaN Jacobian](https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F)
- [Casadi Docs](https://web.casadi.org/docs/)
- [Casadi Opti Stack Docs](https://web.casadi.org/docs/#document-opti)
- [Casadi Examples](https://web.casadi.org/blog/)

---