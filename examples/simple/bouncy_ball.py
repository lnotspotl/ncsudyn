import casadi as ca
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Constants
e = 0.8  # coefficient of restitution (0 ≤ e ≤ 1)
g = 9.8  # gravitational constant (m/s^2)
x0, xf = 0, 4
z0, zf = 1, 1

fig, ax = plt.subplots(figsize=(10, 4))
ax.set_xlabel("x")
ax.set_ylabel("z")
ax.set_xlim(x0 - 0.1, xf + 0.1)

def bounce_pass(number_of_bounces=0, zdot0_is_positive=True, debug=True):
    duration = 2 + number_of_bounces
    xdot = (xf - x0) / duration
    
    # Create CasADi optimization problem
    opti = ca.Opti()
    
    # Decision variables
    h = opti.variable(number_of_bounces + 1)  # time intervals
    zdot = opti.variable(number_of_bounces + 1)  # vertical velocities after collision
    
    # Constraints
    opti.subject_to(ca.sum1(h) == duration)
    opti.subject_to(h >= 0)
    
    # Initial velocity constraint
    if zdot0_is_positive:
        opti.subject_to(zdot[0] >= 0)
    else:
        opti.subject_to(zdot[0] <= 0)
    
    # Add dynamics constraints for each segment that ends with a bounce
    for i in range(number_of_bounces):
        # z must be zero at the end of this segment
        z = zdot[i] * h[i] - 0.5 * g * h[i] * h[i] + (z0 if i == 0 else 0)
        opti.subject_to(z == 0)
        opti.subject_to(zdot[i + 1] == -e * (zdot[i] - h[i] * g))
    
    # Final segment constraint
    z = zdot[-1] * h[-1] - 0.5 * g * h[-1] * h[-1] + (z0 if number_of_bounces == 0 else 0)
    opti.subject_to(z == zf)
    
    # Solve the optimization problem
    opts = {'ipopt.print_level': 0, 'print_time': 0}
    opti.solver('ipopt', opts)
    
    try:
        sol = opti.solve()
        h_sol = sol.value(h)
        zdot_sol = sol.value(zdot)
        
        # Plot the resulting trajectory
        ax.set_prop_cycle(plt.rcParams["axes.prop_cycle"])
        relative_time = np.linspace(0, 1, 10)
        x_start = x0
        
        for i in range(number_of_bounces + 1):
            t = h_sol[i] * relative_time
            x = x_start + t * xdot
            z = (z0 if i == 0 else 0) + zdot_sol[i] * t - 0.5 * g * t * t
            ax.plot(x, z, ".-")
            x_start = x[-1]
            
    except:
        if debug:
            print("Failed to find solution")
        return

# Generate solutions for different numbers of bounces
for number_of_bounces in range(0, 10):
    print(f"Number of bounces: {number_of_bounces}")
    bounce_pass(number_of_bounces=number_of_bounces, zdot0_is_positive=True)
    bounce_pass(number_of_bounces=number_of_bounces, zdot0_is_positive=False)

# Plot the solutions
plt.show()