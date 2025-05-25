import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Parámetros del robot
m = 50
Iz = 10
r = 0.1
b = 0.3
M = np.diag([m, m, Iz])
L = 1.0  # Longitud del remolque

# Parámetros LuGre
sigma_0 = 100
sigma_1 = 1
sigma_2 = 0.5
Fs = 30
Fc = 18
vs = 0.01

# Parámetros del control PI
Kp_pos = np.array([-0.1, -0.1])  # Ganancia proporcional [x, y]
Ki_pos = np.array([-1000, -1000])  # Ganancia integral [x, y]
x_ref = 5.0  # Posición objetivo en x
y_ref = 3.0  # Posición objetivo en y

# Tiempo de simulación
dt = 0.01
T_total = 20  # Incrementado para dar tiempo al control
t = np.arange(0, T_total + dt, dt)

# Estado inicial
q = np.array([0.0, 0.0, 0.0])  # [x, y, theta]
u = np.array([0.0, 0.0])       # [U, omega]
z_r, z_l = 0.0, 0.0

# Inicialización de errores e integrales para el control PI
e_pos = np.zeros(2)  # Error de posición [ex, ey]
e_pos_int = np.zeros(2)  # Integral del error de posición

# Almacenamiento
Q = np.zeros((3, len(t)))        # Estado del tractor
Q_trailer = np.zeros((2, len(t)))  # Posición del remolque
U = np.zeros((2, len(t)))
E_pos = np.zeros((2, len(t)))    # Error de posición

# Simulación
for k in range(len(t)):
    # Cálculo del error de posición
    e_pos[0] = x_ref - q[0]  # Error en x
    e_pos[1] = y_ref - q[1]  # Error en y
    
    # Actualización de la integral del error
    e_pos_int += e_pos * dt
    
    # Control PI para posición
    # Transformamos el error del marco global al marco local del robot
    theta = q[2]
    R = np.array([
        [np.cos(theta), np.sin(theta)],
        [-np.sin(theta), np.cos(theta)]
    ])
    e_pos_local = R @ e_pos
    e_pos_int_local = R @ e_pos_int
    
    # Cálculo de las acciones de control
    v_cmd = Kp_pos[0] * e_pos_local[0] + Ki_pos[0] * e_pos_int_local[0]
    omega_cmd = Kp_pos[1] * e_pos_local[1] + Ki_pos[1] * e_pos_int_local[1]
    
    # Limitamos las acciones de control
    v_cmd = np.clip(v_cmd, -2.0, 2.0)
    omega_cmd = np.clip(omega_cmd, -1.0, 1.0)
    
    # Convertimos velocidades a torques para los motores
    # Torque derecho e izquierdo basados en v_cmd y omega_cmd
    Tau_r_cmd = (v_cmd + b * omega_cmd) * r
    Tau_l_cmd = (v_cmd - b * omega_cmd) * r
    
    # Aplicamos los torques calculados por el controlador
    Tau_r = Tau_r_cmd
    Tau_l = Tau_l_cmd
    
    theta = q[2]
    N = np.array([[np.cos(theta), 0],
                  [np.sin(theta), 0],
                  [0, 1]])
    q_dot = N @ u
    Uq = np.cos(theta) * q_dot[0] + np.sin(theta) * q_dot[1]
    Vq = -np.sin(theta) * q_dot[0] + np.cos(theta) * q_dot[1]
    omega = u[1]

    v_r = Uq + b * omega
    v_l = Uq - b * omega

    g_r = Fc + (Fs - Fc) * np.exp(-(v_r / vs)**2)
    g_l = Fc + (Fs - Fc) * np.exp(-(v_l / vs)**2)

    dz_r = v_r - (sigma_0 / g_r) * z_r
    dz_l = v_l - (sigma_0 / g_l) * z_l
    z_r += dz_r * dt
    z_l += dz_l * dt

    F_r = sigma_0 * z_r + sigma_1 * dz_r + sigma_2 * v_r
    F_l = sigma_0 * z_l + sigma_1 * dz_l + sigma_2 * v_l

    F_x = F_r + F_l
    M_z = b * (F_r - F_l)
    f_fric = np.array([-F_x * np.cos(theta), -F_x * np.sin(theta), -M_z])
    Cq_dot = np.array([m * Vq * omega, -m * Uq * omega, 0])
    Bq = (1 / r) * np.array([[np.cos(theta), np.cos(theta)],
                             [np.sin(theta), np.sin(theta)],
                             [b, -b]])

    tau = np.array([Tau_r, Tau_l])
    M_u = N.T @ M @ N
    C_u = N.T @ Cq_dot
    f_u = N.T @ f_fric
    B_u = N.T @ Bq

    dot_u = np.linalg.solve(M_u, B_u @ tau - C_u - f_u)
    u += dot_u * dt
    q += q_dot * dt

    # Posición del remolque (arrastrado)
    trailer_x = q[0] - L * np.cos(q[2])
    trailer_y = q[1] - L * np.sin(q[2])

    Q[:, k] = q
    Q_trailer[:, k] = [trailer_x, trailer_y]
    U[:, k] = u
    E_pos[:, k] = e_pos

# Animación
fig, ax = plt.subplots(figsize=(10, 8))
ax.set_aspect('equal')
ax.set_xlim(min(np.min(Q[0, :]) - 2, x_ref - 2), max(np.max(Q[0, :]) + 2, x_ref + 2))
ax.set_ylim(min(np.min(Q[1, :]) - 2, y_ref - 2), max(np.max(Q[1, :]) + 2, y_ref + 2))
tractor_plot, = ax.plot([], [], 'ro', label="Tractor")
trailer_plot, = ax.plot([], [], 'bo', label="Trailer")
link_plot, = ax.plot([], [], 'k--', lw=1)
target_plot, = ax.plot(x_ref, y_ref, 'gx', markersize=10, label="Objetivo")

def init():
    tractor_plot.set_data([], [])
    trailer_plot.set_data([], [])
    link_plot.set_data([], [])
    return tractor_plot, trailer_plot, link_plot, target_plot

def animate(i):
    x_t, y_t = Q[0, i], Q[1, i]
    x_r, y_r = Q_trailer[0, i], Q_trailer[1, i]

    tractor_plot.set_data([x_t], [y_t])
    trailer_plot.set_data([x_r], [y_r])
    link_plot.set_data([x_r, x_t], [y_r, y_t])
    return tractor_plot, trailer_plot, link_plot, target_plot

ani = animation.FuncAnimation(fig, animate, frames=len(t),
                              init_func=init, blit=True, interval=10)
plt.legend()
plt.title("Simulación Tractor con Remolque - Control PI de Posición")
plt.grid(True)

# Gráficas adicionales para visualizar el rendimiento del control
fig2, axs = plt.subplots(2, 1, figsize=(10, 8))
axs[0].plot(t, Q[0, :], 'r-', label='Posición x')
axs[0].plot(t, np.ones_like(t) * x_ref, 'g--', label='Referencia x')
axs[0].set_ylabel('Posición x (m)')
axs[0].legend()
axs[0].grid(True)

axs[1].plot(t, Q[1, :], 'b-', label='Posición y')
axs[1].plot(t, np.ones_like(t) * y_ref, 'g--', label='Referencia y')
axs[1].set_xlabel('Tiempo (s)')
axs[1].set_ylabel('Posición y (m)')
axs[1].legend()
axs[1].grid(True)

plt.tight_layout()
plt.show()
