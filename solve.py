from amplpy import AMPL
import math
# Initialize AMPL instance
ampl = AMPL()

# Params
N = 5
mi = 250000  # initial mass
md = 150000
tf_min = 0.2*3*2.26*10**6  # min thrust (newton)
tf_max = 1.0*3*2.26*10**6  # max thrust (newton)
phi_max = math.radians(15)  # max thrust gimbal angle
th_max = math.radians(120)  # max thrust gimbal angle
L = 50.3  # length of body
M = 10**15  # big number go brrr
c = 1 / (3700)  # mass flow rate per newton
d = 110  # drag coefficient
# catch bounds
pos = 0.5
angle = math.radians(10)
vel = 0.5
rate = math.radians(10)
# Tuning variables
Qx = 1
Qy = 1
Qth = 1000
Qxd = 1
Qyd = 1
Qthd = 100
Rtfa = 0.0002
Qm = -0.006
QTR = 1500
QLB = 1500
# Initial conditions
x0 = 200
y0 = 1500
xd0 = 0
yd0 = -150
th0 = 0 # math.radians(90) TODO
thd0 = 0

# Define the optimization model directly in Python
ampl.eval(f"set T := 1..{N};")
ampl.eval('''
    # Params
    param mi;
    param md;
    param tf_min;
    param tf_max;
    param th_max;
    param phi_max;
    param L;
    param N;
    param M;
    param c;
    param d;
    param dt {T};
    param Qx;
    param Qy;
    param Qth;
    param Qxd;
    param Qyd;
    param Qthd;
    param Rtfa;
    param Qm;
    param QTR;
    param QLB;
    param pos;
    param angle;
    param vel;
    param rate;
    param x0;
    param y0;
    param th0;
    param xd0;
    param yd0;
    param thd0;
''')
ampl.eval('''
    # Decision variables
    var LB {T} binary;
    var TR {T} binary;
    var x {T} >= 0;
    var y {T} >= 0;
    var th {T} >= -th_max, <= th_max;
    var xd {T};
    var yd {T} <= 0;
    var thd {T};
    var tf {T} >= tf_min, <= tf_max;
    var phi {T} >= -phi_max, <= phi_max;
    var m {T} >= md;
    var tfa {T};
''')
ampl.eval('''
    # Objective function
    minimize obj:
        Qx * sum {t in T} x[t]^2
      + Qy * sum {t in T} y[t]^2
      + Qth * sum {t in T} th[t]^2
      + Qxd * sum {t in T} xd[t]^2
      + Qyd * sum {t in T} yd[t]^2
      + Qthd * sum {t in T} thd[t]^2
      + Qm * sum {t in T} m[t]^2
      + QTR * sum {t in T} TR[t]^2
      + QLB * sum {t in T} LB[t]^2
      + Rtfa * sum {t in T} tfa[t]^2;
''')
ampl.eval('''
    # Constraints
    # subject to thrust_constraint {t in T}:
    #     tfa[t] = tf[t] * LB[t] * TR[t];
    # subject to position_catch_constraint {t in T}:
    #     x[t]^2 + y[t]^2 <= pos^2 + M * TR[t];
    # subject to angle_catch_constraint {t in T}:
    #     th[t]^2 <= angle^2 + M * TR[t];
    # subject to velocity_catch_constraint {t in T}:
    #     xd[t]^2 + yd[t]^2 <= vel^2 + M * TR[t];
    # subject to rate_catch_constraint {t in T}:
    #     thd[t]^2 <= rate^2 + M * TR[t];
    # subject to landing_shutoff_constraint {t in 1..N-1}:
    #     LB[t + 1] >= LB[t];
    # subject to caught_constraint {t in 1..N-1}:
    #     TR[t + 1] <= TR[t];
    # subject to mass_model_constraint {t in 1..N-1}:
    #     m[t + 1] = m[t] - c*dt[t]*tfa[t];
    subject to xpos_model_constraint {t in 1..N-1}:
        x[t + 1] = TR[t]*(x[t] + dt[t]*xd[t]);
    subject to ypos_model_constraint {t in 1..N-1}:
        y[t + 1] = TR[t]*(y[t] + dt[t]*yd[t]);
    subject to th_model_constraint {t in 1..N-1}:
        th[t + 1] = TR[t]*(th[t] + dt[t]*thd[t]);
    # subject to xvel_model_constraint {t in 1..N-1}:
    #     xd[t + 1] = TR[t]*(xd[t] + dt[t]*sin(th[t]+phi[t])*tfa[t]/m[t]);
    # subject to yvel_model_constraint {t in 1..N-1}:
    #     yd[t + 1] = TR[t]*(yd[t] + dt[t]*(cos(th[t]+phi[t])*tfa[t]/m[t] - 9.8 - d*y[t]^2/m[t]));
    # subject to rate_model_constraint {t in 1..N-1}:
    #     thd[t + 1] = TR[t]*(thd[t] + dt[t]*4*tfa[t]*sin(phi[t])/(L*m[t]));
    subject to initial_x_constraint:
        x[1] = x0;
    subject to initial_y_constraint:
        y[1] = y0;
    subject to initial_th_constraint:
        th[1] = th0;
    subject to initial_xd_constraint:
        xd[1] = xd0;
    subject to initial_yd_constraint:
        yd[1] = yd0;
    subject to initial_thd_constraint:
        thd[1] = thd0;
    subject to initial_mass_constraint:
        m[1] = mi;
    # subject to initial_tr_constraint:
    #     TR[1] = 1;
    subject to initial_tr_constraint {t in 1..N}:
        TR[t] = 1;
    subject to initial_lb_constraint:
        LB[1] = 1;
''')
# Set params
ampl.getParameter("mi").set(mi)
ampl.getParameter("md").set(md)
ampl.getParameter("tf_min").set(tf_min)
ampl.getParameter("tf_max").set(tf_max)
ampl.getParameter("th_max").set(th_max)
ampl.getParameter("phi_max").set(phi_max)
ampl.getParameter("N").set(N)
ampl.getParameter("L").set(L)
ampl.getParameter("M").set(M)
ampl.getParameter("c").set(c)
ampl.getParameter("d").set(d)
ampl.getParameter("Qx").set(Qx)
ampl.getParameter("Qy").set(Qy)
ampl.getParameter("Qth").set(Qth)
ampl.getParameter("Qxd").set(Qxd)
ampl.getParameter("Qyd").set(Qyd)
ampl.getParameter("Qthd").set(Qthd)
ampl.getParameter("Qm").set(Qm)
ampl.getParameter("QTR").set(QTR)
ampl.getParameter("QLB").set(QLB)
ampl.getParameter("Rtfa").set(Rtfa)
ampl.getParameter("pos").set(pos)
ampl.getParameter("angle").set(angle)
ampl.getParameter("vel").set(vel)
ampl.getParameter("rate").set(rate)
ampl.getParameter("x0").set(x0)
ampl.getParameter("y0").set(y0)
ampl.getParameter("th0").set(th0)
ampl.getParameter("xd0").set(xd0)
ampl.getParameter("yd0").set(yd0)
ampl.getParameter("thd0").set(thd0)

dt_values = [0.1 * math.exp(i / 5) for i in range(N)]
ampl.getParameter("dt").setValues({i+1: dt_values[i] for i in range(N)})

# Set the solver to COUENNE
ampl.setOption('solver', 'couenne')

# Set initial conditions
ampl.getVariable('x').setValues({i+1: x0 for i in range(N)})
ampl.getVariable('y').setValues({i+1: y0 for i in range(N)})
ampl.getVariable('th').setValues({i+1: th0 for i in range(N)})
ampl.getVariable('xd').setValues({i+1: xd0 for i in range(N)})
ampl.getVariable('yd').setValues({i+1: yd0 for i in range(N)})
ampl.getVariable('thd').setValues({i+1: thd0 for i in range(N)})
ampl.getVariable('TR').setValues({i+1: 1 for i in range(N)})

# Solve the problem
ampl.solve()

# Display the results
x = ampl.getVariable('x').getValues().toList()
y = ampl.getVariable('y').getValues().toList()
th = ampl.getVariable('th').getValues().toList()
xd = ampl.getVariable('xd').getValues().toList()
yd = ampl.getVariable('yd').getValues().toList()
thd = ampl.getVariable('thd').getValues().toList()
TR = ampl.getVariable('TR').getValues().toList()
LB = ampl.getVariable('TR').getValues().toList()
tf = ampl.getVariable('tf').getValues().toList()
phi = ampl.getVariable('phi').getValues().toList()
m = ampl.getVariable('m').getValues().toList()
tfa = ampl.getVariable('tfa').getValues().toList()


def display():
    for i in range(N):
        print(f"Timestep: {i+1}:")
        print(f"\tx, y, th: {x[i][1]}, {y[i][1]}, {th[i][1]}")
        print(f"\txd, yd, thd: {xd[i][1]}, {yd[i][1]}, {thd[i][1]}")
        print(f"\tTR, LB, m: {TR[i][1]}, {LB[i][1]}, {m[i][1]}")
        print(f"\ttf, fta, phi: {tf[i][1]}, {tfa[i][1]}, {phi[i][1]}")


display()
