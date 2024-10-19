from amplpy import AMPL
import xarray as xr
from matplotlib import pyplot as plt
import math
# Initialize AMPL instance
ampl = AMPL()


class ProblemState:
    def __init__(self, problem_state=None):
        self.LB = []
        self.TR = []
        self.x = []
        self.y = []
        self.th = []
        self.xd = []
        self.yd = []
        self.thd = []
        self.tf = []
        self.phi = []
        self.m = []
        self.tfa = []
        # Initial conditions
        if problem_state is None:
            self.x0 = 850
            self.y0 = 2000
            self.th0 = math.radians(90)
            self.xd0 = 0
            self.yd0 = -200
            self.thd0 = 0
            self.m0 = 250000
            self.TR0 = 1
            self.LB0 = 0
            self.phi0 = 0
            self.tf0 = tf_max
        else:
            self.x0 = problem_state.x[1]
            self.y0 = problem_state.y[1]
            self.th0 = problem_state.th[1]
            self.xd0 = problem_state.xd[1]
            self.yd0 = problem_state.yd[1]
            self.thd0 = problem_state.thd[1]
            self.m0 = problem_state.m[1]
            self.TR0 = problem_state.TR[1]
            self.LB0 = problem_state.LB[1]
            self.phi0 = problem_state.phi[1]
            self.tf0 = problem_state.tf[1]

    def solve(self, ampl):
        # Set initial conditions
        ampl.getParameter("x0").set(self.x0)
        ampl.getParameter("y0").set(self.y0)
        ampl.getParameter("th0").set(self.th0)
        ampl.getParameter("xd0").set(self.xd0)
        ampl.getParameter("yd0").set(self.yd0)
        ampl.getParameter("thd0").set(self.thd0)
        ampl.getParameter("phi0").set(self.phi0)
        ampl.getParameter("tf0").set(self.tf0)
        ampl.getParameter("m0").set(self.m0)
        ampl.getParameter("TR0").set(self.TR0)
        ampl.getParameter("LB0").set(self.LB0)

        # Solve the problem
        ampl.solve()
        x = ampl.getVariable('x').getValues().toList()
        x = ampl.getVariable('x').getValues().toList()
        y = ampl.getVariable('y').getValues().toList()
        y = ampl.getVariable('y').getValues().toList()
        th = ampl.getVariable('th').getValues().toList()
        th = ampl.getVariable('th').getValues().toList()
        xd = ampl.getVariable('xd').getValues().toList()
        xd = ampl.getVariable('xd').getValues().toList()
        yd = ampl.getVariable('yd').getValues().toList()
        yd = ampl.getVariable('yd').getValues().toList()
        thd = ampl.getVariable('thd').getValues().toList()
        thd = ampl.getVariable('thd').getValues().toList()
        TR = ampl.getVariable('TR').getValues().toList()
        TR = ampl.getVariable('TR').getValues().toList()
        LB = ampl.getVariable('LB').getValues().toList()
        LB = ampl.getVariable('LB').getValues().toList()
        tf = ampl.getVariable('tf').getValues().toList()
        tf = ampl.getVariable('tf').getValues().toList()
        phi = ampl.getVariable('phi').getValues().toList()
        phi = ampl.getVariable('phi').getValues().toList()
        m = ampl.getVariable('m').getValues().toList()
        m = ampl.getVariable('m').getValues().toList()
        tfa = ampl.getVariable('tfa').getValues().toList()
        tfa = ampl.getVariable('tfa').getValues().toList()
        self.x = [val[1] for val in x]
        self.y = [val[1] for val in y]
        self.th = [val[1] for val in th]
        self.xd = [val[1] for val in xd]
        self.yd = [val[1] for val in yd]
        self.thd = [val[1] for val in thd]
        self.TR = [val[1] for val in TR]
        self.LB = [val[1] for val in LB]
        self.tf = [val[1] for val in tf]
        self.phi = [val[1] for val in phi]
        self.m = [val[1] for val in m]
        self.tfa = [val[1] for val in tfa]

    def __str__(self):
        string = []
        time = 0
        for i in range(N):
            string.append(f"Timestep: {i+1}: t={time}\n")
            string.append(f"\tx, y, th: {self.x[i]}, {self.y[i]}, {self.th[i]}\n")
            string.append(f"\txd, yd, thd: {self.xd[i]}, {self.yd[i]}, {self.thd[i]}\n")
            string.append(f"\tTR, LB, m: {self.TR[i]}, {self.LB[i]}, {self.m[i]}\n")
            string.append(f"\ttf, tfa, phi: {self.tf[i]}, {self.tfa[i]}, {self.phi [i]}\n\n")
            time += dt_values[i]
        string.append("Done\n")
        return "".join(string)

    def plot(self):
        da = xr.DataArray(self.y, dims=('x',), coords={'x': self.x, 'LB': ('x', self.LB), 'TR': ('x', self.TR)})
        fig, ax = plt.subplots()
        da.plot(ax=ax, color='blue', linewidth=3)
        da.where(da.LB == 1).plot(ax=ax, color='red', linewidth=3)
        da.where(da.TR == 0).plot(ax=ax, color='green', linewidth=3)
        ax.set_aspect('equal')
        plt.show()


# Params
N = 10
md = 150000
tf_min = 0.4*3*2.26*10**6  # min thrust (newton)
tf_max = 1.0*3*2.26*10**6  # max thrust (newton)
phi_max = math.radians(15)  # max thrust gimbal angle
th_max = math.radians(120)  # max thrust gimbal angle
L = 50.3  # length of body
M = 10**9  # big number go brrr
c = 1 / (3700)  # mass flow rate per newton
d = 200  # drag coefficient
# catch bounds
pos = 0.5
angle = math.radians(10)
vel = 0.5
rate = math.radians(10)
# Tuning variables
Qx = 1
Qy = 1
Qth = 1000**2
Qxd = 1
Qyd = 1
Qthd = 100**2
Rtfa = 0.5*0.0002**2
Qm = -0.006**2
QTR = 10*1500**2
QLB = 1500**2
# Rate decay const
dt0 = 0.1
dt_decay = 0.7

# Define the optimization model directly in Python
ampl.eval(f"set T := 1..{N};")
ampl.eval('''
    # Params
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
    param phi0;
    param tf0;
    param m0;
    param TR0;
    param LB0;

    # Decision variables
    var LB {T} binary;
    var TR {T} binary;
    var x {T};
    var y {T};
    var th {T} >= -th_max, <= th_max;
    var xd {T};
    var yd {T};
    var thd {T};
    var tf {T} >= tf_min, <= tf_max;
    var phi {T} >= -phi_max, <= phi_max;
    var m {T} >= md;
    var tfa {T};

    # Objective function
    minimize obj:
        Qx * sum {t in T} dt[t]*x[t]^2
      + Qy * sum {t in T} dt[t]*y[t]^2
      + Qth * sum {t in T} dt[t]*th[t]^2
      + Qxd * sum {t in T} dt[t]*xd[t]^2
      + Qyd * sum {t in T} dt[t]*yd[t]^2
      + Qthd * sum {t in T} dt[t]*thd[t]^2
      + QTR * sum {t in T} dt[t]*TR[t]^2
      + Qm * sum {t in T} dt[t]*m[t]^2;

    # Constraints
    subject to thrust_constraint {t in T}:
        tfa[t] = tf[t] * LB[t] * TR[t];
    subject to position_catch_constraint {t in T}:
        x[t]^2 + y[t]^2 + th[t]^2 + xd[t]^2 + yd[t]^2 + thd[t]^2 <= 1 + M * TR[t];
    subject to landing_shutoff_constraint {t in 1..N-1}:
        LB[t + 1] >= LB[t];
    subject to mass_model_constraint {t in 1..N-1}:
        m[t + 1] = m[t] - c*dt[t]*tfa[t];
    subject to xpos_model_constraint {t in 1..N-1}:
        x[t + 1] = TR[t]*(x[t] + dt[t]*xd[t]);
    subject to ypos_model_constraint {t in 1..N-1}:
        y[t + 1] = TR[t]*(y[t] + dt[t]*yd[t]);
    subject to th_model_constraint {t in 1..N-1}:
        th[t + 1] = TR[t]*(th[t] + dt[t]*thd[t]);
    subject to xvel_model_constraint {t in 1..N-1}:
        xd[t + 1] = TR[t]*(xd[t] - dt[t]*sin(th[t]+phi[t])*tfa[t]/m[t]);
    subject to yvel_model_constraint {t in 1..N-1}:
        yd[t + 1] = TR[t]*(yd[t] + dt[t]*(cos(th[t]+phi[t])*tfa[t]/m[t] - 9.8 + d*yd[t]^2/m[t]));
    subject to rate_model_constraint {t in 1..N-1}:
        thd[t + 1] = TR[t]*(thd[t] + dt[t]*4*tfa[t]*sin(phi[t])/(L*m[t]));
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
    subject to initial_phi_constraint:
        phi[1] = phi0;
    subject to initial_tf_constraint:
        tf[1] = tf0;
    subject to initial_mass_constraint:
        m[1] = m0;
    subject to initial_tr_constraint:
        TR[1] = TR0;
    subject to initial_turn_constraint:
        LB[1] = LB0;
''')
# Set params
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

dt_values = [dt0 * math.exp(dt_decay*i) for i in range(N)]
ampl.getParameter("dt").setValues({i+1: dt_values[i] for i in range(N)})

# Set the solver to COUENNE
ampl.setOption('solver', 'bonmin')

# Display the results
prob = ProblemState()
prob.solve(ampl)
print(prob)
prob.plot()
