import matplotlib.pyplot as plt
from pyomo.environ import *
from pyomo.dae import *
import numpy as np

# lunar module
m_ascent_dry = 2445.0          # kg mass of ascent stage without fuel
m_ascent_fuel = 2376.0         # kg mass of ascent stage fuel
m_descent_dry = 2034.0         # kg mass of descent stage without fuel
m_descent_fuel = 8248.0        # kg mass of descent stage fuel

m_fuel = m_descent_fuel
m_dry = m_ascent_dry + m_ascent_fuel + m_descent_dry
m_total = m_dry + m_fuel

# descent engine characteristics
v_exhaust = 3050.0             # m/s
u_max = 45050.0/v_exhaust      # 45050 newtons / exhaust velocity

# landing mission specifications
h_initial = 100000.0           # meters
v_initial = 1520               # orbital velocity m/s
g = 1.62                       # m/s**2

#t_f = 3000

m = ConcreteModel()
m.t = ContinuousSet(bounds=(0, 1))
m.h = Var(m.t, domain=NonNegativeReals)
m.m = Var(m.t)
m.u = Var(m.t, bounds=(0, u_max))  #比冲
m.T = Var(bounds=(50,3000))  #时间？

m.v = DerivativeVar(m.h, wrt=m.t)
m.a = DerivativeVar(m.v, wrt=m.t)
m.mdot = DerivativeVar(m.m, wrt=m.t)



# 定义推力变量
m.Thrust = Var(m.t)  # 推力，必须是非负的

# 添加约束，表示推力的计算公式
def thrust_constraint(m, t):
    return m.Thrust[t] == m.mdot[t] * m.u[t] * g

m.ThrustConstraint = Constraint(m.t, rule=thrust_constraint)





m.fuel = Integral(m.t, wrt=m.t, rule = lambda m, t: m.u[t]*m.T)
m.obj = Objective(expr=m.fuel, sense=minimize)

m.ode1 = Constraint(m.t, rule = lambda m, t: m.m[t]*m.a[t]/m.T**2 == -m.m[t]*g + v_exhaust*m.u[t])
m.ode2 = Constraint(m.t, rule = lambda m, t: m.mdot[t]/m.T == -m.u[t])

m.h[0].fix(h_initial)
m.v[0].fix(-v_initial)
m.m[0].fix(m_total)

m.h[1].fix(0)    # land on surface
m.v[1].fix(0)    # soft landing

def solve(m):
    TransformationFactory('dae.finite_difference').apply_to(m, nfe=50, scheme='FORWARD')
    SolverFactory('ipopt').solve(m)
    
    m_nonfuel = m_ascent_dry + m_ascent_fuel + m_descent_dry
    
    tsim = [t*m.T() for t in m.t]
    hsim = [m.h[t]() for t in m.t]
    usim = [m.u[t]() for t in m.t]
    fsim = [m.m[t]() - m_nonfuel for t in m.t]

    Thrustsim = [m.ThrustConstraint[t]() for t in m.t]

    plt.subplot(3,1,1)
    plt.plot(tsim, hsim)
    plt.title('altitude')
    plt.ylabel('meters')
    plt.legend(['mission length = ' + str(round(m.T(),1)) + ' seconds'])

    plt.subplot(3,1,2)
    plt.plot(tsim, usim)
    plt.title('engine mass flow')
    plt.ylabel('kg/sec')
    plt.legend(['fuel burned = ' + str(round(m.fuel(),1)) + ' kg'])

    plt.subplot(3,1,3)
    plt.plot(tsim, fsim)
    plt.title('fuel remaining')
    plt.xlabel('time / seconds')
    plt.ylabel('kg')
    plt.legend(['fuel remaining = ' + str(round(fsim[-1],2)) + ' kg'])

    plt.tight_layout()
    plt.show()
    print(Thrustsim)
solve(m)
