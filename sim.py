import numpy as np
import math
from matplotlib import pyplot as plt, patches
from dataclasses import dataclass
from matplotlib.patches import Polygon
import matplotlib
import matplotlib.animation as animation
import matplotlib.patches as patches

pi = np.pi


# Boat geometry parameters
@dataclass
class BoatGP:
    T: float             # Maximal thrust force [N]
    Lt: float            # Arm of thrust origin [m]
    J: float             # moment of inertia [kg m2]
    m: float             # mass [kg]
    L: float             # boat length;
    frontAreaAir: float    # front cross section area of upper (air) structure [m2]
    frontAreaWater: float  # front cross section area of under (water) structure [m2]

## material properties
@dataclass
class MatParameter:
    rhoWater: float  # water density [kg/m3]
    rhoAir: float    # air density [kg/m3]

# Relative speed and direction
@dataclass
class Relative:
    v: float   # speed
    phi: float # direction

# External forces parameters
@dataclass
class External:
    F: list[float]   # Force
    My: float  # Yaw Moment
    Mr: float  # Rotational Moment

# Aerodynamic coefficients
@dataclass
class Coefficient:
    Cd: float    # drag coefficient
    Cs: float    # side coefficient
    Cy: float    # yaw coefficient
    Cs_R: float  # side rotation coefficient
    A: float     # cross section area
    rho: float   # fluid density

# Boat dynamic data at time t
@dataclass
class BoatDD:
    p:    list[float]     # position in Earth frame (EREC)
    pWgs: list[float]     # position in Earth frame (WGS)
    v:    list[float]     # velocity in Earth frame
    vR:   list[float]     # velocity in Boat frame
    a:    list[float]     # acceleration in Earth frame
    aR:   list[float]     # acceleration in Boat frame
    phi:  float           # true course
    omega: float          # angular velocity
    alpha: float          # angular acceleration
    t: float              # time
    fin:   list[float]    # final docking position

    propulsion: External      # propulsion force, moment
    airResistance: External   # air resistance
    waterResistance: External # water resistance

@dataclass
class Boat:
    m_Propulsion: External
    m_WaterResistance: External

    m_Pos: list[float]     ## (x,y) position
    m_wgsPos: list[float]  ## (LAT,LONG) position
    m_RelVel: list[float]  ## velocity in boat cc
    m_Vel: list[float]     ## velocity in fixed cc
    m_Acc: list[float]     ## acceleration in fixed cc
    m_RelAcc: list[float]  ## acceleration in boat cc
    m_Phi: float           ## true course
    m_Omega: float         ## angular velocity
    m_Alpha: float         ## angular acceleration
    m_t: float             ## time
    m_tDamp: float         ## characteristic damping time

@dataclass
class Flow:
    m_Vel: float
    m_Direction: float



### functions to use
# relative velocity calculator
def setRelative(shipRelVel, shipCourse, Relative):
    '''
    sets the relative velocites of the extrtnal flows
    list: shipRelVel - velocity in Boat frame [m/s]
    float: shipCourse - true course of boat [rad]
    class: Relative - class of relative boat values

    utilizes other constants:
    float: m_Vel - boat velocity (earth frame)
    float: m_Direction - fluid direction (earth frame)

    rewrites the relative values
    also defines the angle between wind and ship
    '''
    r = Relative
    m_Vel = Flow.m_Vel
    m_Direction = Flow.m_Direction

    if(m_Vel == 0.0):
        vx = -shipRelVel[0]
        vy = -shipRelVel[1]

        r.v = np.sqrt(vx*vx + vy*vy)

        if(r.v == 0):
            r.phi = pi
        else:
            if (abs(vy) == 0.0):
                r.phi = pi
            else:
                r.phi = np.arccos(vx/r.v)
    else:
        # find the angle between the wind and ship
        # w is the enviornmental fluid direction
        w = m_Direction - shipCourse

        if(abs(w) < 1e-8 ):
            w = 0.0
        elif(abs((abs(w) - pi)) < 1e-8 ):
            w = pi
        else:
            if( shipCourse >= 0 and m_Direction >= 0 ):
                w = m_Direction - shipCourse

            elif( shipCourse >= 0 and m_Direction < 0 ):
                if( shipCourse - m_Direction > pi ):
                    w = 2*pi - shipCourse + m_Direction
                else:
                    w = m_Direction - shipCourse

            elif( shipCourse < 0 and m_Direction >= 0 ):
                if( m_Direction - shipCourse > pi ):
                    w = 2*pi - m_Direction + shipCourse
                else:
                    w = shipCourse - m_Direction
            else:
                w = m_Direction - shipCourse

        if( abs(w) < 1e-8 ):
            w = 0.0

        if( abs((abs(w) - pi)) < 1e-8 ):
            w = pi

        vx = m_Vel*np.cos(w) - shipRelVel[0]
        vy = m_Vel*np.sin(w) - shipRelVel[1]

        r.v = np.sqrt(vx*vx + vy*vy)

        if( r.v < 1e-8):
            r.phi = pi
            r.v = 0
        else:
            if (abs(vy) < 1e-8):
                vy = 0.0
                if( vx > 0 ):
                    r.phi = 0.0
                else:
                    r.phi = pi
            else:
                r.phi = np.arccos(vx/r.v)

    if(vy < 0.0 and r.phi > 0):
        r.phi = -r.phi


# math angle calculator function
def getMathAngle(theta):
    '''calculates the usable angle of the boat in radians based on a provided theta angle

    '''
    theta_r = math.fmod(theta, 360.0)

    if (theta_r < 0):
        theta_r = 360 + theta_r

    if( theta_r >= 0 and theta_r <= 90):
        a = 90 - theta_r

    elif( theta_r > 90 and theta_r <= 270):
        a= -theta_r + 90

    elif( theta_r > 270 and theta_r <= 360):
        a = 450 - theta

    return a*pi/180.0



## throttle calculations
def setPropulsion(t, phi):
    '''
    sets the propulsion of the boat, given the rudder angle and throttle
    phi = rudder angle in degrees
    t = throttle from -1,1

    Writes the propulsion to the m_Propulsion member of the External class
    Utilizes the (T) member of the BoatParameter class which is the distance from the center of gravity to the throttle
    '''
    x = phi * np.pi/180

    if(t < 0):
        Boat.m_Propulsion.F[1] = 0.0
        Boat.m_Propulsion.F[0] = gp.T * t
        Boat.m_Propulsion.My = 0.0
        Boat.m_Propulsion.Mr = 0.0
    else:
        Boat.m_Propulsion.F[0] = gp.T * t * np.cos(x)
        Boat.m_Propulsion.F[1] = gp.T * t * np.sin(x)
        Boat.m_Propulsion.My = 0.0
        Boat.m_Propulsion.Mr = -Boat.m_Propulsion.F[1]*gp.Lt


#calculate rotational moments
def getRotationalMoment(a, v):
    '''
    a = c.Cs_R*c.A*c.rho/gp.L
    v = velocity
    m_omega =
    '''
    L = gp.L
    w = abs(Boat.m_Omega)

    Ma = (L*L*a)/192 * (3*L*L*w*w + 16*L*w*v + 24*v*v)

    if( v > w*L/2 ):
        Mb = -(L*L*a)/192 * (3*L*L*w*w - 16*L*w*v + 24*v*v)
    else:
        if(w == 0.0):
            Mb = 0.0
        else:
            Mb = a/(192*w*w) * (pow(L*w - 2*v, 3)*(3*L*w + 2*v) - 16*v*v*v*v)

    return Ma + Mb


#set the external functions
def setExternal(c, r, e):
    '''
    set the wind drag, side force and yaw moment based on fit model
        x: angle in rad [0,pi]
        v: velocity in m/s

    inputs:
        class: Coefficient - c
        class: Relative - r
        class: External - e

    sets the values for F, My, and My for the External class
    '''

    x = abs(r.phi)

    if(x < 0 or x>pi):
        print("angle is out of range")

    e.F[0] = 0.5*c.Cd*c.A*c.rho*r.v*r.v ## Fx
    e.F[1] = 0.5*c.Cs*c.A*c.rho*r.v*r.v ## Fy

    ## Calculate the rotational part of moment
    vy = r.v*np.sin(x)
    a = c.Cs_R*c.A*c.rho/gp.L

    Mr = getRotationalMoment(a, abs(vy))

    if (Boat.m_Omega > 0):
        Mr = -abs(Mr)
    else:
        Mr = abs(Mr)

    ## calculate yaw moment
    My = 0.2*c.Cy*c.A*c.rho*r.v*r.v

    e.My = My
    e.Mr = Mr


def setOdeFunction(dt, x, f):
    '''
    sets the derivative array for the ODE as array f

    inputs:
    initial array: x[x, y, vx, vy, phi, omega]
    derivative array: f[vx, vy, ax, ay, omega, alpha]
    '''
    f[0] = x[2]
    f[1] = x[3]
    f[2] = Boat.m_Acc[0]
    f[3] = Boat.m_Acc[1]
    f[4] = x[5]
    f[5] = Boat.m_Alpha


#equation for the rk2 method to calculate the next state
def rk2_step(dt, x):
    '''
    utilizes a second order rutta-kunga numerical solution to calculate the next state of an input

    state: current state array
    dt: change in time from the current state to the next state

    utilizes setOdeFunction to calculate the
    '''
    k = np.zeros(6)
    f = np.zeros(6)

    setOdeFunction(dt, x, k)
    setOdeFunction(dt, x + 0.5*dt*k, f)

    xNew = x + dt*f
    return(xNew)


def rk4_step(dt, x):
    '''
    utilizes a fourth order rutta-kunga numerical solution to calculate the next state of an input

    state: current state array
    dt: change in time from the current state to the next state

    utilizes setOdeFunction to calculate the
    '''
    k1 = np.zeros(6)
    k2 = np.zeros(6)
    k3 = np.zeros(6)
    k4 = np.zeros(6)

    setOdeFunction(dt, x, k1)
    setOdeFunction(dt, x + 0.5*dt*k1, k2)
    setOdeFunction(dt, x + 0.5*dt*k2, k3)
    setOdeFunction(dt, x + dt*k3, k4)

    xNew = x + dt/6*(k1 + 2*k2 + 2*k3 + k4)
    return(xNew)


#defines the variables for the boat data
def setBoatData(dd):
    '''
    sets the boat data based on the given array (should be the dynamic boat data)
    '''
    #vectors
    Boat.m_Pos = dd.p                # (x,y) position
    Boat.m_wgsPos = dd.pWgs          # (LAT,LONG) position
    Boat.m_RelVel = dd.vR            # velocity in boat cc
    Boat.m_Vel = dd.v                # velocity in fixed cc
    Boat.m_Acc = dd.a                # acceleration in fixed cc

    #floats
    Boat.m_Phi = dd.phi              # true course
    Boat.m_Omega = dd.omega          # angular velocity
    Boat.m_Alpha = dd.alpha          # angular acceleration
    Boat.m_t = dd.t                  # time
    #also floats but weird ones
    #boat.m_Propulsion = None
    #boat.m_WaterResistance = None


def setVelDir(vel, direction):
    '''Set current direction'''
    Flow.m_Direction = getMathAngle(direction)
    Flow.m_Vel = vel

#calculate the next state
def move(dt, array, endtime):
    '''
    calculates the next state based on th
    input:
    dt - change in time
    array: array to be appended

    state array: x, y, vx, vy, phi, omega
    also requires the other states to be initialized
    '''

    # Set air and water relative parameters
    setRelative(Boat.m_RelVel, Boat.m_Phi, rW)

    # Set water coefficients
    setWaterCoefficients(cW, rW.phi)
    cW.A = gp.frontAreaWater
    cW.rho = mp.rhoWater

    # Set the externals in m_WaterResistance
    Boat.m_WaterResistance = External([0,0],0,0)
    setExternal(cW, rW, Boat.m_WaterResistance);

    #calculate the force on the boat from propulsion and water resistance
    Fe = np.array(Boat.m_Propulsion.F) + Boat.m_WaterResistance.F
    MeY = np.array(Boat.m_Propulsion.My) + Boat.m_WaterResistance.My
    MeR = np.array(Boat.m_Propulsion.Mr) + Boat.m_WaterResistance.Mr

    #create universal and relative direction arrays
    R = np.zeros([2, 2])
    rR = np.zeros([2, 2])

    R[0,0] = np.cos(Boat.m_Phi)
    R[0,1] = -np.sin(Boat.m_Phi)
    R[1,0] = np.sin(Boat.m_Phi)
    R[1,1] = np.cos(Boat.m_Phi)
    rR[0,0] = np.cos(-Boat.m_Phi)
    rR[0,1] = -np.sin(-Boat.m_Phi)
    rR[1,0] = np.sin(-Boat.m_Phi)
    rR[1,1] = np.cos(-Boat.m_Phi)

    # Find dynamics
    Boat.m_RelAcc = Fe/gp.m                # relative acceleration
    Boat.m_Acc = np.dot(R, Boat.m_RelAcc)  # acceleration in fixed frame
    Boat.m_Alpha = (MeY + MeR)/gp.J        # angular acceleration

    #damp oscillation if rudder angle is zero. Sets m_tDamp
    #only activates if True
    if True:
        if (Boat.m_Propulsion.Mr == 0.0):
            if (Boat.m_tDamp == 0.0):
                Boat.m_tDamp = Boat.m_t
            gda = 5e-2
            gdo = 1e-2
            Boat.m_Alpha = Boat.m_Alpha*np.exp(-gda*(Boat.m_t - Boat.m_tDamp))
            Boat.m_Omega = Boat.m_Omega*np.exp(-gdo*(Boat.m_t - Boat.m_tDamp))
        else:
            Boat.m_tDamp = 0.0

    state = np.zeros(6)
    #define current states
    state[0] = Boat.m_Pos[0]
    state[1] = Boat.m_Pos[1]
    state[2] = Boat.m_Vel[0]
    state[3] = Boat.m_Vel[1]
    state[4] = Boat.m_Phi
    state[5] = Boat.m_Omega

    #set the derivative array
    #New = np.zeros(6)

    #calculate the next state. This can be switched to rk4 method
    New = rk4_step(dt, state)

    Boat.m_Pos[0] = New[0]
    Boat.m_Pos[1] = New[1]
    Boat.m_Vel[0] = New[2]
    Boat.m_Vel[1] = New[3]

    if (New[4] > pi):
        Boat.m_Phi = New[4] - 2*pi
    else:
        Boat.m_Phi = New[4]

    Boat.m_Omega = New[5]
    Boat.m_RelVel = np.dot(rR, Boat.m_Vel)
    dd.t += dt

    global running

    if dd.t > endtime:
        running = False
        print('Simulation complete')
    else:
        running = True
    #array = np.vstack([array, New])



def step(dt, action):
    '''
    calculates the next state based on th
    input:
    dt - change in time
    action: sets the propulsion and the like

    state array: x, y, vx, vy, phi, omega, x_final, y_final
    these last two should not change
    also requires the other states to be initialized
    '''
    #set propulsion using actions
    setPropulsion(action[0],action[1])

    # Set air and water relative parameters
    setRelative(Boat.m_RelVel, Boat.m_Phi, rW)

    # Set water coefficients
    setWaterCoefficients(cW, rW.phi)
    cW.A = gp.frontAreaWater
    cW.rho = mp.rhoWater

    # Set the externals in m_WaterResistance
    Boat.m_WaterResistance = External([0,0],0,0)
    setExternal(cW, rW, Boat.m_WaterResistance);

    #calculate the force on the boat from propulsion and water resistance
    Fe = np.array(Boat.m_Propulsion.F) + Boat.m_WaterResistance.F
    MeY = np.array(Boat.m_Propulsion.My) + Boat.m_WaterResistance.My
    MeR = np.array(Boat.m_Propulsion.Mr) + Boat.m_WaterResistance.Mr

    #create universal and relative direction arrays
    R = np.zeros([2, 2])
    rR = np.zeros([2, 2])

    R[0,0] = np.cos(Boat.m_Phi)
    R[0,1] = -np.sin(Boat.m_Phi)
    R[1,0] = np.sin(Boat.m_Phi)
    R[1,1] = np.cos(Boat.m_Phi)
    rR[0,0] = np.cos(-Boat.m_Phi)
    rR[0,1] = -np.sin(-Boat.m_Phi)
    rR[1,0] = np.sin(-Boat.m_Phi)
    rR[1,1] = np.cos(-Boat.m_Phi)

    # Find dynamics
    Boat.m_RelAcc = Fe/gp.m                # relative acceleration
    Boat.m_Acc = np.dot(R, Boat.m_RelAcc)  # acceleration in fixed frame
    Boat.m_Alpha = (MeY + MeR)/gp.J        # angular acceleration

    #damp oscillation if rudder angle is zero. Sets m_tDamp
    #only activates if True
    if True:
        if (Boat.m_Propulsion.Mr == 0.0):
            if (Boat.m_tDamp == 0.0):
                Boat.m_tDamp = Boat.m_t
            gda = 5e-2
            gdo = 1e-2
            Boat.m_Alpha = Boat.m_Alpha*np.exp(-gda*(Boat.m_t - Boat.m_tDamp))
            Boat.m_Omega = Boat.m_Omega*np.exp(-gdo*(Boat.m_t - Boat.m_tDamp))
        else:
            Boat.m_tDamp = 0.0

    state = np.zeros(6)
    #define current states
    state[0] = Boat.m_Pos[0]
    state[1] = Boat.m_Pos[1]
    state[2] = Boat.m_Vel[0]
    state[3] = Boat.m_Vel[1]
    state[4] = Boat.m_Phi
    state[5] = Boat.m_Omega

    #calculate the next state. This can be switched to rk4 method
    New = rk4_step(dt, state)

    Boat.m_Pos[0] = New[0]
    Boat.m_Pos[1] = New[1]
    Boat.m_Vel[0] = New[2]
    Boat.m_Vel[1] = New[3]

    if (New[4] > pi):
        Boat.m_Phi = New[4] - 2*pi
    else:
        Boat.m_Phi = New[4]

    Boat.m_Omega = New[5]
    Boat.m_RelVel = np.dot(rR, Boat.m_Vel)
    dd.t += dt

    next_state = np.array([Boat.m_Pos[0],
                           Boat.m_Pos[1],
                           Boat.m_Vel[0],
                           Boat.m_Vel[1],
                           Boat.m_Phi,
                           Boat.m_Omega,
                           dd.t])
    
            
    return next_state


def initialize_boat(initial_state, final_pos):
    '''
    Initializes the boat conditions.
    '''
    #initialize the global constants - material and boat properties
    global gp, mp
    gp = BoatGP(0, 0, 0, 0, 0, 0, 0)
    mp = MatParameter(0, 0)
    #set the constants to the predefined conditions
    setGeoParameter(gp)
    setMatParameter(mp)


    #initialize the dynamic boat conditions
    global prop, airR, waterR, dd
    prop = External(0, 0, 0)
    airR = External(0, 0, 0)
    waterR = External(0, 0, 0)
    dd = BoatDD([0,0], [0,0], [0,0], [0,0], [0,0], [0,0],
                0, 0, 0, 0, [0,0],
                prop, airR, waterR)
    #set the initial dynamic conditions
    setInital(dd)
             
    #Set final position
    dd.fin[0] = final_pos[0]
    dd.fin[1] = final_pos[1]
               
    #initialize the other boat conditions. I dont remember why I have two boat dataclasses but I do so just use this
    Boat.m_Propulsion = External([0,0],0,0)
    Boat.m_WaterResistance = External([0,0],0,0)
    Boat.m_tDamp = 0.0
               
    #set the boat data. again I dont remember why but i need it for some reason
    setBoatData(dd)

    #initialize the coefficients for friction in the water
    global cW, rW
    cW = Coefficient(0, 0, 0, 0, 0, 0)
    rW = Relative(0, 0)
    
    #define current states
    Boat.m_Pos[0] = initial_state[0]
    Boat.m_Pos[1] = initial_state[1]
    Boat.m_Vel[0] = initial_state[2]
    Boat.m_Vel[1] = initial_state[3]
    Boat.m_Phi = initial_state[4]
    Boat.m_Omega = initial_state[5]



def run_simulation(dt, runtime, array):
    '''
    runs the simulation
    inputs:
    float dt - simulation time step [s]
    float runtime - how long the simulation should run for [s]

    outputs the data into startarray
    '''
    global running, endtime
    running = True
    endtime = runtime + dd.t

    while running:
        move(dt, array, endtime)
        New = np.array([Boat.m_Pos[0],
                        Boat.m_Pos[1],
                        Boat.m_Vel[0],
                        Boat.m_Vel[1],
                        Boat.m_Phi,
                        Boat.m_Omega,
                        dd.t])
        array = np.vstack([array, New])

    return(array)




#sets the values for the resistance coefficients based on the boat geometry
#this is for some test boat, provided by the paper.
def setWaterCoefficients(c, w):
    '''
    sets the values for the coefficients

    class: c - class of coefficients to be assigned
    float: w - relative resistance direction, from relative.phi class
    '''
    x = abs(w)

    #this is another set for a different test boat, rather than a real one
    #c.Cd = 0.5*np.sin(2*x) + (pi/2 - x)/pi
    #c.Cs = np.sign(w)*0.5*np.sin(x)
    #c.Cy = np.sign(w)*0.5*np.sin(2*x)
    #c.Cs_R = 0.5


    c.Cd = 0.245219 - 0.93044*x + 0.745752*x*x - 0.15915*x*x*x + 2.79188 * np.sin(2*x) * np.exp(-1.05667*(x - pi/2)*(x - pi/2))
    c.Cs = np.sign(w)*(0.115554 + 3.09423*x - 0.984923*x*x) * np.sin(x)
    c.Cy = np.sign(w)*(1.61493 + 1.58982*x - 0.510922*x*x) * np.sin(2*x)
    c.Cs_R = 0.115554 + 3.09423*pi/2 - 0.984923*pi/2*pi/2



def setGeoParameter(BoatGP):
    gp = BoatGP
    gp.T = 8000 # maximal thrust [N]
    gp.Lt = 5 # thrust arm [m]
    gp.m = 5000 # boat Mass [kg]
    gp.L = 10.0 # boat length [m]
    gp.J = 23000 # moment of Inertia [kg m^2]
    gp.frontAreaAir = 2.5 # air front cross section
    gp.frontAreaWater = 15.0 # water front cross section

def setMatParameter(MatParameter):
    mp = MatParameter
    mp.rhoWater = 1000 # water density [kg/m^3]
    mp.rhoAir = 1.225 # air density [kg/m^3]

def setInital(BoatDD):
    dd = BoatDD
    vRx = 0.0 #initial velotity
    vRy = 0.0

    dd.p[0] = 0.0 #initial positions
    dd.p[1] = 0.0

    dd.phi = getMathAngle(0) #initial angle of boat

    dd.v[0] = vRx * np.cos(dd.phi) #initial velocity
    dd.v[1] = vRy * np.sin(dd.phi)

    dd.omega = 0.0 #initial angular velocity



def animate_boat(animation_array):
    '''
    Animates a given boat path generated previously

    inputs:
    animation_array: generated boat path [x, y, vx, vy, phi, omega]
    '''
    global ani

    theta = animation_array[:,4:5]
    t = animation_array[:,6:7]

    #draw boat
    h = 10
    w = 5
    vertices0_frame = np.array([[-w/2, -h/2], [w/2, -h/2], [w/2, h/6], [0, h/2], [-w/2, h/6]]).T

    R = np.array([[np.sin(float(theta[0])), np.cos(float(theta[0]))], [-np.cos(float(theta[0])), np.sin(float(theta[0]))]])
    vertices_frame = R @ vertices0_frame

    patch_frame = Polygon(vertices_frame.T,  edgecolor ='indigo', facecolor = 'grey')

    fig = plt.figure(figsize = (7,7))
    ax = plt.gca()
    ax.add_patch(patch_frame)
    ax.set_aspect('equal', adjustable='box')
    text_t = ax.set_xlabel(f't = {float(np.round(t[0], 2)):.2f}')
    plt.show()

    def init():
        return patch_frame

    def animate(i):
        R = np.array([[float(np.sin(theta[i])), float(np.cos(theta[i]))], [float(-np.cos(theta[i])), float(np.sin(theta[i]))]])
        vertices_frame = R @ vertices0_frame
        x = float(animation_array[:,0:1][i])
        y = float(animation_array[:,1:2][i])
        vertices_frame[0] = vertices_frame[0] + x
        vertices_frame[1] = vertices_frame[1] + y

        patch_frame.set_xy(vertices_frame.T)
        text_t.set_text(f't = {float(np.round(t[i], 2)):.1f}s')
        return patch_frame

    ani = animation.FuncAnimation(fig, animate, len(theta), init_func=init, interval=1, repeat=False)
    plt.plot(animation_array[:,:1],animation_array[:,1:2], linestyle = '--')
    plt.show()