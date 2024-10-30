global Kp,Kd,Ki,Del,Integ
Kp = 4.5
Kd = 0.00112
Ki =0.5
Del = 0
Integ = 0
global forwardVel,turnVel
forwardVel = 0.0
turnVel = 0.0
def sysCall_init():
    sim = require('sim')
    global  botBody,rightWheel,leftWheel
    botBody = sim.getObject('/body')
    rightWheel = sim.getObject('/body/right_joint')
    leftWheel = sim.getObject('/body/left_joint')
    global pitchAngle,pitchError,prevPitchError
    pitchAngle = 0
    pitchError = 0
    prevPitchError =0
def sysCall_actuation():
    global pitchError,prevPitchError,Integ,Del,Kp,Ki,Kd,forwardVel,turnVel
    dt = sim.getSimulationTimeStep()
    Del = (pitchError-prevPitchError)/dt
    Integ = (Integ + pitchError)*dt
    print(pitchError)
    Co = max(min((Kp*pitchError - Ki*Integ - Kd*Del)*1000,20),-20)
    print(str(sim.getJointTargetVelocity(rightWheel)) + " " + str(sim.getJointTargetVelocity(leftWheel)))
    sim.setJointTargetVelocity(rightWheel,-Co+forwardVel-turnVel)
    sim.setJointTargetVelocity(leftWheel,-Co+forwardVel+turnVel)
    pass

def sysCall_sensing():
    global pitchAngle,pitchError,botBody,forwardVel,turnVel
    pitchAngle = sim.getObjectOrientation(botBody,sim.handle_world)[0]
    pitchError = pitchAngle - 0.0
    message, data, data2 = sim.getSimulatorMessage()
    if message == sim.message_keypress:
        if data[0] == 2007:  # Up arrow key (move forward)
            forwardVel = 5.0
        elif data[0] == 2008:  # Down arrow key (move backward)
            forwardVel = -5.0
        else:
            forwardVel = 0.0

        if data[0] == 2009:  # Left arrow key (turn left)
            turnVel = 2.0
        elif data[0] == 2010:  # Right arrow key (turn right)
            turnVel = -2.0
        else:
            turnVel = 0.0
    pass

def sysCall_cleanup():
   
    pass

# See the user manual or the available code snippets for additional callback functions and details