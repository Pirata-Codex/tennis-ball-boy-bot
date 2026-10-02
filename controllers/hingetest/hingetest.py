import math
import sys

from controller import Supervisor

s = Supervisor()
STEP = int(s.getBasicTimeStep())

NOMINAL = {
    'BODY_WHEEL_LEFT': (-0.10, 0.185, 0.09),
    'BODY_WHEEL_RIGHT': (-0.10, -0.185, 0.09),
}
parts = {k: s.getFromDef(k) for k in NOMINAL}

wm = s.getDevice('leftWheelMotor')
rm2 = s.getDevice('rightWheelMotor')
for m in (wm, rm2):
    if m is None:
        continue
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
TORQUE = float(sys.argv[1]) if len(sys.argv) > 1 else 4.0
if abs(wm.getMaxTorque() - TORQUE) > 1e-9:
    print('hingetest: cannot retorque at runtime; world says %.2f'
          % wm.getMaxTorque(), flush=True)
rm = s.getDevice('intakeRoller')
if rm is not None:
    rm.setPosition(float('inf'))
    rm.setVelocity(0.0)

print('hingetest: wheel=%s right=%s maxTorque=%.2f'
      % ('yes' if parts['BODY_WHEEL_LEFT'] else 'no',
         'yes' if parts.get('BODY_WHEEL_RIGHT') else 'no', TORQUE), flush=True)
enc = s.getDevice('leftWheelEncoder')
if enc is not None:
    enc.enable(STEP)
print('t,phase,wheel_dx,wheel_dz,wheel_z,roller_dz,chassis_z,pitch,rx,enc',
      flush=True)

IDLE = 100
LEG = 150
SPD = 6.0
cmd = 0.0
for i in range(IDLE + LEG):
    if i >= IDLE:
        cmd = min(SPD, cmd + (1.2 / 0.09) * 0.032)
    wm.setVelocity(cmd)
    if rm2 is not None:
        rm2.setVelocity(cmd)
    if rm is not None:
        rm.setVelocity(0.0)

    # Track the CHASSIS, not the Robot node. In this world the Robot node is
    # itself a tiny free body (0.05 kg, 2 mm bounding box) that is jointed to
    # nothing, so getSelf().getPosition() reports where that speck is, not
    # where the 12 kg chassis it contains has driven to.
    chassis = s.getFromDef('BODY_CHASSIS')
    pos = chassis.getPosition()
    o = chassis.getOrientation()

    def dev(name):
        n = parts.get(name)
        if n is None:
            return (float('nan'),) * 3
        p = n.getPosition()
        nx, ny, nz = NOMINAL[name]
        return (p[0] - pos[0] - nx, p[1] - pos[1] - ny, p[2] - pos[2] - nz)

    if i % 5 == 0:
        wd = dev('BODY_WHEEL_LEFT')
        rd = dev('BODY_WHEEL_RIGHT')
        if rm is None:
            rd = (rd[0], float('nan'), rd[2])
        pitch = math.degrees(math.atan2(-o[2], math.hypot(o[5], o[8])))
        print('%.2f,%s,%+.4f,%+.4f,%.4f,%+.4f,%.4f,%+.2f,%.4f,%.3f'
              % (i * STEP / 1000.0, 'idle' if i < IDLE else 'drive',
                 wd[0], wd[2], wd[2] + 0.09, rd[2], pos[2], pitch, pos[0],
                 enc.getValue() if enc is not None else float('nan')),
              flush=True)
    s.step(STEP)

s.simulationQuit(0)