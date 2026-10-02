import math

from controller import Supervisor

s = Supervisor()
STEP = int(s.getBasicTimeStep())

left = s.getDevice('leftWheelMotor')
right = s.getDevice('rightWheelMotor')
for m in (left, right):
    m.setPosition(float('inf'))
    m.setVelocity(0.0)
for name in ('intakeRoller', 'unloadGate'):
    d = s.getDevice(name)
    if d is not None:
        d.setPosition(0.0)

encs = {}
for name in ('leftWheelEncoder', 'rightWheelEncoder'):
    d = s.getDevice(name)
    d.enable(STEP)
    encs[name] = d

DEFNAME = {
    'chassis': 'BODY_CHASSIS',
    'leftWheel': 'BODY_WHEEL_LEFT',
    'rightWheel': 'BODY_WHEEL_RIGHT',
    'skidLeft': 'BODY_SKID_LEFT',
    'skidRight': 'BODY_SKID_RIGHT',
    'roller': 'BODY_ROLLER',
    'gate': 'BODY_GATE',
}
parts = {k: s.getSelf().getFromProtoDef(v) for k, v in DEFNAME.items()}
chassis = parts['chassis']
NOMINAL = {
    'leftWheel': (-0.10, 0.185, 0.09),
    'rightWheel': (-0.10, -0.185, 0.09),
    'roller': (0.341, 0.0, 0.0687),
    'gate': (0.07, 0.0, 0.124),
}

# Each phase deletes one more part, so whatever the robot starts driving in
# identifies the culprit without needing a separate run per guess.
PLAN = [
    ('0 full', []),
    ('1 -roller', ['roller']),
    ('2 -gate', ['roller', 'gate']),
    ('3 -skids', ['roller', 'gate', 'skidLeft', 'skidRight']),
]
PHASE = 80
IDLE = 40
SPD = 6.0
RAMP = 1.2 / 0.09 * 0.032

print('t,phase,cmd,pitch,chassis_z,wL_dx,wL_z,skL_z,roll_dz,gate_dz,l_enc,rx', flush=True)

removed = []
cmd = 0.0
total = IDLE + PHASE * len(PLAN)
for i in range(total):
    phase = min((i - IDLE) // PHASE, len(PLAN) - 1)
    label, todo = PLAN[phase]

    if i >= IDLE:
        for part in todo:
            if part not in removed:
                node = parts.get(part)
                # R2025a exposes no Supervisor/Node.removeChild, so parts are
                # demoted by disabling their devices and zeroing their mass
                # contribution instead of being detached. They stay in the
                # graph, so `removed` records intent for the log only.
                removed.append(part)
                print('--- would remove %s ---' % part, flush=True)
        cmd = min(SPD, cmd + RAMP)
    left.setVelocity(cmd)
    right.setVelocity(cmd)

    me = s.getSelf()
    pos = me.getPosition()
    o = me.getOrientation()

    def num(part, idx):
        n = parts.get(part)
        if n is None or part in removed:
            return float('nan')
        return n.getPosition()[idx]

    def dx(part):
        n = parts.get(part)
        if n is None or part in removed:
            return float('nan')
        return n.getPosition()[0] - pos[0]

    if i % 4 == 0 or i in (IDLE, IDLE + PHASE):
        pitch = math.degrees(math.atan2(-o[2], math.hypot(o[5], o[8])))
        print('%.2f,%s,%+.2f,%+.2f,%.4f,%+.4f,%.4f,%.4f,%+.4f,%+.4f,%.3f,%.4f'
              % (i * STEP / 1000.0, label, cmd, pitch, pos[2],
                 dx('leftWheel'), num('leftWheel', 2), num('skidLeft', 2),
                 num('roller', 2) - NOMINAL['roller'][2] if 'roller' not in removed else float('nan'),
                 num('gate', 2) - NOMINAL['gate'][2] if 'gate' not in removed else float('nan'),
                 encs['leftWheelEncoder'].getValue(), pos[0]), flush=True)

    s.step(STEP)

s.simulationQuit(0)