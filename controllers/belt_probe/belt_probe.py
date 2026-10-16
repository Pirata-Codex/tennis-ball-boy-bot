"""Probe: does a Track belt move a ball? Drives every LinearMotor and logs
the balls' x positions. Used to diagnose FINDINGS 6.7 (belts had no effect)."""
from controller import Supervisor

sup = Supervisor()
dt = int(sup.getBasicTimeStep())
motors = []
for i in range(sup.getNumberOfDevices()):
    d = sup.getDeviceByIndex(i)
    if d.getNodeType() == 54 or 'Motor' in d.getName():
        try:
            d.setPosition(float('inf'))
            d.setVelocity(-0.3)
            motors.append(d.getName())
        except Exception:
            pass
print('motors', motors)
balls = [sup.getFromDef(n) for n in ('BALL_DIRECT', 'BALL_NESTED')]
t = 0.0
while sup.step(dt) != -1:
    t += dt / 1000.0
    if abs(t * 2 - round(t * 2)) < 1e-6:
        print('t=%.1f' % t, ' '.join('%s x=%+.3f z=%.3f' % (n, b.getPosition()[0], b.getPosition()[2])
                                      for n, b in zip(('direct', 'nested'), balls)))
    if t > 3.0:
        sup.simulationQuit(0)
