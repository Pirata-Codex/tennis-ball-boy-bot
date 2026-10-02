from controller import Supervisor

s = Supervisor()
t = int(s.getBasicTimeStep())
m = s.getDevice('leftWheelMotor')
m.setPosition(float('inf'))
m.setVelocity(5.0)
print('stepcheck: timestep=%d mode=%d' % (t, s.simulationGetMode()), flush=True)
for i in range(200):
    r = s.step(t)
    if i < 5 or r != 0:
        p = s.getSelf().getPosition()
        print('  i=%3d step=%d mode=%d self=(%.4f,%.4f,%.4f)'
              % (i, r, s.simulationGetMode(), p[0], p[1], p[2]), flush=True)
    if r != 0:
        print('  -> nonzero return, stopping', flush=True)
        break
print('stepcheck: ran %d steps' % (i + 1), flush=True)
s.simulationQuit(0)