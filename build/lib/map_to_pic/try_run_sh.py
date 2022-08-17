from run_sh import run_prog
from time import sleep
sub = run_prog()
sub.run()
for i in range(0,5,1):
    print(i)
    sleep(0.5)
sub.stop()
print("something")