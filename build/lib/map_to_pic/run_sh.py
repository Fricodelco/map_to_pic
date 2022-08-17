import subprocess
import os
import threading
from time import sleep
import signal
import psutil
class run_prog():
    def __init__(self):
        dir_path = os.path.dirname(os.path.realpath(__file__))
        x = dir_path.split('/')
        dir_path = ''
        for i in range(0, len(x)-1,1):
            dir_path+=x[i]+'/'
        dir_path = dir_path +'sh_scripts/'
        # p = subprocess.call(['sh', dir_path+'run_map_server.sh'])
        # os.system('. '+dir_path+'run_map_server.sh')
        self.cmd = [dir_path+'run_map_server.sh']

        # self.cmd = ['exec . /home/rodion/.bashrc; ros2 run examples_rclcpp_minimal_publisher publisher_lambda']
        self.shellscript = 1
    def run(self):
        self.shellscript = subprocess.Popen(self.cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE, shell=True)
    def stop(self, timeout=None):
        # os.killpg(self.shellscript.pid, signal.SIGTERM)
        # self.shellscript.kill()
        # process = psutil.Process(self.shellscript.pid)
        # for proc in process.children(recursive=True):
        #     proc.kill()
        # process.kill()
        subprocess.Popen("KILL /F /PID {pid} /T".format(pid=self.shellscript.pid))
        # os.killpg(os.getpgid(self.shellscript.pid), signal.SIGTERM)
def save_map():
    dir_path = os.path.dirname(os.path.realpath(__file__))
    x = dir_path.split('/')
    dir_path = ''
    for i in range(0, len(x)-1,1):
       dir_path+=x[i]+'/'
    dir_path = dir_path +'sh_scripts/'
    p = subprocess.call(['sh', dir_path+'run_map_server.sh'])
    # os.system('. '+dir_path+'run_map_server.sh')
    # print(p)
def main():
    # sub = run_prog()
    # sub.run()
    # sub.stop()
    save_map()
if __name__ == '__main__':
    main()
