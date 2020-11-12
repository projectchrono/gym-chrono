import os
import glob
import shutil
import random as np

#os.system("export PYTHONPATH=/home/simonebenatti/codes/chronosensor/chrono_build/bin")
#os.system("export CHRONO_DATA_DIR=/home/simonebenatti/codes/chronosensor/chrono-dev/data/")
for i in range(12):
	#ep = np.randint(2, 6)
	folder = './Video/frames'+str(i)+'/'
	os.mkdir(folder)
	os.system("python trainMS.py gym_chrono.envs:off_road-v2 ppoCNN_off_road.pth -l=1e-4 -e=1 -s=300 -u=1000 -m=150 -p=6 -a=MultiSensorEarlyFusion --play_mode --max_episodes="+str(i))
	for file in glob.glob("frame*"):
		shutil.move(file, folder+file)
