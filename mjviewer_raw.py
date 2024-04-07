from inspect import isframe
import pickle, re, time
import numpy as np
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
from scipy.spatial.transform import Rotation as R
import sys

# データをそのまま拡散モデルに突っ込んで学習。生成データをmujocoで見る。
# コマンドライン引数からmotion indexを受け取る。

args = sys.argv
motion_index = int(args[1])

npydata = np.load("npyresult/results.npy", allow_pickle=True)
npydata = npydata.item()

model = mujoco.MjModel.from_xml_path('assets/mujoco_models/mjmodel.xml')
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

qpos = npydata["motion"][motion_index]
x1 = [x for x in range(196)]
y1 = np.zeros((196,76))
counter = 0
for qpos1 in qpos:
  mujoco.mj_resetData(model,data)
  data.qpos[:] = qpos1[:76]
  y1[counter,:] = qpos1[:76]
  mujoco.mj_forward(model,data)
  viewer.sync()
  time.sleep(1/40)
  counter += 1
time.sleep(3)

flg = "Pelvis"
if flg == "Pelvis":
  for i in range(3):
    y2 = y1[:,i]
    plt.xlabel('frame[(1/40)s]')
    plt.ylabel('Position')
    plt.plot(x1, y2, label="Pelvis")
else:
  for i in range(7,76):
    y2 = y1[:,i]
    plt.xlabel('frame[(1/40)s]')
    plt.ylabel('Angele[π]')
    plt.plot(x1, y2, label="Joint")

plt.show()