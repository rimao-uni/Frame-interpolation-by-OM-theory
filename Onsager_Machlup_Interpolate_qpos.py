# from inspect import isframe
import math, pickle, re, time
import numpy as np
import scipy, scipy.fft
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
# from scipy.spatial.transform import Rotation as R
# sample に従う。

# スタート位置、ゴール位置、手本となる軌道のmotion indexを定義する。
start = 7
example = 4
end = 2

def calcLy(y):
  nframe1 = y.shape[0] - 1
  Ly = np.zeros_like(y)
  for i in range(1,nframe1):
      Ly[i,:] = -y[i-1,:] + y[i,:]*2 - y[i+1,:]
  Ly[0,:] = y[0,:]*2 - y[0+1,:]
  Ly[nframe1,:] = -y[nframe1-1,:] + y[nframe1,:]*2
  return Ly

def chksol(nframe,x,y,xfix,ifix):
#  make full L matrix
  L = np.zeros((nframe,nframe),dtype=np.float32)
  for i in range(1,nframe-1):
    L[i,i-1] = -1; L[i,i] = 2; L[i,i+1] = -1
  L[0,0] = 2; L[0,1] = -1
  L[nframe-1,nframe-2] = -1; L[nframe-1,nframe-1] = 2

  xtmp = np.zeros_like(y)
  xtmp[ifix,:] = xfix[:,:]
  ind = [i for i in range(nframe) if i not in ifix]
  xtmp[ind[:],:] = x[:,:] #; print('xtmp=',xtmp.shape,y.shape,L.shape)

  err = np.matmul(xtmp.transpose(),L) - np.matmul(y.transpose(),L)
  return err  

npydata = np.load("npyresult/results.npy", allow_pickle=True)  #  numpy array[nbatch,nframes,76]

npydata = npydata.item()['motion'] 
#  motion exampleをお手本にするが、0th frameは motion start, last frameはmotion endから取る。
y = npydata[example,:,:]#.transpose(1,0)#; print('y=',y.shape,y) #,calcLy(y))

nframe = len(y); ifix = [0,nframe-1]; nfix = len(ifix); n = nframe - nfix
# [1~194]
ind = [i for i in range(nframe) if i not in ifix]
# 0の最初のframe、last frameはmotion endの170frameから取る。
xfix = np.array([npydata[start,0,:],npydata[end,170,:]])#; print(y.shape,xfix.shape); raise RuntimeError
# 連立方程式右辺 b の計算
xtmp = y.copy()#; print('ifix,xfix=',ifix,xfix)

# お手本の動きの最初と最後を0の最初のフレーム、1の最後のフレームで引く
xtmp[ifix,:] -= xfix[:,:]; 
btmp = calcLy(xtmp)
b = btmp[ind,:]

# Lx = b をFFTで解く。
Vb = scipy.fft.idst(b,type=1,axis=0,norm='ortho')

for k in range(n):
#  print('e,Vb=',np.cos((k+1)/(n+1) * np.pi)*2 + 2, Vb[n-1-k,:])
  Vb[n-1-k,:] /= (np.cos((k+1)/(n+1) * np.pi)*2 + 2)
# print('Vb=',Vb.shape,Vb)
x = scipy.fft.dst(Vb,type=1,axis=0,norm='ortho')#; print('x=',x)
err = chksol(nframe,x,y,xfix,ifix)
err1 = np.max(np.abs(err),axis=0); # print('err1=',err1)

# 解いたxの可視化
model = mujoco.MjModel.from_xml_path('assets/mujoco_models/mjmodel.xml')
data = mujoco.MjData(model)
viewer = mujoco.viewer.launch_passive(model, data)

# 最初フレーム
for qpos1 in range(196):

  mujoco.mj_resetData(model,data)
  data.qpos[:] = npydata[start,qpos1,:]
  mujoco.mj_forward(model,data)
  viewer.sync()
  time.sleep(1/40)
  if qpos1 == 0:
    break
time.sleep(3)

# 最終フレーム
for qpos1 in range(196):
  if qpos1 != 170:
    continue
  mujoco.mj_resetData(model,data)
  data.qpos[:] = npydata[end,qpos1,:]
  mujoco.mj_forward(model,data)
  viewer.sync()
  time.sleep(1/40)
time.sleep(3)

# 手本motionと補間motion
for xy in [y,x]:

  for i,qpos in enumerate(xy):
    mujoco.mj_resetData(model,data)
    data.qpos[:] = qpos[:]
    mujoco.mj_forward(model,data)
    viewer.sync()
    time.sleep(1/40)  

  time.sleep(3)
viewer.close()

