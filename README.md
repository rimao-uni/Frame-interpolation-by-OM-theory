
# **frame-interpolation-by-Onsager-Machlup**

## 概要 
生成されたデータで両端のモーションを与え, OM理論に基づく計算手法によって拘束条件に従うような最尤経路を計算する.  
例えば以下のように端点を拘束条件をとして与える.  

<img src="https://github.com/rimao-uni/Frame-interpolation-by-OM-theory/assets/117995370/23aa5926-fb31-49d1-8201-6d218ca93aab" height="250">
<img src="https://github.com/rimao-uni/Frame-interpolation-by-OM-theory/assets/117995370/14266795-9c37-4d94-8e70-0e7be0be8fe4" height="250">

　　　　　[スタート位置]　　　　　　　 　　　　  [ゴール位置]　　　　　　　　　　

手本起動を左のように定めると, 右のように始点から手本軌道のめくような運動をしながら最終的にゴール位置に向かうような軌道が得られる.  

<img src="https://github.com/rimao-uni/Frame-interpolation-by-OM-theory/assets/117995370/58fa6976-06cf-4037-874f-4cf4a8512a58" height="400">  
<img src="https://github.com/rimao-uni/Frame-interpolation-by-OM-theory/assets/117995370/08a3790c-cadc-4e9c-8f1e-642f340db244" height="400">

　　　　[手本となる軌道]  　　　　　　　　　　　[最尤経路を辿る軌道]  

## Dependencies  
環境を作成するには次の手順に従う.  
新しい conda 環境を作成し, 以下をインストール  
シミュレーション環境としてはオープンソースの物理演算エンジンである[Mujoco](https://mujoco.org/)を使用する.
```
conda create -n uhc python=3.8
pip install -r requirements.txt
```
## 実行方法
※入力する軌道はいくつかのmotionを/npyresult/results.npy に保存する.(motionの生成方法は今後公開予定. 現在はサンプルをご利用ください.)

以下のようにして, [--n]にmotion indexをコマンドライン引数として入力し, 生成データの中身を確認する.  
```
mjpython mjviewer_raw.py --n
```

以下のようにして, フレーム補間結果を可視化する.   
※Onsager_Machlup_Interpolate_qpos.pyのstart, example, end に任意のmotion indexを定義することができる.
```
mjpython Onsager_Machlup_Interpolate_qpos.py  
```

## 理論の概要
Onsager-Machlup作用とは, 拡散過程の確率的な過程に対する定式化を行ったもので, 拡散モデルと同じ過減衰ランジュバン方程式に従う系において, 分子のブラウン運動における最尤経路を導出するために用いられる. この手法を拘束条件を与えたモーションの最尤経路を計算するために用いた. まず, 以下のような外力𝐹(𝑡)を受けて運動する粒子系 𝑥の過減衰ランジュバン方程式 
$$\frac{dx}{dt}=\mu F+\sqrt{2\mu K_BT}\xi\left(t\right)$$

を考える. ここで, 𝜇は摩擦定数 𝑟の逆数であり, $K_BT$は温度, 𝜉(𝑡)はランダム力である. このとき経路 𝑥(𝑡)をとる確率 𝑝[𝑥(𝑡)]は以下のような関係にある. 
$$p\left[x\left(t\right)\right]\propto exp\left(-\int_{t_i}^{t_f}{\frac{\left(\dot{x}-\mu F\right)^2}{4\mu K_BT}dt}\right)$$

この指数にあたる部分が Onsager-Machlup作用であり, これを最小化することで 𝑥(𝑡)の最尤経路を得ることができる. また運動方程式は以下のようになる. 
$$m\ddot{x}\left(t\right)=F-r\dot{x}\left(t\right)+\xi\left(t\right)$$

慣性項 𝑚𝑥̈(𝑡)が無視できるとき, 軌道の短時間平均を取ると, 𝐹 = 𝑟𝑥̇(𝑡)となる. このときの外力が同じであるような目標としたい動き(以下目標軌道) 𝑦(𝑡)があるとき, 𝑦と同じ初期条件から出発すれば, 𝑥(𝑡) = 𝑦(𝑡)となる. よって Onsager-Machlup 作用 
$$S_{OM}=\int_{t_i}^{t_f}\frac{r}{4K_BT}\left(\dot{x}\left(t\right)-\dot{y}\left(t\right)\right)^2dt$$
を最小にする経路 𝑥(𝑡)を 𝑥の拘束条件下で最小化することで, 拘束条件を与えたときの経路 𝑦(𝑡)の最尤経路を求めることができる.   
𝑦(𝑡)が等間隔の時刻で与えられるとき, 速度を中心差分で求め, 定数項を無視して積分を台形則で求めると以下のようになる.   
$$S_{OM}\simeq\sum_{i=0}^{P-1}\left(\left(x_{i+1}-x_i\right)-\left(y_{i+1}-y_i\right)\right)^2$$

よって $x_k$における極値必要条件では, 微分すると以下のような関係になる.  
$$\frac{{\partial S}_{OM}}{\partial x_k}=0,k\neq0,P$$  

また $k=0,P$においてはそれぞれ ${-x_1+x}_0={-y_1+y}_0$, $x_P{-x} _ {P-1}=y_P-y _{P-1}$ となる。
  
係数行列を $L$とし, 例えば端点のフレームが ${\widetilde{x}}_0$, ${\widetilde{x}}_P$と定まっているとき以下のような連立方程式となる.  
$$Lx=Ly-L\widetilde{x}$$

この連立方程式を数値的に解き, 目標軌道 𝑦(𝑡)の拘束条件に従った経路 𝑥(𝑡)を得る.   

$$ x=\left(\begin{matrix}0\\x_1\\\vdots\\x_{P-1}\\0\\\end{matrix}\right)$$


## References
・SMPL構造のデータをMujocoで扱えるようにするため, [Universal Humanoid Controller](https://github.com/ZhengyiLuo/UHC?tab=readme-ov-file)のソースコードを参照し, データを変換した.
