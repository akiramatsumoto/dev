# Joint_Commander

## 概要
ざっくり何をするプログラムか  
→歩行パターン生成によって与えられる重心・足先の目標位置・速度から関節の目標位置・速度を求める

どのような手順で解いているか  
1.重心・足先の目標位置から逆運動学を解いて関節の目標位置を求める  
2.1で得た関節の目標位置をもとに順運動学を解く  
3.2で得た順運動学をもとにヤコビアンを求める  
4.3で得たヤコビアンと重心・足先の目標速度をもとに関節の目標速度を求める  

## 使用している数式
### 逆運動学
あとで書く
### 順運動学
あとで書く
### ヤコビアン
ここで，$\mathbf{J}$はヤコビアンを，${}^{W}\mathbf{a}$はそのリンクの回転軸を，$p$はそのリンクの位置を表す．  
（ヒューマノイドロボット，オーム社，P74，式2.73参考） 

$\mathbf{J} = \begin{bmatrix}
{}^{W}\mathbf{a}_{rwl} \times (p_e - p_{rwl}) \quad 
{}^{W}\mathbf{a}_{rwp} \times (p_e - p_{rwp}) \quad 
{}^{W}\mathbf{a}_{rkp} \times (p_e - p_{rkp}) 
\end{bmatrix}
$

### 関節速度
順運動学により求めた足先の座標を$p$，重心の座標を$p_b$，  
足先の目標速度$v$を  

$\mathbf{v} = \begin{bmatrix}
v_x \\
v_y \\
v_z \\
\end{bmatrix}
$，  

重心の目標速度$v_b$を  

$\mathbf{v_b} = \begin{bmatrix}
v_{bx} \\
v_{by} \\
v_{bz} \\
\end{bmatrix}
$，  

重心の目標角速度$\omega_b$を  

$\mathbf{\omega_b} = \begin{bmatrix}
\omega_{bx} \\
\omega_{by} \\
\omega_{bz} \\
\end{bmatrix}
$，  

としたとき，

関節速度$\dot{q}$を  

$\dot{q} = J^{-1}\begin{bmatrix}
v - (v_b + \omega_b \times(p-p_b))
\end{bmatrix}
$  

として求めている．
（ヒューマノイドロボット，オーム社，P75，式2.75参考） 

