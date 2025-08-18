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
ここで，$\mathbf{J}$ はヤコビアンを，${}^{W}\mathbf{a}$ はそのリンクの回転軸を，$\mathbf{p}$ はそのリンクの位置を表す．  
（ヒューマノイドロボット，オーム社，P74，式2.73参考）

$$
\mathbf{J} =
\begin{bmatrix}
{}^{W}\mathbf{a}_{\mathrm{rwl}} \times \bigl(\mathbf{p}_e - \mathbf{p}_{\mathrm{rwl}}\bigr) &
{}^{W}\mathbf{a}_{\mathrm{rwp}} \times \bigl(\mathbf{p}_e - \mathbf{p}_{\mathrm{rwp}}\bigr) &
{}^{W}\mathbf{a}_{\mathrm{rkp}} \times \bigl(\mathbf{p}_e - \mathbf{p}_{\mathrm{rkp}}\bigr)
\end{bmatrix}
$$

### 関節速度
順運動学により求めた足先の座標を $\mathbf{p}$，重心の座標を $\mathbf{p}_b$，  
足先の目標速度 $\mathbf{v}$ を

$$
\mathbf{v} =
\begin{bmatrix}
v_x \\
v_y \\
v_z
\end{bmatrix}
$$

重心の目標速度 $\mathbf{v}_b$ を

$$
\mathbf{v}_b =
\begin{bmatrix}
v_{bx} \\
v_{by} \\
v_{bz}
\end{bmatrix}
$$

重心の目標角速度 $\boldsymbol{\omega}_b$ を

$$
\boldsymbol{\omega}_b =
\begin{bmatrix}
\omega_{bx} \\
\omega_{by} \\
\omega_{bz}
\end{bmatrix}
$$

としたとき，関節速度 $\dot{\mathbf{q}}$ は

$$
\dot{\mathbf{q}}
= \mathbf{J}^{-1}\Bigl( \mathbf{v} - \bigl( \mathbf{v}_b + \boldsymbol{\omega}_b \times (\mathbf{p} - \mathbf{p}_b) \bigr) \Bigr)
$$

として求める．（ヒューマノイドロボット，オーム社，P75，式2.75参考）
