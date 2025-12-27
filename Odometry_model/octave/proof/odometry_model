# Odometry Model

狀態向量：
\[
\begin{bmatrix}
x \\
y \\
\theta
\end{bmatrix}
\;\longrightarrow\;
\begin{bmatrix}
\bar{x}' \\
\bar{y}' \\
\bar{\theta}'
\end{bmatrix}
\]

---

## 定義

\[
\delta_{trans} = \sqrt{(\bar{x}' - \bar{x})^2 + (\bar{y}' - \bar{y})^2}
\]

\[
\delta_{rot1} = \operatorname{atan2}(\bar{y}' - \bar{y},\; \bar{x}' - \bar{x}) - \bar{\theta}
\]

\[
\delta_{rot2} = \bar{\theta}' - \bar{\theta} - \delta_{rot1}
\]

---

## 問題設定

給定：
\[
\begin{bmatrix}
\delta_{rot1},\; \delta_{trans},\; \delta_{rot2}
\end{bmatrix}^T
\]

計算：
\[
\begin{bmatrix}
\bar{x}',\; \bar{y}',\; \bar{\theta}'
\end{bmatrix}^T
\]

初始姿態：
\[
\begin{bmatrix}
\bar{x},\; \bar{y},\; \bar{\theta}
\end{bmatrix}^T
=
\begin{bmatrix}
0,\;0,\;0
\end{bmatrix}^T
\]

---

## 角度關係

\[
\bar{\theta}' = \delta_{rot2} + \bar{\theta} + \delta_{rot1}
\]

---

## 幾何推導

定義：
\[
\bar{x}' - \bar{x} = A, \quad \bar{y}' - \bar{y} = B
\]

\[
\delta_{trans}^2 = A^2 + B^2
\]

\[
\tan(\delta_{rot1} + \bar{\theta}) = \frac{B}{A}
\]

---

### 解出 \(A, B\)

\[
B = A \tan(\delta_{rot1} + \bar{\theta})
\]

\[
\delta_{trans}^2
= A^2 \left(1 + \tan^2(\delta_{rot1} + \bar{\theta})\right)
\]

\[
= A^2 \sec^2(\delta_{rot1} + \bar{\theta})
\]

\[
\Rightarrow
A^2 = \delta_{trans}^2 \cos^2(\delta_{rot1} + \bar{\theta})
\]

\[
A = \delta_{trans} \cos(\delta_{rot1} + \bar{\theta})
\]

\[
B = \delta_{trans} \sin(\delta_{rot1} + \bar{\theta})
\]

---

## 最終更新方程式

\[
\boxed{
\begin{aligned}
\bar{x}' &= \bar{x} + \delta_{trans} \cos(\delta_{rot1} + \bar{\theta}) \\
\bar{y}' &= \bar{y} + \delta_{trans} \sin(\delta_{rot1} + \bar{\theta})
\end{aligned}
}
\]
