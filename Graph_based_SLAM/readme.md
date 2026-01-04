```markdown
公式如下：

$$
e_{ij} =
\begin{bmatrix}
R_{ij}^T (R_i^T (t_j - t_i) - t_{ij})
\end{bmatrix}_{2 \times 1}
=
\begin{bmatrix}
e_t \\
e_0
\end{bmatrix}
$$

偏導數公式：

$$
\frac{\partial e_t}{\partial x_i} = -R_{ij}^T R_i = \frac{\partial e_t}{\partial y_i}
$$

$$
\frac{\partial e_t}{\partial \theta_i} = R_{ij}^T \frac{\partial R_i^T}{\partial \theta_i} (t_j - t_i)
= R_{ij}^T 
\begin{bmatrix}
\cos \theta_i & \sin \theta_i \\
-\sin \theta_i & \cos \theta_i
\end{bmatrix}
\frac{\partial}{\partial \theta_i} (t_j - t_i)
= R_{ij}^T 
\begin{bmatrix}
-\sin \theta_i & \cos \theta_i \\
-\cos \theta_i & -\sin \theta_i
\end{bmatrix} (t_j - t_i)
$$

$$
\frac{\partial e_t}{\partial x_j} = R_{ij}^T R_i = \frac{\partial e_t}{\partial y_j}
$$

$$
\frac{\partial e_t}{\partial \theta_j} = 0 \quad (\text{no } \theta_j \text{-related term})
$$
