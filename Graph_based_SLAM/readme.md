## Equation

$$
\mathbf{e}_{ij}
=
\left[
\mathbf{R}_{ij}^{\top}
\left(
\mathbf{R}_i^{\top}(\mathbf{t}_j-\mathbf{t}_i)-\mathbf{t}_{ij}
\right)
\right]_{2\times 1}
=
\begin{bmatrix}
e_t \\
e_0
\end{bmatrix}
$$

## Partial derivatives

### With respect to $x_i$ and $y_i$

$$
\frac{\partial e_t}{\partial x_i}
=
-\mathbf{R}_{ij}^{\top}\mathbf{R}_i
=
\frac{\partial e_t}{\partial y_i}
$$

### With respect to $\theta_i$

$$
\frac{\partial e_t}{\partial \theta_i}
=
\mathbf{R}_{ij}^{\top}
\frac{\partial \mathbf{R}_i^{\top}}{\partial \theta_i}
(\mathbf{t}_j-\mathbf{t}_i)
$$

$$
=
\mathbf{R}_{ij}^{\top}
\begin{bmatrix}
\cos\theta_i & \sin\theta_i \\
-\sin\theta_i & \cos\theta_i
\end{bmatrix}
(\mathbf{t}_j-\mathbf{t}_i)
$$

$$
=
\mathbf{R}_{ij}^{\top}
\begin{bmatrix}
-\sin\theta_i & \cos\theta_i \\
-\cos\theta_i & -\sin\theta_i
\end{bmatrix}
(\mathbf{t}_j-\mathbf{t}_i)
$$

### With respect to $x_j$ and $y_j$

$$
\frac{\partial e_t}{\partial x_j}
=
\mathbf{R}_{ij}^{\top}\mathbf{R}_i
=
\frac{\partial e_t}{\partial y_j}
$$

### With respect to $\theta_j$

$$
\frac{\partial e_t}{\partial \theta_j}=0 \quad (\text{no } \theta_j\text{-related term})
$$
