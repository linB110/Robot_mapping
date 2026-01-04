# 公式

e_{ij} =
[
    R_{ij}^T (R_i^T (t_j - t_i) - t_{ij})
]_{2x1}
=
[
    e_t
    e_0
]

# 偏導數公式

∂e_t/∂x_i = - R_{ij}^T R_i = ∂e_t/∂y_i

∂e_t/∂θ_i = R_{ij}^T ∂R_i^T/∂θ_i (t_j - t_i)
           = R_{ij}^T [
               cosθ_i   sinθ_i
              -sinθ_i   cosθ_i
             ] (t_j - t_i)
           = R_{ij}^T [
              -sinθ_i   cosθ_i
              -cosθ_i  -sinθ_i
             ] (t_j - t_i)

∂e_t/∂x_j = R_{ij}^T R_i = ∂e_t/∂y_j

∂e_t/∂θ_j = 0   (no θ_j-related term)
