# Pose Graph Edge Error Model

state：

```
[x, y, θ]^T
```

edge measurement：

```
[t_ij, R_ij]  ->  relative pose from node i to node j
```

---

## Definition

Error of edge i -> j :

```
e_ij = [ R_ij^T ( R_i^T (t_j - t_i) - t_ij ) ]_{2x1}
```

```
e_ij = [ e_t
         e_o ]
```

where:

* `e_t` : translational error
* `e_o` : rotational error

---

## Partial Derivatives

### With respect to node i: x_i, y_i

```
∂e_t / ∂x_i = - R_ij^T R_i
∂e_t / ∂y_i = - R_ij^T R_i
```

### With respect to node i: θ_i

```
∂e_t / ∂θ_i = R_ij^T ∂R_i^T / ∂θ_i * (t_j - t_i)
```

```
∂R_i^T / ∂θ_i = [ -sinθ_i   cosθ_i
                  -cosθ_i  -sinθ_i ]
```

```
∴ ∂e_t / ∂θ_i = R_ij^T [ -sinθ_i   cosθ_i
                          -cosθ_i  -sinθ_i ] (t_j - t_i)
```

### With respect to node j: x_j, y_j

```
∂e_t / ∂x_j = R_ij^T R_i
∂e_t / ∂y_j = R_ij^T R_i
```

### With respect to node j: θ_j

```
∂e_t / ∂θ_j = 0    # no θ_j-related term
```
