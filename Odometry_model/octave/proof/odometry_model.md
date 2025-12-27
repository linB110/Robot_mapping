# Odometry Model

state：

```
[x, y, θ]^T  ->  [x̄', ȳ', θ̄']^T
```

---

## definition

```
δ_trans = sqrt((x̄' - x̄)^2 + (ȳ' - ȳ)^2)
```

```
δ_rot1 = atan2(ȳ' - ȳ, x̄' - x̄) - θ̄
```

```
δ_rot2 = θ̄' - θ̄ - δ_rot1
```

---

## problem formulation

given：

```
[δ_rot1, δ_trans, δ_rot2]^T
```

compute：

```
[x̄', ȳ', θ̄']^T
```

initial pose：

```
[x̄, ȳ, θ̄]^T = [0, 0, 0]^T
```

---

## angular relationship

```
θ̄' = δ_rot2 + θ̄ + δ_rot1
```

---

## geometry proof


```
A = x̄' - x̄
B = ȳ' - ȳ
```

```
δ_trans^2 = A^2 + B^2
```

```
tan(δ_rot1 + θ̄) = B / A
```

---

### solving A, B

```
B = A * tan(δ_rot1 + θ̄)
```

```
δ_trans^2 = A^2 (1 + tan^2(δ_rot1 + θ̄))

          By trignometry identity
 
          = A^2 sec^2(δ_rot1 + θ̄)
```

```
A^2 = δ_trans^2 cos^2(δ_rot1 + θ̄)
```

```
A = δ_trans cos(δ_rot1 + θ̄)
B = δ_trans sin(δ_rot1 + θ̄)
```

---

## final relationship of observation and update robot pose

```
x̄' = x̄ + δ_trans cos(δ_rot1 + θ̄)
ȳ' = ȳ + δ_trans sin(δ_rot1 + θ̄)
```
