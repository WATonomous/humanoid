# Inverse kinematics: `fingertip_ik.py`

We implemented **iterative Jacobian IK with damped least squares (DLS)** to take in the 5 target fingertip position and outputting the joint angle that will make it happen. It utilize the Jacobians and body positions from MuJoCo for computation, and Isaac Sim for visualization.

## Algorithm

Joint Position is $\mathbf{q} \in \mathbb{R}^{n_v}$ (where $n_v = 21$, representing the 21 DOF, specfically `model.nv` is the dimension of the joint velocity). Targets are world positions for the five fingertip bodies (`targets` dict). **Only fingertip positions** are constrained (`jacp` from `mj_jacBody`; `jacr`, the fingertip orientaton, is unused).

### Iteration loop

Repeat up to `max_iter` times / return early if error lower than `tol` tolerance:

1. **`mj_forward(model, data)`** — compute forward kinematics with mujoco function
2. **Build error $\mathbf{e}$** — for each fingertip that has a target, compute its error, `err = target - pos`; concatenate into one vector.

   For fingertip $i$:

   $$
   \mathbf{e}_i = \mathbf{p}_i^{\ast} - \mathbf{p}_i(\mathbf{q}) \in \mathbb{R}^3.
   $$

   Stacking $m$ fingertips:

   $$
   \mathbf{e} =
   \begin{bmatrix}
   \mathbf{e}_1 \\ \vdots \\ \mathbf{e}_m
   \end{bmatrix}
   \in \mathbb{R}^{3m}.
   $$
3. **Convergence check** — if $\max |\mathbf{e}| < \text{tol}$, exit early successfully.
4. **Build Jacobian `J`** — for each such fingertip, `mj_jacBody` → translation Jacobian `jacp`; `vstack` into `J`.

   Translation-only linearization for fingertip $i$:

   $$
   \mathrm{d}\mathbf{p}_i \approx \mathbf{J}_i\,\mathrm{d}\mathbf{q},
   \qquad
   \mathbf{J}_i \in \mathbb{R}^{3 \times n_v}.
   $$

   Stacking:

   $$
   \mathbf{J} =
   \begin{bmatrix}
   \mathbf{J}_1 \\ \vdots \\ \mathbf{J}_m
   \end{bmatrix}
   \in \mathbb{R}^{3m \times n_v}.
   $$

5. **Damped least squares** — solve for a joint increment `dq` using damped least squares with $\lambda =$ `damping`.

   Compute $\Delta\mathbf{q}$ that minimizes (**formalizing the optimization problem**)

   $$
   \min_{\Delta\mathbf{q}} \;
   \bigl\|\mathbf{J}\,\Delta\mathbf{q} - \mathbf{e}\bigr\|_2^2
   + \lambda\bigl\|\Delta\mathbf{q}\bigr\|_2^2.
   $$

   The minimizer satisfies the normal equations (The **actual equation that we need to solve**)

   $$
   \bigl(\mathbf{J}^{\mathsf T}\mathbf{J} + \lambda \mathbf{I}\bigr)\,\Delta\mathbf{q}
   = \mathbf{J}^{\mathsf T}\mathbf{e}.
   $$
6. **Update** — `qpos[:nv] += step * dq` (i.e. $\mathbf{q} \leftarrow \mathbf{q} + \alpha\,\Delta\mathbf{q}$, $\alpha =$ `step`).
7. **Joint limits** — if `clip_limits`, run `clip_to_joint_limits`, enforce joint limit

**Return value**: new joint pos, boolean on whether it successfully converge with error under `tol`, max error

**Defaults:** `damping=1e-4`, `step=0.5`, `max_iter=200`, `tol=1e-4`, `clip_limits=True`.