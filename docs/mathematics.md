# Mathematical foundation of the supplied implementation

[README](../README.md) · [Source](../firmware/gimbal_stabilizer/gimbal_stabilizer.ino)

All equations below describe the supplied code, including its approximations. Angles inside trigonometric functions and rotation vectors are radians. Serial values, extracted attitudes, joint solutions and servo commands are degrees. Indices of matrix elements are zero-based when matching the source.

## 1. Coordinates and calibration

Let `R` map a sensor's Forward–Right–Down (FRD) frame into its estimated world frame. World down is `e3 = (0,0,1)ᵀ`. Each IMU has its own matrix; initial heading is set implicitly to zero and is not externally observed.

For sensor `i` and component `j`, the calibrated acceleration is:

$$a_{c,ij}=(a_{raw,ij}-b_{ij})s_{ij}.$$

The source applies scale after bias subtraction. Gyro bias is the mean of 500 stationary raw samples. Mappings to FRD are:

$$M_P=\begin{bmatrix}-1&0&0\\0&1&0\\0&0&-1\end{bmatrix},\qquad
M_B=\begin{bmatrix}0&-1&0\\-1&0&0\\0&0&-1\end{bmatrix}.$$

Then `a=M_i a_c` and `gyroFRD=M_i(g_raw−gyroBias)`. The gyro state remains in °/s for feedback; a copy is converted to rad/s for integration. Both mappings have determinant +1, but that does not validate their alignment with the hardware.

## 2. Initialization and Euler-angle extraction

Initialization accepts calibrated acceleration magnitude in the inclusive interval `[0.85,1.15] g`. It calculates:

$$\phi=\operatorname{atan2}(-a_y,-a_z),\qquad
\theta=\operatorname{atan2}(a_x,\sqrt{a_y^2+a_z^2}),\qquad
R=R_y(\theta)R_x(\phi).$$

The elementary rotations are:

$$R_x(r)=\begin{bmatrix}1&0&0\\0&\cos r&-\sin r\\0&\sin r&\cos r\end{bmatrix},\qquad
R_y(p)=\begin{bmatrix}\cos p&0&\sin p\\0&1&0\\-\sin p&0&\cos p\end{bmatrix}.$$

`readAngles()` extracts roll and pitch from any updated `R`:

$$\phi=\operatorname{atan2}(R_{21},R_{22}),\qquad
\theta=\arcsin(\operatorname{clamp}(-R_{20},-1,1)).$$

Multiply by `RAD_TO_DEG` for stored angles. Pitch is constrained by this representation to ±90°; Euler behavior becomes singular at vertical pitch. The matrix representation does not remove singularities in the later Euler extraction and feedback.

## 3. Gravity correction and gyro integration

The predicted world-down direction in sensor coordinates is:

$$\hat d=R^T e_3.$$

At rest the accelerometer measures specific force opposite down, so the correction uses `d_m=−a/||a||`. Only when `0.80 < ||a|| < 1.20 g`, the source computes:

$$\epsilon=d_m\times\hat d,\qquad
\omega=\omega_{gyro}+2.0\,\epsilon.$$

Outside that interval it uses gyro integration without accelerometer correction; automatic control is not disabled solely because of acceleration magnitude. This proportional correction does not update the estimated gyro bias online. It cannot distinguish gravity from all linear accelerations, including ones whose resultant magnitude stays near 1 g.

For `v=ω dt`, let `K=[v]×` and `t=||v||`. `rotVector()` implements Rodrigues' formula:

$$\Delta R=I+A K+B K^2,\qquad A=\frac{\sin t}{t},\quad B=\frac{1-\cos t}{t^2}.$$

For `t < 0.001`, it uses `A=1−t²/6` and `B=1/2−t²/24`. The update is **right-multiplied**, consistent with body-frame rates:

$$R_{k+1}=R_k\Delta R.$$

There is no explicit matrix re-orthonormalization. Yaw remains unobservable from gravity alone. The controller uses gravity vectors rather than the relative yaw of the two drifting world frames.

## 4. Desired gravity vector

For requested sensor roll/pitch `(r,p)`, `downVectorAt()` returns:

$$h=\begin{bmatrix}-\sin p\\\sin r\cos p\\\cos r\cos p\end{bmatrix}.$$

This is the third row of `Ry(p) Rx(r)`, transposed, and is consistent with `readAngles()`. The platform mounting offsets are added to user target angles before constructing `h`. Both offsets are zero in the supplied sketch. Additive Euler offsets are a limited alignment model, not a general three-dimensional mounting calibration.

## 5. Coupled geometric inverse kinematics

The ideal nested mechanism obeys:

$$g=R_x(q_r)R_y(q_p)h,$$

where `g` is normalized base gravity and `h` is the desired platform gravity. Apply `Rx(−qr)` to both sides. Its Y component must equal `h_y`, because `Ry(qp)` leaves Y unchanged:

$$h_y=g_y\cos q_r+g_z\sin q_r.$$

With `ρ=sqrt(g_y²+g_z²)` and `β=atan2(g_y,g_z)`, the selected upright branch is:

$$q_r=\arcsin\left(\operatorname{clamp}\left(\frac{h_y}{\rho},-1,1\right)\right)-\beta.$$

Then calculate `z′=−sin(qr)g_y+cos(qr)g_z`. The remaining X–Z rotation yields:

$$q_p=\operatorname{atan2}(g_x h_z-z' h_x,\;g_x h_x+z' h_z).$$

These are physical model joint angles, converted to degrees on return. There is no independently calibrated inverse actuator model. The code assumes aligned sensor/mechanical frames, orthogonal ideal axes and an upright solution branch.

The source rejects solutions if gravity norm is below 0.8 after the normalization helper, `g_z < 0.20`, `ρ < 0.20`, `h_y/ρ` lies outside `[-1.001,1.001]`, or either solved angle exceeds ±89°. Fractions within the small tolerance are clamped before `asin`; this can introduce a small target mismatch near the boundary. Since normalized `g_z` must be at least 0.2, the model excludes total base tilt beyond approximately `acos(0.2)=78.46°` from world down, independently of reference-relative roll/pitch checks. This is a mathematical gate, not a demonstrated operating envelope.

## 6. Reference, feedforward and feedback

At startup and successful `z`, the code records base Euler references, the measured platform attitude, current servo commands and geometric reference angles for the **current platform down vector**. `z` changes the desired target to that captured attitude minus mounting offsets; it does not mean “make level.”

At each control update, two solves use the current base gravity:

- `q_hold`: joint solution for the platform attitude captured at the reference.
- `q_want`: joint solution for the current user target, including offsets.

For each active axis, with wrapped degree differences `Δ(a,b)`:

$$e=\Delta(\alpha_{platform},\alpha_{target}+\alpha_{offset}),$$
$$u^*=u_{ref}+0.85\,\Delta(q_{hold},q_{ref})+\Delta(q_{want},q_{hold})-0.30\,e-0.050\,\omega_{body}.$$

Base-motion compensation is scaled by 0.85; deliberate target changes have geometric gain 1.0. Roll damping uses platform FRD X rate, pitch damping uses Y rate. This is PD-style attitude feedback with body-rate damping, not the exact time derivative of Euler error at arbitrary combined attitude. The reference command is the command state, not an encoder measurement.

The code checks both active `u*` values before moving either servo. If valid, it advances each active command by:

$$u_{k+1}=u_k+\operatorname{clamp}(u^*-u_k,-80\,dt,80\,dt).$$

Finally `Servo.write((int)(u+0.5))` quantizes the command to integer degrees. `0` and initial centering bypass this automatic slew limit. Roll goals must be in `[45,180]`, pitch goals in `[15,165]`; failure latches both automatic axes in HOLD rather than saturating the goal.
