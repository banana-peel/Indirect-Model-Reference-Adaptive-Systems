**Indirect Model Reference Adaptive Control**

**Abstract:** *This report analyses Indirect Adaptive Control. A basic
theory of adaptive control is provided and then we move on to Indirect
Adaptive control. The Indirect Adaptive Control is divided into the
adaptive observer and the tuner and the controller. The design as well
as stability proofs for adaptive observer are provided. The tuner and
the controller are briefly explained as they are partially covered in
the adaptive observer section. Additionally this type of controller is
simulated on a second order plant which has to follow a first order
reference model. The results are presented for various inputs for
varying parameters of plant.*

**Introduction:** Adaptive control generally refers to the control of
partially known systems. In real life we rarely have fixed parameters
for a given system. These parameters are likely to change in face of
disturbances or change in environment. The main focus of adaptive
control is to design a feedback system such that the plant output tracks
the reference output of the model.\
Interest in adaptive systems generated during the 1950s for autopilot
systems in Aircrafts. The first major breakthrough in this field was by
Whitaker whose 1961 paper became the famous MIT rule. However few of the
major drawbacks of this approach was its difficult analysis and as well
as cases of instability. A modification of this rule using Lyapunovs
method was also presented by Parks.

Currently there are 3 major methods for adaptive control, which are Gain
Scheduling, Self-Tuning Regulators and Model Reference Adaptive Systems.
Self-Tuning Regulators and MRAS are pretty similar. One of the few major
differences between them is the fact that STRs are used for stochastic
process while MRAS are used for deterministic processes. In MRAS there
are 2 categories of adaptive control: Direct Adatptive Control and
Indirect Adaptive Control. In Indirect Adaptive control we estimate the
plant parameters using an adaptive observer. The estimated measurements
are then used by the tuning and the control circuit to generate the
desired input to track the output reference. In Direct Control we
directly adjust the control parameters.

**Indirect Adaptive Control:**\
In Direct Control we control adjust the control parameters in a feedback
loop depending upon the error between plant and the model outputs. In
Indirect control we estimate the plant parameters and then adjust the
control on the basis of these estimates. Here we give an initial basic
overview of indirect adaptive control using a scalar system.

1)  **Identification Problem:**

$$\dot{x_{p}}(t) = \ a_{p}(t)x_{p}(t) + k_{p}(t)u(t)$$

And the reference model is represented as

$$\dot{x_{m}} = \ a_{m}x_{m}(t) + k_{m}r(t)$$

Here when the plant parameter $k_{p}$ is known while the other parameter
i.e. $a_{p}$ is unknown. We can define new parameters

$\theta(t) \triangleq \frac{a_{m} - \hat{a_{p}}(t)}{k_{p}}$ and
$\text{\ k}^{*} \triangleq \frac{k_{m}}{k_{p}}$

$$u\left( t \right) = \ \theta(t)x_{p}\left( t \right) + k^{*}r(t)$$

$$\operatorname{}{\left| x_{p}\left( t \right) - x_{m}\left( t \right) \right| = 0}$$

$$\dot{\hat{x_{p}}} = \ a_{m}\hat{x_{p}} + \left( \hat{a_{p}}\left( t \right) - a_{m} \right)x_{p} + k_{p}u$$

The output identification error and the parameter estimation error is
given as:

$e_{i}\left( t \right) \triangleq x_{p}\left( t \right) - \ \hat{x_{p}}(t)$
and
$\tilde{a_{p}}\left( t \right) \triangleq \hat{a_{p}}\left( t \right) - a_{p}$

$$V(e_{i},\tilde{a_{p}})\  \triangleq \frac{1}{2}\lbrack e_{i}^{2} + {\tilde{a_{p}}}^{2}\rbrack$$

$$\dot{V} = a_{m}e_{i}^{2} - e_{i}\tilde{a_{p}}x_{p} + \tilde{a_{p}}\dot{\tilde{a_{p}}}$$

For stability we requires $\dot{V} \leq 0$. This is only possible when

$\dot{\tilde{a_{p}}} = e_{i}x_{p}$ which gives us
$\dot{V} = a_{m}e_{i}^{2} \leq 0$

$$\dot{\hat{x_{p}}} = \ a_{m}\hat{x_{p}}(t) + k_{m}r(t)$$

$$\dot{x_{m}} = \ a_{m}x_{m}(t) + k_{m}r(t)$$

$$\hat{x_{p}}\left( t \right) = x_{m}\left( t \right)\ \ \ \ \ \ \ \forall\ t \geq t_{0}\ $$

As $x_{m}$ is uniformly bounded we get $\hat{x_{p}}$ is uniformly
bounded. And as $x_{p} = \hat{x_{p}} + e_{i}$, $x_{p}$ is also bounded.
Hence $e_{i}\left( t \right)$ is also uniformly bounded. Now
$e_{i} \in \mathcal{L}^{2}$ therefore

$$\operatorname{}{\left| x_{p}\left( t \right) - \hat{x_{p}}\left( t \right) \right| = \operatorname{}{\left| x_{p}\left( t \right) - x_{m}\left( t \right) \right| =}0}$$

$$\dot{\tilde{a_{p}}} = \dot{\hat{a_{p}}} = e_{i}x_{p}$$

The proofs described above was for scalar equations. However they can
readily be extended to vector equations. Assuming that all the states in
the system can be accessed we can write the plant as:

$$\dot{x_{p}}(t) = \ A_{p}(t)x_{p}(t) + B_{p}(t)u(t)$$

Where $A_{p}$,$\ B_{p}$ are unknown and
$A_{p} \in \mathbf{R}^{n \times n}$ and
$B_{p} \in \mathbf{R}^{n \times p}$. Here we assume that $A_{p}$ is
asymptotically stable and u is bounded. This assumption is valid as we
are trying to solve an identification problem. When we have an unstable
plant in a control problem, for a properly designed system, the
controller and plant together will form an asymptotically stable
system.\
The estimator used in this case is, analogous to the scalar case, of the
form

$$\dot{\hat{x_{p}}} = \ A_{m}\hat{x_{p}} + \left( \hat{A_{p}}\left( t \right) - A_{m} \right)x_{p} + \hat{B_{p}}u$$

Where, $A_{m}$ is a stable (nxn) matrix. We adjust $\hat{A_{p}}$ and
$\hat{B_{p}}$ to reduce the identifier error to zero. The state errors
and the parameter errors are defined as

$e_{i}\left( t \right) \triangleq x_{p}\left( t \right) - \ \hat{x_{p}}(t)$
, $\Phi\left( t \right) \triangleq \hat{A_{p}}\left( t \right) - A_{p}$,
$\Psi\left( t \right) \triangleq \hat{B_{p}}\left( t \right) - B_{p}$

The error equation is given by

$$\dot{e_{i}}\left( t \right) = A_{m}e_{i}\left( t \right) + \Phi\left( t \right)x_{p}\left( t \right) + \ \Psi\left( t \right)u(t)$$

Now the objective is to adjust $\hat{A_{p}}(t)$ and $\hat{B_{p}}(t)$ so
that $e_{i}\left( t \right)$, $\Phi\left( t \right)$ and
$\Psi\left( t \right)$ tend to zero as $t \rightarrow \infty$. We choose
the adaptive laws by solving Lyapunov’s equation.

$$V\left( e_{i},\Phi,\Psi \right) \triangleq {e_{i}}^{T}Pe_{i} + Tr(\Phi^{T}\Phi + \Psi^{T}\Psi)$$

Solving the equations like in the scalar case using the adaptive laws

$$\dot{\hat{A_{p}}} = \dot{\Phi}(t) = {- Pe}_{i}x_{p}^{T}(t)$$

$$\dot{\hat{B_{p}}} = \dot{\Psi}(t) = {- Pe}_{i}u^{T}(t)$$

We get

$$\dot{V} = - e_{i}^{T}Q_{0}e_{i} \leq 0$$

**3) Adaptive Observers:**\
The case we considered earlier we assumed that we have all the states
available for measurement. However this is not necessarily always true.
It might be expensive to measure states or sometimes the states may be
simply unmeasurable. Hence we would like to estimate the states using
only input and output and then design a controller to track the output
of the reference model.

**Representation:**

Consider a n^th^ order Single Input Single Output LTI system. It can be
represented as:

$$\dot{x_{p}} = \ A_{p}x_{p} + bu$$

$$y_{p} = h^{T}x_{p}$$

We can describe an input-output equivalent to the LTI system as

$$\dot{x_{1}} = \  - \lambda x_{1} + \theta^{T}\omega$$

$$\dot{\omega_{1}} = \ \Lambda\omega_{1} + lu$$

$$\dot{\omega_{2}} = \ \Lambda\omega_{2} + lu$$

$$y_{p} = \ x_{1}$$

We define θ as the control vector

$\theta \triangleq \begin{bmatrix}
c_{0} \\
\overset{\overline{}}{c} \\
d \\
\overset{\overline{}}{d} \\
\end{bmatrix}$ and $\omega \triangleq \begin{bmatrix}
u \\
\omega_{1} \\
y_{p} \\
\omega_{2} \\
\end{bmatrix}$

Here 𝜆 &gt;0 is a scalar, (Λ,l)is controllable and Λ is a (n-1)x(n-1)
asymptotically stable matrix.

![](media/image1.jpeg)

Defining the vectors as

![](media/image2.jpeg)

The transfer function from u to just before the summation block is given
by

![](media/image3.jpeg)

Here P(s) is a (n-1)^th^ degree polynomial and R(s) is the
characteristic polynomial of the asymptotically stable matrix Λ.
Similarly the transfer function from $y_{p}$ to
$\theta_{2}^{T}\overset{\overline{}}{\omega_{2}}$ is given by

![](media/image4.jpeg)

Here Q(s) is an (n-1)^th^ degree polynomial.\
It is seen that P(s) and Q(s) are dependent upon the elements of
$\theta_{1}\ $and $\theta_{2}$ respectively. The transfer function from
u to the output $y_{p}$ can be expressed as

![](media/image5.jpeg)

Thus any LTI system can be represented as shown above. The parameter
vector $\theta_{1}$is determined by the zeroes of the transfer function
while $\theta_{2}$ is determined by the poles. The adaptive observer in
this form is called the parallel observer and is represented by the
equations:

![](media/image6.jpeg)

The adaptive observer shown above is the parallel observer. However the
observer that we will be using for our simulation is slightly different
and is called a Series Parallel Observer. The difference between this
and the parallel observer is that it makes use of the plant output
$y_{p}$ instead of the estimated output ${\hat{y}}_{p}$

![](media/image7.jpeg)

![](media/image8.jpeg)

Series Parallel adaptive observer

Here the parameters are given as:

![](media/image9.jpeg)

We adjust $\hat{\theta}(t)$ such that all the signals remain bounded
while the error between the plant and the observer output tends to zero.
Doing stability analysis using Lyapunov’s theorm it can be found that
the required adaptive law is

$$\dot{\hat{\theta}} = - e_{1}\hat{\omega}$$

**4) Tuner and Controller:**\
The function of the tuner and the controller is to make the system
perfectly track the output reference asymptotically. The tuner tunes the
parameters θ of the controller while the controller adjusts the input to
ensure output tracking.

It can be shown that if the system is represented as:

![](media/image10.jpeg)

Where Λ is an asymptotically stable matrix and
$\lambda\left( s \right) = det\lbrack sI - \Lambda\rbrack$\
For constant values of θ the overall transfer function can be written
as:

![](media/image11.jpeg)

Where 𝜆(s) is a monic polynomial of degree (n-1) and C(s) and D(s) are
polynomials of degree (n-2) and (n-1) respectively. $\theta_{1}$
determines the coefficients of C(s) while $\theta_{0}$ and $\theta_{2}$
determine the coefficients of D(s)

If we define $C^{*}(s)$ and $D^{*}(s)$ as

![](media/image12.jpeg)

And let $\lambda\left( s \right) = \ Z_{m}(s)$ (this can be done if the
relative degree of the reference model is 1)\
Then $\theta^{*}$ exists such that

![](media/image13.jpeg)

The transfer function in this case becomes:

![](media/image14.jpeg)

Thus if the system is represented in the given form we can create
controllers such that they transfer function of the controller and plant
will be the same as the transfer function of the reference model.

**5) Simulation:** ![](media/image15.jpeg){width="6.495833333333334in"
height="2.8159722222222223in"}\
The final control algorithm combines the observer, tuner and controller
described previously. The structure is shown above

*Assumptions:*\
1) The relative degree of W~p~ and W~m~ is one.\
2) k~p~=k~m~=1 i.e the numerator and denominator of both polynomials
W~p~ and W~m~ is monic.\
3) The numerator of both W~p~ and W~m~ is also Hurwitz.

*Adaptive Observer:*

![](media/image16.jpeg)

Here the adaptive law to minimize the Identification error is

$$\dot{\hat{\overset{\overline{}}{p}}} = - e_{1}\overset{\overline{}}{\omega_{0}}$$

The adaptive controller is given by:

$$u = r + {\hat{\overset{\overline{}}{p}}}^{T}(t)\ \overset{\overline{}}{\omega_{0}}$$

*Parameters:*\
We choose:\
Λ = -1 l = 1 $W_{m}\left( s \right) = \frac{1}{\left( s + 3 \right)}$

$$S = \{ - 0.6 \leq a_{1} \leq 3.4, - 2 \leq a_{0} \leq 2\}$$

*Simulation Results:*

**Square Wave Input:**

1)  Unstable and oscillatory Plant: $a_{1} = - 0.5$ and $a_{0} = 2$

    ![](media/image17.jpeg)

2)  Unstable and nonoscillatory plant: $a_{1} = 0.5$ and $a_{0} = - 2$

    ![](media/image18.jpeg)

3)  Stable and nonoscillatory plant: $a_{1} = 3.4$ and $a_{0} = 2$

    ![](media/image19.jpeg)

**Sinusoidal Input:**

1)  Sinusoidal input to an unstable and oscillatory plant:
    $a_{1} = - 0.5$ and $a_{0} = 2$

    ![](media/image20.jpeg)

2)  Low frequency sinusoidal input to an unstable and oscillatory plant:

    ![](media/image21.jpeg)

3)  Combination of Low and High frequency inputs to an unstable and
    oscillatory plant:

    ![](media/image22.jpeg)

**Cosntant Input:**

Constant Input to an unstable and oscillatory plant:

![](media/image23.jpeg)

**Simulation Model:**

![](media/image24.jpeg){width="6.495833333333334in"
height="3.4319444444444445in"}

**References:**

\[1\] K. S. Narendra and J. Balakrishnan, “Improving transient response
of adaptive control systems using multiple models and switching,” *IEEE
Transactions on Automatic Control*, vol. 39, no. 9, pp. 1861–1866, Sep.
1994.

\[2\] R. H. Middleton, G. C. Goodwin, D. J. Hill, and D. Q. Mayne,
“Design issues in adaptive control,” *IEEE Transactions on Automatic
Control*, vol. 33, no. 1, pp. 50–58, Jan. 1988.

\[3\] K. S. Narendra and J. Balakrishnan, “Adaptive control using
multiple models,” *IEEE Transactions on Automatic Control*, vol. 42, no.
2, pp. 171–187, Feb. 1997.

\[4\] P. Parks, “Liapunov redesign of model reference adaptive control
systems,” *IEEE Transactions on Automatic Control*, vol. 11, no. 3, pp.
362–367, Jul. 1966.

\[5\] C. Hang and P. Parks, “Comparative studies of model reference
adaptive control systems,” *IEEE Transactions on Automatic Control*,
vol. 18, no. 5, pp. 419–428, Oct. 1973.

\[6\] K. S. Narendra and A. M. Annaswamy, Stable Adaptive Systems.

Englewood Cliffs, NJ: Prentice-Hall, 1989.
