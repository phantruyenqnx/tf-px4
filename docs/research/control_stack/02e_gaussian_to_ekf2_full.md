# From Gaussian Distribution to EKF2 in PX4 — A Complete Derivation

> This document builds the full chain without skipping steps:
> Gaussian distribution from first principles → Kalman Filter → Extended KF → Error-State EKF → PX4 EKF2 implementation.
> Every formula is derived, every code path is traced.
> Prerequisites: calculus (derivatives, integrals), basic probability (pdf, expectation), some linear algebra.

---

## CHAPTER 1 — The Problem That Created the Gaussian

### 1.1 The Original Question (circa 1800)

Astronomers in the 18th century faced a frustrating problem:

> You measure the position of a star ten times with the same telescope.
> You get ten slightly different numbers. Which one is the true position?

| Measurement | Value |
|---|---|
| $z_1$ | 47.21° |
| $z_2$ | 47.18° |
| $z_3$ | 47.25° |
| $z_4$ | 47.19° |
| ... | ... |
| $z_n$ | 47.22° |

Errors come from atmospheric distortion, instrument imprecision, observer's eye, vibration — each source small and independent.

**The intuitive answer:** take the arithmetic mean $\bar{z} = \frac{1}{n}\sum z_i$.

But *why*? Is the mean actually the *best* estimator? Best by what criterion?

---

### 1.2 Gauss's Insight: Define "Best" as Least Squares

Carl Friedrich Gauss (1809) proposed:

> The best estimate $\hat{\mu}$ is the one that minimizes the sum of squared errors.

$$\hat{\mu} = \arg\min_{\mu} \sum_{i=1}^{n} (z_i - \mu)^2$$

Taking the derivative and setting to zero:

$$\frac{d}{d\mu} \sum_{i=1}^n (z_i - \mu)^2 = -2\sum_{i=1}^n (z_i - \mu) = 0 \quad\Rightarrow\quad \hat{\mu} = \bar{z}$$

So **the mean minimizes least squares**. But Gauss then asked the deeper question:

> **What probability distribution of errors makes the arithmetic mean also the Maximum Likelihood Estimator?**

This forces us to derive the distribution from first principles — not assume a bell shape.

---

## CHAPTER 2 — Deriving the Gaussian from First Principles

### 2.1 Setup: Maximum Likelihood Estimation

Let each measurement be $z_i = \mu + e_i$, where $e_i$ has unknown pdf $p(e)$.

The **likelihood** of observing the full data set $\{z_1, \ldots, z_n\}$:

$$L(\mu) = \prod_{i=1}^n p(z_i - \mu)$$

Maximize the log-likelihood:

$$\frac{d}{d\mu}\sum_{i=1}^n \ln p(z_i - \mu) = 0 \quad\Rightarrow\quad \sum_{i=1}^n \frac{p'(z_i-\mu)}{p(z_i-\mu)} = 0 \tag{I}$$

---

### 2.2 The Constraint: MLE Must Equal the Mean

We demand that the MLE gives $\hat{\mu} = \bar{z}$ for **any** data set.

At $\mu = \bar{z}$, the residuals $e_i = z_i - \bar{z}$ always satisfy $\sum e_i = 0$. Equation (I) becomes:

$$\sum_{i=1}^n \frac{p'(e_i)}{p(e_i)} = 0 \tag{II}$$

**What $p$ satisfies (II) for any residuals summing to zero?**

If $\dfrac{p'(e)}{p(e)} = c \cdot e$ for some constant $c$, then:

$$\sum_{i=1}^n c\,e_i = c\underbrace{\sum e_i}_{=\,0} = 0 \checkmark$$

So the constraint reduces to the ODE: $\dfrac{p'(e)}{p(e)} = c\,e$, with $c < 0$ (so $p(e) \to 0$ as $|e|\to\infty$).

---

### 2.3 Solving the Differential Equation

$$\frac{d}{de}\ln p(e) = c\,e \quad\Rightarrow\quad \ln p(e) = \frac{c}{2}e^2 + C_1 \quad\Rightarrow\quad p(e) = A\,e^{\,\frac{c}{2}e^2}$$

Substitute $c = -\frac{1}{\sigma^2}$ (giving $c$ a physical name):

$$p(e) = A\cdot\exp\!\left(-\frac{e^2}{2\sigma^2}\right) \tag{III}$$

---

### 2.4 Finding the Normalization Constant A

Every pdf must integrate to 1. We need $\displaystyle\int_{-\infty}^{\infty} \exp\!\left(-\frac{e^2}{2\sigma^2}\right)de$.

**Poisson's trick:** Let $I = \int_{-\infty}^{\infty} e^{-t^2}dt$. Then:

$$I^2 = \int\!\!\int e^{-(t^2+s^2)}dt\,ds \xrightarrow{\text{polar}} 2\pi\int_0^{\infty} r\,e^{-r^2}dr = \pi \quad\Rightarrow\quad I = \sqrt{\pi}$$

Substituting $u = e/(\sigma\sqrt{2})$:

$$\int_{-\infty}^{\infty}\exp\!\left(-\frac{e^2}{2\sigma^2}\right)de = \sigma\sqrt{2\pi}$$

Therefore $A = \dfrac{1}{\sqrt{2\pi\sigma^2}}$, giving the **Gaussian distribution**:

$$\boxed{p(e) = \frac{1}{\sqrt{2\pi\sigma^2}}\exp\!\left(-\frac{e^2}{2\sigma^2}\right)}$$

**Derived** — not assumed — from the requirement that the arithmetic mean is the MLE.

**Mean:** $\mathbb{E}[e] = 0$ (odd integrand over symmetric domain).
**Variance:** $\text{Var}[e] = \sigma^2$ (integration by parts confirms $\sigma^2$ is exactly the variance).

---

## CHAPTER 3 — The Five Properties That Power the Kalman Filter

### 3.0 — Read This First: What "Vector Gaussian" Means

Before the properties, we need to be precise about what a **multivariate (vector) Gaussian** is. Everything in the KF is multivariate, so this section is essential.

---

**A scalar Gaussian** $X\sim\mathcal{N}(\mu,\sigma^2)$ describes one uncertain number. It has:
- Mean $\mu$: the center of the bell curve
- Variance $\sigma^2$: how spread out the bell is (standard deviation $\sigma$)

---

**A vector Gaussian** $\mathbf{x}\sim\mathcal{N}(\boldsymbol{\mu},\mathbf{P})$ describes $n$ uncertain numbers **at the same time**, where each pair of numbers may be correlated with each other.

The state of a drone in 2D (for example) is:

$$\mathbf{x} = \begin{bmatrix}p_x \\ p_y \\ v_x \\ v_y\end{bmatrix} \in\mathbb{R}^{4\times 1}$$

(east position, north position, east velocity, north velocity — 4 numbers in a single column vector)

This vector is uncertain, so it follows a 4-dimensional Gaussian described by:

**Mean vector** $\boldsymbol{\mu}$ — the best guess for each component:

$$\boldsymbol{\mu} = \begin{bmatrix}\mu_{p_x} \\ \mu_{p_y} \\ \mu_{v_x} \\ \mu_{v_y}\end{bmatrix} = \begin{bmatrix}10.0\text{ m} \\ 5.0\text{ m} \\ 2.0\text{ m/s} \\ 0.5\text{ m/s}\end{bmatrix}$$

**Covariance matrix** $\mathbf{P}$ — the $n\times n$ matrix describing the uncertainty:

$$\mathbf{P} = \begin{bmatrix}
P_{11} & P_{12} & P_{13} & P_{14} \\
P_{21} & P_{22} & P_{23} & P_{24} \\
P_{31} & P_{32} & P_{33} & P_{34} \\
P_{41} & P_{42} & P_{43} & P_{44}
\end{bmatrix}$$

where:
- **Diagonal entries** $P_{ii}$ = variance of component $i$ = how uncertain we are about $x_i$ alone
  - $P_{11}$: variance of east position (e.g. $4\text{ m}^2$ → standard deviation 2 m)
  - $P_{22}$: variance of north position
  - etc.
- **Off-diagonal entries** $P_{ij}$ (with $i\neq j$) = covariance of components $i$ and $j$ = how much they move together
  - $P_{12} = P_{21}$: if the drone is east of our estimate, is it also likely north? (correlation between $p_x$ and $p_y$)
  - $P_{13}$: if position is east, is the east velocity likely positive too? (correlation $p_x$–$v_x$)

**$\mathbf{P}$ is always symmetric** ($P_{ij} = P_{ji}$) and **positive semi-definite** (all eigenvalues $\geq 0$).

**Concrete example** of what $\mathbf{P}$ looks like when uncertainties are independent (no correlation):

$$\mathbf{P} = \begin{bmatrix}4 & 0 & 0 & 0 \\ 0 & 9 & 0 & 0 \\ 0 & 0 & 1 & 0 \\ 0 & 0 & 0 & 0.25\end{bmatrix}$$

This says: east position uncertain by $\sigma=2$ m, north by $\sigma=3$ m, east vel by $\sigma=1$ m/s, north vel by $\sigma=0.5$ m/s — and none of them are correlated.

---

**Notation index** — every symbol used in this chapter:

| Symbol | Shape | Meaning |
|---|---|---|
| $x, X$ | scalar | A single uncertain number |
| $\mathbf{x}$ | $n\times 1$ column vector | $n$ uncertain numbers stacked vertically |
| $\mathbf{y}$ | $m\times 1$ column vector | Another random vector (may have different dimension $m$) |
| $\mu, \sigma^2$ | scalars | Mean and variance of a scalar Gaussian |
| $\boldsymbol{\mu}$ | $n\times 1$ | Mean vector of a vector Gaussian |
| $\mathbf{P}$ | $n\times n$ | Covariance matrix of $\mathbf{x}$ |
| $\mathbf{R}$ | $m\times m$ | Covariance matrix of $\mathbf{y}$ or measurement noise |
| $\mathbf{A}$ | $m\times n$ | A constant matrix that transforms $\mathbf{x}$ to $\mathbf{y}$ |
| $\mathbf{H}$ | $m\times n$ | Observation matrix (special name for $\mathbf{A}$ when mapping state to measurement) |
| $\mathbf{A}^\top$ | $n\times m$ | Transpose: rows and columns swapped |
| $\mathbf{A}^{-1}$ | $n\times n$ | Matrix inverse (only square, full-rank matrices) |
| $\mathcal{N}(\mu,\sigma^2)$ | — | Scalar Gaussian distribution |
| $\mathcal{N}(\boldsymbol{\mu},\mathbf{P})$ | — | Vector Gaussian distribution |
| $\mathbb{E}[\mathbf{x}]$ | $n\times 1$ | Expected value = mean of $\mathbf{x}$ |
| $\text{Cov}[\mathbf{x}]$ | $n\times n$ | Covariance matrix of $\mathbf{x}$ |
| $\propto$ | — | "Proportional to": $f\propto g$ means $f = c\cdot g$ for some constant $c$ |
| $\perp$ | — | Statistically independent: knowing one tells you nothing about the other |
| $\square$ | — | End of proof |

---

### Property 1: Linear Transform of a Gaussian is Gaussian

---

#### 1A — Scalar version (one number → one number)

**Setup.** You have one uncertain number $X$. You know it follows a Gaussian:

$$X\sim\mathcal{N}(\mu,\,\sigma^2)$$

meaning: the best guess for $X$ is $\mu$, and the spread is $\sigma^2$ (variance).

You apply a linear function to $X$: multiply by $a$, then add $b$:

$$Y = aX + b$$

where $a$ and $b$ are fixed known constants.

**Claim:**

$$Y \sim \mathcal{N}(a\mu + b,\;\; a^2\sigma^2)$$

The new mean is $a\mu + b$, the new variance is $a^2\sigma^2$.

**Proof — step by step:**

We need to find the pdf of $Y$. The key tool is the **change-of-variables formula**: if $Y = g(X)$ and $g$ is monotone, then:

$$p_Y(y) = p_X\!\left(g^{-1}(y)\right)\cdot\left|\frac{d}{dy}g^{-1}(y)\right|$$

*Reading this formula:* to get the probability density of $Y$ at value $y$, we (1) map $y$ back to the corresponding $x = g^{-1}(y)$, (2) look up $p_X(x)$, and (3) multiply by the stretch factor $|dg^{-1}/dy|$ that accounts for how much the transformation stretches or compresses the axis.

Here $g(x) = ax + b$, so $g^{-1}(y) = \dfrac{y-b}{a}$ and $\dfrac{d}{dy}g^{-1}(y) = \dfrac{1}{a}$.

**Step 1** — Substitute $x = \dfrac{y-b}{a}$ into $p_X$:

$$p_Y(y) = \underbrace{\frac{1}{\sqrt{2\pi\sigma^2}}\exp\!\left(-\frac{\left(\dfrac{y-b}{a}-\mu\right)^2}{2\sigma^2}\right)}_{p_X\!\left(\frac{y-b}{a}\right)}\cdot\frac{1}{|a|}$$

**Step 2** — Simplify what's inside the exponent. Compute $\dfrac{y-b}{a} - \mu$:

$$\frac{y-b}{a} - \mu = \frac{y-b - a\mu}{a} = \frac{y-(a\mu+b)}{a}$$

Let $\mu_Y = a\mu + b$ (the new mean). Then:

$$\left(\frac{y-b}{a} - \mu\right)^2 = \left(\frac{y-\mu_Y}{a}\right)^2 = \frac{(y-\mu_Y)^2}{a^2}$$

**Step 3** — Substitute back:

$$p_Y(y) = \frac{1}{\sqrt{2\pi\sigma^2}}\cdot\frac{1}{|a|}\cdot\exp\!\left(-\frac{(y-\mu_Y)^2}{2a^2\sigma^2}\right)$$

**Step 4** — Tidy up the prefactor. Note $\dfrac{1}{\sqrt{2\pi\sigma^2}}\cdot\dfrac{1}{|a|} = \dfrac{1}{\sqrt{2\pi\,a^2\sigma^2}}$:

$$\boxed{p_Y(y) = \frac{1}{\sqrt{2\pi\,(a^2\sigma^2)}}\exp\!\left(-\frac{(y-\mu_Y)^2}{2\,(a^2\sigma^2)}\right)} \quad\Rightarrow\quad Y\sim\mathcal{N}(a\mu+b,\;a^2\sigma^2)\quad\square$$

**Concrete example:**

Temperature sensor reads $X\sim\mathcal{N}(25°C,\; 4)$. You convert Celsius to Fahrenheit: $Y = 1.8X + 32$.

$$Y\sim\mathcal{N}(1.8\cdot 25 + 32,\;\; 1.8^2\cdot 4) = \mathcal{N}(77°F,\;\; 12.96)$$

Standard deviation in Celsius: $\sigma_X = 2°C$. In Fahrenheit: $\sigma_Y = 1.8\times 2 = 3.6°F = 1.8\cdot\sigma_X$. Makes sense — the scale stretches by a factor of 1.8.

---

#### 1B — Vector version (n numbers → m numbers)

**Setup.** Now you have $n$ uncertain numbers packed into a column vector:

$$\mathbf{x} = \begin{bmatrix}x_1 \\ x_2 \\ \vdots \\ x_n\end{bmatrix} \quad (n\times 1 \text{ column vector})$$

This vector follows an $n$-dimensional Gaussian:

$$\mathbf{x}\sim\mathcal{N}(\boldsymbol{\mu},\,\mathbf{P})$$

where:
- Mean vector $\boldsymbol{\mu} = \begin{bmatrix}\mu_1\\\mu_2\\\vdots\\\mu_n\end{bmatrix}$ — the best guess for each component
- Covariance matrix $\mathbf{P}$ — an $n\times n$ symmetric matrix:

$$\mathbf{P} = \begin{bmatrix}
P_{11} & P_{12} & \cdots & P_{1n} \\
P_{21} & P_{22} & \cdots & P_{2n} \\
\vdots & & \ddots & \vdots \\
P_{n1} & P_{n2} & \cdots & P_{nn}
\end{bmatrix}$$

where $P_{ii} = \text{Var}[x_i]$ (variance of component $i$) and $P_{ij} = \text{Cov}[x_i, x_j]$ for $i\neq j$ (how much $x_i$ and $x_j$ tend to move together).

You apply a **matrix linear transform** $\mathbf{A}$ (an $m\times n$ matrix of constants) to get an $m$-dimensional output:

$$\mathbf{y} = \mathbf{A}\mathbf{x}$$

Written out, the matrix multiplication looks like:

$$\begin{bmatrix}y_1\\y_2\\\vdots\\y_m\end{bmatrix} = \begin{bmatrix}A_{11}&A_{12}&\cdots&A_{1n}\\A_{21}&A_{22}&\cdots&A_{2n}\\\vdots&&\ddots&\vdots\\A_{m1}&A_{m2}&\cdots&A_{mn}\end{bmatrix}\begin{bmatrix}x_1\\x_2\\\vdots\\x_n\end{bmatrix}$$

so $y_i = A_{i1}x_1 + A_{i2}x_2 + \cdots + A_{in}x_n$ — each output is a linear combination of the inputs.

**Claim:**

$$\mathbf{y}\sim\mathcal{N}(\mathbf{A}\boldsymbol{\mu},\;\mathbf{A}\mathbf{P}\mathbf{A}^\top)$$

where $\mathbf{A}^\top$ is the $n\times m$ **transpose** of $\mathbf{A}$ (rows and columns swapped).

**Proof — step by step:**

**Step 1 — Find the mean of $\mathbf{y}$.**

$$\mathbb{E}[\mathbf{y}] = \mathbb{E}[\mathbf{A}\mathbf{x}]$$

$\mathbf{A}$ is a constant matrix — it doesn't change. So:

$$= \mathbf{A}\,\mathbb{E}[\mathbf{x}] = \mathbf{A}\boldsymbol{\mu}$$

Written out entry by entry: $\mathbb{E}[y_i] = \sum_j A_{ij}\mathbb{E}[x_j] = \sum_j A_{ij}\mu_j$ — the $i$-th entry of $\mathbf{A}\boldsymbol{\mu}$. ✓

**Step 2 — Find the covariance matrix of $\mathbf{y}$.**

The covariance matrix of $\mathbf{y}$ is defined as:

$$\text{Cov}[\mathbf{y}] = \mathbb{E}\!\left[(\mathbf{y}-\mathbb{E}[\mathbf{y}])(\mathbf{y}-\mathbb{E}[\mathbf{y}])^\top\right]$$

*Reading this:* for each pair of output components $(y_i, y_j)$, compute the average of $(y_i - \mu_{y_i})(y_j - \mu_{y_j})$. This gives an $m\times m$ matrix.

Now substitute $\mathbf{y} - \mathbb{E}[\mathbf{y}] = \mathbf{A}\mathbf{x} - \mathbf{A}\boldsymbol{\mu} = \mathbf{A}(\mathbf{x}-\boldsymbol{\mu})$:

$$\text{Cov}[\mathbf{y}] = \mathbb{E}\!\left[\mathbf{A}(\mathbf{x}-\boldsymbol{\mu})\cdot\left(\mathbf{A}(\mathbf{x}-\boldsymbol{\mu})\right)^\top\right]$$

Using the transpose rule $(\mathbf{A}\mathbf{v})^\top = \mathbf{v}^\top\mathbf{A}^\top$:

$$= \mathbb{E}\!\left[\mathbf{A}(\mathbf{x}-\boldsymbol{\mu})(\mathbf{x}-\boldsymbol{\mu})^\top\mathbf{A}^\top\right]$$

Since $\mathbf{A}$ and $\mathbf{A}^\top$ are constants (not random), pull them outside the expectation:

$$= \mathbf{A}\underbrace{\mathbb{E}\!\left[(\mathbf{x}-\boldsymbol{\mu})(\mathbf{x}-\boldsymbol{\mu})^\top\right]}_{\text{definition of }\mathbf{P}}\mathbf{A}^\top = \mathbf{A}\mathbf{P}\mathbf{A}^\top$$

**Step 3 — Verify the result is Gaussian.**

A multivariate Gaussian is uniquely determined by its mean vector and covariance matrix. A linear transform of a Gaussian is always Gaussian (the pdf of $\mathbf{y}$ stays in the bell-curve family). So $\mathbf{y}\sim\mathcal{N}(\mathbf{A}\boldsymbol{\mu},\;\mathbf{A}\mathbf{P}\mathbf{A}^\top)$. $\square$

---

**Worked example with explicit numbers:**

Drone state $\mathbf{x} = \begin{bmatrix}p_x \\ p_y\end{bmatrix}$ (east and north position), with:

$$\boldsymbol{\mu} = \begin{bmatrix}10\\ 5\end{bmatrix}\text{ m}, \qquad \mathbf{P} = \begin{bmatrix}4 & 1 \\ 1 & 9\end{bmatrix}\text{ m}^2$$

($P_{11}=4$: east uncertain by $\sigma=2$ m; $P_{22}=9$: north uncertain by $\sigma=3$ m; $P_{12}=1$: slight positive correlation — when drone is east of estimate, it's also slightly north.)

GPS reports only east position: $\mathbf{A} = [1\quad 0]$ (a $1\times 2$ matrix, picks out $p_x$).

$$\mathbf{y} = \mathbf{A}\mathbf{x} = [1\quad 0]\begin{bmatrix}p_x\\p_y\end{bmatrix} = p_x$$

**New mean:**

$$\mathbf{A}\boldsymbol{\mu} = [1\quad 0]\begin{bmatrix}10\\5\end{bmatrix} = 10 \text{ m}$$

**New covariance:**

$$\mathbf{A}\mathbf{P}\mathbf{A}^\top = [1\quad 0]\begin{bmatrix}4&1\\1&9\end{bmatrix}\begin{bmatrix}1\\0\end{bmatrix}$$

First compute $[1\quad 0]\begin{bmatrix}4&1\\1&9\end{bmatrix} = [4\quad 1]$. Then $[4\quad 1]\begin{bmatrix}1\\0\end{bmatrix} = 4$.

So $y = p_x\sim\mathcal{N}(10,\; 4)$. The GPS reading only carries the east variance ($P_{11}=4$), not the correlation or north variance. Makes sense.

---

**Connection to Kalman Filter:**

In the PREDICT step, the state evolves as $\mathbf{x}_k = \mathbf{F}\mathbf{x}_{k-1}$ (no noise yet). Property 1 with $\mathbf{A}=\mathbf{F}$ gives:

$$\mathbf{x}_k\sim\mathcal{N}\!\left(\mathbf{F}\hat{\mathbf{x}}_{k-1},\;\underbrace{\mathbf{F}\mathbf{P}_{k-1}\mathbf{F}^\top}_{\text{from Property 1}}\right)$$

This is where the $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ term in the KF predict step comes from.

---

### Property 2: Sum of Independent Gaussians is Gaussian

**Statement:** If $X\sim\mathcal{N}(\mu_1,\sigma_1^2)$ and $Y\sim\mathcal{N}(\mu_2,\sigma_2^2)$ are **independent**, then:

$$Z = X + Y \sim \mathcal{N}(\mu_1+\mu_2,\;\sigma_1^2+\sigma_2^2)$$

Means add. Variances add (not standard deviations — variances).

**Why $\sigma_1^2 + \sigma_2^2$ and not something else?**

Before the proof, build intuition. If you measure altitude with error $e_1\sim\mathcal{N}(0,4)$ (std dev 2 m) and then the drone flies, accumulating IMU integration error $e_2\sim\mathcal{N}(0,1)$ (std dev 1 m), the total position error is $e_1 + e_2$. The errors are independent — they don't help cancel each other. So the total uncertainty is larger than either alone: the total variance is $4 + 1 = 5$, giving std dev $\sqrt{5}\approx 2.2$ m. Note that you can't just add standard deviations ($2+1=3$ would be wrong and too pessimistic).

**Proof — using Property 1 directly (no advanced machinery needed):**

**Step 1 — Stack $X$ and $Y$ into a joint vector.**

Since $X$ and $Y$ are independent Gaussians, the vector $\begin{bmatrix}X\\Y\end{bmatrix}$ is a 2D Gaussian:

$$\begin{bmatrix}X\\Y\end{bmatrix}\sim\mathcal{N}\!\left(\begin{bmatrix}\mu_1\\\mu_2\end{bmatrix},\;\begin{bmatrix}\sigma_1^2 & 0\\0&\sigma_2^2\end{bmatrix}\right)$$

*Why is the covariance matrix diagonal?* Off-diagonal entry $P_{12} = \text{Cov}(X,Y) = 0$ because $X$ and $Y$ are **independent** — knowing $X$ tells you nothing about $Y$. Independent variables always have zero covariance.

**Step 2 — Express $Z = X + Y$ as a linear transform.**

$$Z = X + Y = \underbrace{[1\quad 1]}_{\mathbf{A},\;1\times 2}\underbrace{\begin{bmatrix}X\\Y\end{bmatrix}}_{\mathbf{x},\;2\times 1}$$

This is exactly the form $\mathbf{y} = \mathbf{A}\mathbf{x}$ from Property 1, with $m=1$, $n=2$, $\mathbf{A} = [1\quad 1]$.

**Step 3 — Apply Property 1.**

New mean (Property 1: $\mathbf{A}\boldsymbol{\mu}$):

$$\mathbb{E}[Z] = [1\quad 1]\begin{bmatrix}\mu_1\\\mu_2\end{bmatrix} = \mu_1 + \mu_2$$

New variance (Property 1: $\mathbf{A}\mathbf{P}\mathbf{A}^\top$):

$$\text{Var}[Z] = [1\quad 1]\begin{bmatrix}\sigma_1^2&0\\0&\sigma_2^2\end{bmatrix}\begin{bmatrix}1\\1\end{bmatrix}$$

First compute $[1\quad 1]\begin{bmatrix}\sigma_1^2&0\\0&\sigma_2^2\end{bmatrix} = [\sigma_1^2\quad\sigma_2^2]$.

Then $[\sigma_1^2\quad\sigma_2^2]\begin{bmatrix}1\\1\end{bmatrix} = \sigma_1^2 + \sigma_2^2$.

**Step 4 — Conclude.**

Since $Z$ is a linear transform of a Gaussian vector, by Property 1, $Z$ is also Gaussian:

$$Z\sim\mathcal{N}(\mu_1+\mu_2,\;\sigma_1^2+\sigma_2^2)\quad\square$$

**What if X and Y are correlated (not independent)?** Then $\text{Cov}(X,Y) = \rho\sigma_1\sigma_2\neq 0$ and $\mathbf{P} = \begin{bmatrix}\sigma_1^2&\rho\sigma_1\sigma_2\\\rho\sigma_1\sigma_2&\sigma_2^2\end{bmatrix}$. The matrix multiply gives:

$$\text{Var}[Z] = \sigma_1^2 + 2\rho\sigma_1\sigma_2 + \sigma_2^2$$

The extra term $2\rho\sigma_1\sigma_2$ is the covariance contribution. If $\rho>0$ (positively correlated), total variance is larger; if $\rho<0$ (negatively correlated), total variance is smaller. Independence ($\rho=0$) is the "no interaction" case.

**Connection to Kalman Filter:**

The state model is $\mathbf{x}_k = \mathbf{F}\mathbf{x}_{k-1} + \mathbf{w}_k$ where $\mathbf{w}_k\sim\mathcal{N}(\mathbf{0},\mathbf{Q})$ is process noise, independent of $\mathbf{x}_{k-1}$. So the joint vector $\begin{bmatrix}\mathbf{F}\mathbf{x}_{k-1}\\\mathbf{w}_k\end{bmatrix}$ has block-diagonal covariance, and:

$$\mathbf{x}_k = \underbrace{\mathbf{F}\mathbf{x}_{k-1}}_{\sim\mathcal{N}(\mathbf{F}\hat{\mathbf{x}},\,\mathbf{F}\mathbf{P}\mathbf{F}^\top)} + \underbrace{\mathbf{w}_k}_{\sim\mathcal{N}(\mathbf{0},\,\mathbf{Q})}$$

By Property 2: $\mathbf{x}_k\sim\mathcal{N}(\mathbf{F}\hat{\mathbf{x}},\;\mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q})$.

This gives us both terms in the KF predict covariance update: $\mathbf{P}_{k|k-1} = \mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^\top + \mathbf{Q}$.

---

### Property 3: Product of Two Gaussian PDFs is (Proportional to) a Gaussian

This is the **most important property for the KF update step**. The Bayesian update rule says: posterior ∝ prior × likelihood. If both prior and likelihood are Gaussian, the posterior is also Gaussian — and we can compute its parameters analytically.

**Statement:** Given two Gaussian pdfs treated as functions of the same variable $x$:

$$p_1(x) = \mathcal{N}(x;\,\mu_1,\sigma_1^2) \quad\text{and}\quad p_2(x) = \mathcal{N}(x;\,\mu_2,\sigma_2^2)$$

Their product is proportional to another Gaussian:

$$p_1(x)\cdot p_2(x) \propto \mathcal{N}(x;\,\mu_\text{new},\,\sigma_\text{new}^2)$$

where:

$$\sigma_\text{new}^2 = \frac{\sigma_1^2\,\sigma_2^2}{\sigma_1^2+\sigma_2^2} \tag{P3-var}$$

$$\mu_\text{new} = \frac{\mu_1\,\sigma_2^2 + \mu_2\,\sigma_1^2}{\sigma_1^2+\sigma_2^2} \tag{P3-mean}$$

**Step-by-step proof:**

Write out the product explicitly, using $\mathcal{N}(x;\mu,\sigma^2) = \frac{1}{\sqrt{2\pi\sigma^2}}\exp\!\left(-\frac{(x-\mu)^2}{2\sigma^2}\right)$:

$$p_1(x)\cdot p_2(x) = \frac{1}{\sqrt{2\pi\sigma_1^2}}\cdot\frac{1}{\sqrt{2\pi\sigma_2^2}}\cdot\exp\!\left(-\frac{(x-\mu_1)^2}{2\sigma_1^2} - \frac{(x-\mu_2)^2}{2\sigma_2^2}\right)$$

The prefactor is a constant (doesn't depend on $x$). Focus entirely on the exponent, which determines the shape.

**Step 1 — Combine the two fractions in the exponent** by finding a common denominator $2\sigma_1^2\sigma_2^2$:

$$-\frac{(x-\mu_1)^2}{2\sigma_1^2} - \frac{(x-\mu_2)^2}{2\sigma_2^2} = -\frac{\sigma_2^2(x-\mu_1)^2 + \sigma_1^2(x-\mu_2)^2}{2\sigma_1^2\sigma_2^2}$$

**Step 2 — Expand the numerator:**

$$\sigma_2^2(x-\mu_1)^2 + \sigma_1^2(x-\mu_2)^2$$

$$= \sigma_2^2(x^2 - 2\mu_1 x + \mu_1^2) + \sigma_1^2(x^2 - 2\mu_2 x + \mu_2^2)$$

$$= (\sigma_1^2+\sigma_2^2)x^2 - 2(\mu_1\sigma_2^2+\mu_2\sigma_1^2)x + (\mu_1^2\sigma_2^2+\mu_2^2\sigma_1^2)$$

**Step 3 — Complete the square in $x$:**

Factor out $(\sigma_1^2+\sigma_2^2)$ from the first two terms:

$$= (\sigma_1^2+\sigma_2^2)\left[x^2 - 2\cdot\frac{\mu_1\sigma_2^2+\mu_2\sigma_1^2}{\sigma_1^2+\sigma_2^2}\cdot x\right] + (\mu_1^2\sigma_2^2+\mu_2^2\sigma_1^2)$$

The expression inside brackets has the form $x^2 - 2\mu_\text{new}x$. Complete the square:

$$x^2 - 2\mu_\text{new}x = (x-\mu_\text{new})^2 - \mu_\text{new}^2$$

where $\mu_\text{new} = \dfrac{\mu_1\sigma_2^2+\mu_2\sigma_1^2}{\sigma_1^2+\sigma_2^2}$.

So the numerator becomes:

$$(\sigma_1^2+\sigma_2^2)(x-\mu_\text{new})^2 + \underbrace{(\mu_1^2\sigma_2^2+\mu_2^2\sigma_1^2) - (\sigma_1^2+\sigma_2^2)\mu_\text{new}^2}_{\text{constant w.r.t. } x}$$

**Step 4 — Substitute back into the exponent:**

$$\text{exponent} = -\frac{(\sigma_1^2+\sigma_2^2)(x-\mu_\text{new})^2}{2\sigma_1^2\sigma_2^2} + \text{const}$$

$$= -\frac{(x-\mu_\text{new})^2}{2\cdot\dfrac{\sigma_1^2\sigma_2^2}{\sigma_1^2+\sigma_2^2}} + \text{const}$$

$$= -\frac{(x-\mu_\text{new})^2}{2\,\sigma_\text{new}^2} + \text{const}$$

where $\sigma_\text{new}^2 = \dfrac{\sigma_1^2\sigma_2^2}{\sigma_1^2+\sigma_2^2}$.

**Step 5 — Recognize the Gaussian form:**

$$p_1(x)\cdot p_2(x) \propto \exp\!\left(-\frac{(x-\mu_\text{new})^2}{2\sigma_\text{new}^2}\right) \propto \mathcal{N}(x;\,\mu_\text{new},\,\sigma_\text{new}^2) \quad\square$$

---

**Rewriting in Kalman Filter form:**

The formula for $\mu_\text{new}$ can be refactored to expose the Kalman Gain:

$$\mu_\text{new} = \frac{\mu_1\sigma_2^2 + \mu_2\sigma_1^2}{\sigma_1^2+\sigma_2^2}$$

Factor out $\sigma_2^2$ from first term and $\sigma_1^2$ from second:

$$= \mu_1\cdot\frac{\sigma_2^2}{\sigma_1^2+\sigma_2^2} + \mu_2\cdot\frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}$$

Define:

$$K \overset{\text{def}}{=} \frac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}$$

Then $\dfrac{\sigma_2^2}{\sigma_1^2+\sigma_2^2} = 1-K$, so:

$$\mu_\text{new} = \mu_1(1-K) + \mu_2 K = \mu_1 + K(\mu_2 - \mu_1) \tag{KF mean update}$$

And:

$$\sigma_\text{new}^2 = \frac{\sigma_1^2\sigma_2^2}{\sigma_1^2+\sigma_2^2} = \sigma_1^2\cdot\frac{\sigma_2^2}{\sigma_1^2+\sigma_2^2} = \sigma_1^2(1-K) \tag{KF variance update}$$

**This is the entire Kalman Filter update in 1D.** Mapping to KF notation:

| Algebra symbol | KF meaning | Physical interpretation |
|---|---|---|
| $\mu_1$ | $\hat{x}_{k\|k-1}$ | Best estimate before this measurement |
| $\sigma_1^2$ | $P_{k\|k-1}$ | Uncertainty of the prediction |
| $\mu_2$ | $z_k$ | The measurement value |
| $\sigma_2^2$ | $R$ | Measurement noise variance (how noisy is the sensor?) |
| $K = \dfrac{\sigma_1^2}{\sigma_1^2+\sigma_2^2}$ | $K_k$ | Kalman Gain: how much to trust this measurement |
| $\mu_\text{new}$ | $\hat{x}_{k\|k}$ | Updated estimate after fusing measurement |
| $\sigma_\text{new}^2$ | $P_{k\|k}$ | Reduced uncertainty after update |

**Intuition — what does $K$ control?**

- If $\sigma_1^2 \gg \sigma_2^2$ (prediction very uncertain, sensor very accurate): $K\to 1$, so $\mu_\text{new}\approx \mu_2$ — **trust the measurement almost completely**.
- If $\sigma_1^2 \ll \sigma_2^2$ (prediction very confident, sensor very noisy): $K\to 0$, so $\mu_\text{new}\approx \mu_1$ — **trust the prediction almost completely**.
- Equal uncertainty ($\sigma_1^2 = \sigma_2^2$): $K = 0.5$, $\mu_\text{new} = (\mu_1+\mu_2)/2$ — **simple average**.

**Concrete example:**

Altitude filter. IMU predicts 100 m with uncertainty $P = 4\text{ m}^2$. Barometer reads 103 m with noise $R = 1\text{ m}^2$.

$$K = \frac{4}{4+1} = 0.8$$

$$\hat{h} = 100 + 0.8\cdot(103 - 100) = 100 + 2.4 = 102.4\text{ m}$$

$$P_\text{new} = (1-0.8)\cdot 4 = 0.8\text{ m}^2$$

The filter strongly trusts the baro (K = 0.8), moves most of the way toward the baro reading, and the uncertainty drops from 4 to 0.8.

---

### Property 4: Joint Distribution of State and Measurement is Gaussian

**Why do we need this property?**

At each timestep, we have two things: the unknown state $\mathbf{x}$ (what we want to know) and a sensor measurement $\mathbf{y}$ (what we can observe). They are related: $\mathbf{y}$ depends on $\mathbf{x}$ through the measurement model. Property 4 tells us the complete probability description of $(\mathbf{x}, \mathbf{y})$ together as a single Gaussian — this is the prerequisite for Property 5, which extracts the conditional $p(\mathbf{x}\mid\mathbf{y})$.

---

**Complete setup — every variable spelled out:**

**The state** $\mathbf{x}$ is an $n$-dimensional column vector of hidden quantities (position, velocity, orientation, biases, …):

$$\mathbf{x} = \begin{bmatrix}x_1\\x_2\\\vdots\\x_n\end{bmatrix} \;\in\mathbb{R}^n, \qquad \mathbf{x}\sim\mathcal{N}(\boldsymbol{\mu}_x,\,\mathbf{P}_{xx})$$

- $\boldsymbol{\mu}_x$ ($n\times 1$): best guess for the state
- $\mathbf{P}_{xx}$ ($n\times n$): covariance of the state uncertainty

**The measurement** $\mathbf{y}$ is an $m$-dimensional column vector of sensor readings (GPS position, barometer altitude, magnetometer heading, …):

$$\mathbf{y} = \mathbf{H}\mathbf{x} + \mathbf{v}$$

where:
- $\mathbf{H}$ ($m\times n$): the **observation matrix** — maps each state variable to what the sensors measure. For example, if $n=4$ (position $x,y$ + velocity $v_x,v_y$) and $m=2$ (GPS measures only position), then:

$$\mathbf{H} = \begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix} \quad\Rightarrow\quad \mathbf{y} = \begin{bmatrix}1&0&0&0\\0&1&0&0\end{bmatrix}\begin{bmatrix}x\\y\\v_x\\v_y\end{bmatrix} = \begin{bmatrix}x\\y\end{bmatrix}$$

GPS reads position, ignores velocity.

- $\mathbf{v}$ ($m\times 1$): **measurement noise** — random errors of the sensor, independent of the state:

$$\mathbf{v}\sim\mathcal{N}(\mathbf{0},\,\mathbf{R}), \qquad \mathbf{v}\perp\mathbf{x}$$

- $\mathbf{R}$ ($m\times m$): covariance of sensor noise. If sensors are independent of each other, $\mathbf{R}$ is diagonal.

**The joint vector** stacks state and measurement together into a single $(n+m)$-dimensional vector:

$$\begin{bmatrix}\mathbf{x}\\\mathbf{y}\end{bmatrix} = \begin{bmatrix}x_1\\\vdots\\x_n\\y_1\\\vdots\\y_m\end{bmatrix} \;\in\mathbb{R}^{n+m}$$

---

**Claim:** This joint vector is Gaussian:

$$\begin{bmatrix}\mathbf{x}\\\mathbf{y}\end{bmatrix}\sim\mathcal{N}\!\left(\begin{bmatrix}\boldsymbol{\mu}_x\\\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix},\;\underbrace{\begin{bmatrix}\mathbf{P}_{xx} & \mathbf{P}_{xx}\mathbf{H}^\top\\\mathbf{H}\mathbf{P}_{xx} & \mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}\end{bmatrix}}_{\boldsymbol{\Sigma}_\text{joint},\;(n+m)\times(n+m)}\right)$$

The joint covariance is a $(n+m)\times(n+m)$ block matrix with four blocks:

$$\boldsymbol{\Sigma}_\text{joint} = \begin{bmatrix}\underbrace{\mathbf{P}_{xx}}_{n\times n} & \underbrace{\mathbf{P}_{xx}\mathbf{H}^\top}_{n\times m}\\\underbrace{\mathbf{H}\mathbf{P}_{xx}}_{m\times n} & \underbrace{\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}}_{m\times m}\end{bmatrix}$$

---

**Step-by-step proof:**

**Step 1 — Find the mean of $\mathbf{y}$.**

$$\mathbb{E}[\mathbf{y}] = \mathbb{E}[\mathbf{H}\mathbf{x}+\mathbf{v}] = \mathbf{H}\underbrace{\mathbb{E}[\mathbf{x}]}_{\boldsymbol{\mu}_x} + \underbrace{\mathbb{E}[\mathbf{v}]}_{\mathbf{0}} = \mathbf{H}\boldsymbol{\mu}_x$$

So the joint mean vector is $\begin{bmatrix}\boldsymbol{\mu}_x\\\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix}$.

**Step 2 — Compute Block (1,1): $\text{Cov}(\mathbf{x},\mathbf{x})$.**

By definition, this is just $\mathbf{P}_{xx}$. Nothing to compute.

**Step 3 — Compute Block (2,2): $\text{Cov}(\mathbf{y},\mathbf{y})$.**

First find the deviation of $\mathbf{y}$ from its mean:

$$\mathbf{y} - \mathbb{E}[\mathbf{y}] = \mathbf{H}\mathbf{x} + \mathbf{v} - \mathbf{H}\boldsymbol{\mu}_x = \underbrace{\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)}_{\text{state uncertainty projected to sensor}} + \underbrace{\mathbf{v}}_{\text{sensor noise}}$$

Now compute:

$$\text{Cov}(\mathbf{y},\mathbf{y}) = \mathbb{E}\!\left[\left(\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)+\mathbf{v}\right)\left(\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)+\mathbf{v}\right)^\top\right]$$

Expand ($(a+b)(a+b)^\top = aa^\top + ab^\top + ba^\top + bb^\top$):

$$= \mathbb{E}\!\left[\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)(\mathbf{x}-\boldsymbol{\mu}_x)^\top\mathbf{H}^\top\right] + \mathbb{E}\!\left[\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)\mathbf{v}^\top\right] + \mathbb{E}\!\left[\mathbf{v}(\mathbf{x}-\boldsymbol{\mu}_x)^\top\mathbf{H}^\top\right] + \mathbb{E}\!\left[\mathbf{v}\mathbf{v}^\top\right]$$

Since $\mathbf{x}\perp\mathbf{v}$ (state and sensor noise are independent):

$$\mathbb{E}\!\left[(\mathbf{x}-\boldsymbol{\mu}_x)\mathbf{v}^\top\right] = \mathbb{E}[\mathbf{x}-\boldsymbol{\mu}_x]\cdot\mathbb{E}[\mathbf{v}^\top] = \mathbf{0}\cdot\mathbf{0}^\top = \mathbf{0}$$

So the cross terms vanish, leaving:

$$\text{Cov}(\mathbf{y},\mathbf{y}) = \mathbf{H}\underbrace{\mathbb{E}[(\mathbf{x}-\boldsymbol{\mu}_x)(\mathbf{x}-\boldsymbol{\mu}_x)^\top]}_{\mathbf{P}_{xx}}\mathbf{H}^\top + \underbrace{\mathbb{E}[\mathbf{v}\mathbf{v}^\top]}_{\mathbf{R}} = \boxed{\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top + \mathbf{R}}$$

This block has a name: $\mathbf{S} = \mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top + \mathbf{R}$, called the **innovation covariance**.

*Interpretation:* The total uncertainty in the sensor reading comes from two sources: (1) state uncertainty projected through $\mathbf{H}$ (term $\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top$) and (2) sensor noise $\mathbf{R}$. Both contribute.

**Step 4 — Compute Block (1,2): $\text{Cov}(\mathbf{x},\mathbf{y})$.**

This $n\times m$ block measures how much the state uncertainty and the measurement uncertainty are correlated — if $\mathbf{x}$ is above its mean, does $\mathbf{y}$ tend to be above its mean too?

$$\text{Cov}(\mathbf{x},\mathbf{y}) = \mathbb{E}\!\left[(\mathbf{x}-\boldsymbol{\mu}_x)\left(\mathbf{H}(\mathbf{x}-\boldsymbol{\mu}_x)+\mathbf{v}\right)^\top\right]$$

$$= \mathbb{E}\!\left[(\mathbf{x}-\boldsymbol{\mu}_x)(\mathbf{x}-\boldsymbol{\mu}_x)^\top\right]\mathbf{H}^\top + \underbrace{\mathbb{E}\!\left[(\mathbf{x}-\boldsymbol{\mu}_x)\mathbf{v}^\top\right]}_{=\,\mathbf{0},\;\mathbf{x}\perp\mathbf{v}}$$

$$= \mathbf{P}_{xx}\mathbf{H}^\top \quad =: \quad \mathbf{P}_{xy}$$

**Step 5 — Block (2,1) follows by symmetry.**

$\text{Cov}(\mathbf{y},\mathbf{x}) = \mathbf{P}_{xy}^\top = \mathbf{H}\mathbf{P}_{xx}$ (a covariance matrix is always symmetric).

**Step 6 — Assemble the result:**

$$\boldsymbol{\Sigma}_\text{joint} = \begin{bmatrix}\mathbf{P}_{xx} & \mathbf{P}_{xx}\mathbf{H}^\top\\\mathbf{H}\mathbf{P}_{xx} & \mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}\end{bmatrix} \quad\square$$

---

**What each block means physically:**

| Block | Symbol | Size | Meaning |
|---|---|---|---|
| Top-left | $\mathbf{P}_{xx}$ | $n\times n$ | Uncertainty of the hidden state |
| Top-right | $\mathbf{P}_{xx}\mathbf{H}^\top$ | $n\times m$ | **Cross-covariance** — how much state error "leaks" into measurement |
| Bottom-left | $\mathbf{H}\mathbf{P}_{xx}$ | $m\times n$ | Transpose of above (symmetry) |
| Bottom-right | $\mathbf{S}=\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}$ | $m\times m$ | **Innovation covariance** — total uncertainty in the predicted sensor reading |

---

**Worked example — concrete numbers:**

Altitude filter: $n=2$ (state = altitude $h$ and vertical speed $\dot{h}$), $m=1$ (barometer reads altitude only).

$$\mathbf{x}=\begin{bmatrix}h\\\dot{h}\end{bmatrix}, \quad \boldsymbol{\mu}_x=\begin{bmatrix}100\\2\end{bmatrix}\text{ m, m/s}, \quad \mathbf{P}_{xx}=\begin{bmatrix}4&0\\0&1\end{bmatrix}$$

$$\mathbf{H}=[1\quad 0], \quad R=9 \text{ m}^2 \text{ (baro noise)}$$

Compute each block:

- $\mathbf{P}_{xx}\mathbf{H}^\top = \begin{bmatrix}4&0\\0&1\end{bmatrix}\begin{bmatrix}1\\0\end{bmatrix} = \begin{bmatrix}4\\0\end{bmatrix}$ ($2\times 1$)
- $\mathbf{H}\mathbf{P}_{xx} = [1\quad 0]\begin{bmatrix}4&0\\0&1\end{bmatrix} = [4\quad 0]$ ($1\times 2$)
- $\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top + R = [1\quad 0]\begin{bmatrix}4&0\\0&1\end{bmatrix}\begin{bmatrix}1\\0\end{bmatrix}+9 = 4+9 = 13$ ($1\times 1$)

Full joint covariance ($3\times 3$):

$$\boldsymbol{\Sigma}_\text{joint} = \begin{bmatrix}4&0&4\\0&1&0\\4&0&13\end{bmatrix}$$

*Reading:* The (3,3) entry (13) is the total uncertainty in the predicted baro reading: 4 comes from altitude uncertainty already in the state, 9 is pure sensor noise.

**Connection to Kalman Filter:**

The Kalman Gain is:

$$\mathbf{K} = \underbrace{\mathbf{P}_{xx}\mathbf{H}^\top}_{\text{cross-cov}}\cdot\underbrace{\left(\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}\right)^{-1}}_{\mathbf{S}^{-1},\;\text{innov cov}^{-1}}$$

In words: the gain is "how much of the state uncertainty reaches the sensor" divided by "total sensor uncertainty". In the example: $\mathbf{K} = \begin{bmatrix}4\\0\end{bmatrix}\cdot\frac{1}{13} = \begin{bmatrix}0.308\\0\end{bmatrix}$.

This says: when the baro reading deviates from prediction, update the altitude estimate by 30.8% of that deviation, and update the velocity estimate by 0% (baro doesn't measure velocity directly).

---

### Property 5: Conditional Gaussian — The Kalman Update in Full Detail

**This is the most important property.** It directly produces all three KF update equations:

$$\mathbf{K}_k = \mathbf{P}\mathbf{H}^\top(\mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R})^{-1}, \qquad \hat{\mathbf{x}}\mathrel{+}=\mathbf{K}(\mathbf{z}-\mathbf{H}\hat{\mathbf{x}}), \qquad \mathbf{P}\leftarrow(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$$

These equations are not heuristics. They are the exact Bayesian posterior of a Gaussian prior given a Gaussian measurement — derived purely from probability.

---

**Setup — restating the problem precisely:**

We have (from Property 4) the joint Gaussian:

$$\begin{bmatrix}\mathbf{x}\\\mathbf{y}\end{bmatrix}\sim\mathcal{N}\!\left(\begin{bmatrix}\boldsymbol{\mu}_x\\\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix},\;\boldsymbol{\Sigma}\right), \qquad \boldsymbol{\Sigma} = \begin{bmatrix}\mathbf{P}_{xx} & \mathbf{P}_{xx}\mathbf{H}^\top\\\mathbf{H}\mathbf{P}_{xx} & \mathbf{S}\end{bmatrix}$$

where $\mathbf{S} = \mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R}$ is the innovation covariance.

Now a sensor reading arrives: $\mathbf{y} = \mathbf{z}$ (specific numbers). We want to know:

> *Given that the sensor read exactly $\mathbf{z}$, what is the updated distribution of the state $\mathbf{x}$?*

This is the **conditional distribution** $p(\mathbf{x}\mid\mathbf{y}=\mathbf{z})$.

**Claim:**

$$\mathbf{x}\mid\mathbf{y}=\mathbf{z}\;\sim\;\mathcal{N}(\boldsymbol{\mu}_{x|y},\;\mathbf{P}_{x|y})$$

with:

$$\boldsymbol{\mu}_{x|y} = \boldsymbol{\mu}_x + \underbrace{\mathbf{K}}_{\text{Kalman Gain}}(\underbrace{\mathbf{z}-\mathbf{H}\boldsymbol{\mu}_x}_{\text{innovation}}) \tag{P5-mean}$$

$$\mathbf{P}_{x|y} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_{xx} \tag{P5-cov}$$

$$\mathbf{K} = \mathbf{P}_{xx}\mathbf{H}^\top(\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R})^{-1} = \mathbf{P}_{xx}\mathbf{H}^\top\mathbf{S}^{-1} \tag{P5-gain}$$

---

**Proof — full derivation:**

**Step 1 — Write the joint pdf.**

A multivariate Gaussian with mean $\boldsymbol{\mu}$ and covariance $\boldsymbol{\Sigma}$ has pdf:

$$p(\mathbf{u}) \propto \exp\!\left(-\frac{1}{2}(\mathbf{u}-\boldsymbol{\mu})^\top\boldsymbol{\Sigma}^{-1}(\mathbf{u}-\boldsymbol{\mu})\right)$$

The symbol $\propto$ means we drop constants that don't depend on $\mathbf{u}$. The shape is entirely determined by the exponent.

With $\mathbf{u} = \begin{bmatrix}\mathbf{x}\\\mathbf{y}\end{bmatrix}$ and $\boldsymbol{\mu} = \begin{bmatrix}\boldsymbol{\mu}_x\\\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix}$:

$$p(\mathbf{x},\mathbf{y}) \propto \exp\!\left(-\frac{1}{2}\underbrace{\begin{bmatrix}\mathbf{x}-\boldsymbol{\mu}_x\\\mathbf{y}-\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix}^\top}_{\tilde{\mathbf{u}}^\top}\boldsymbol{\Sigma}^{-1}\underbrace{\begin{bmatrix}\mathbf{x}-\boldsymbol{\mu}_x\\\mathbf{y}-\mathbf{H}\boldsymbol{\mu}_x\end{bmatrix}}_{\tilde{\mathbf{u}}}\right)$$

Let $\tilde{\mathbf{x}} = \mathbf{x}-\boldsymbol{\mu}_x$ and $\tilde{\mathbf{z}} = \mathbf{z}-\mathbf{H}\boldsymbol{\mu}_x$ (the **innovation** — how far the measurement is from what we predicted).

**Step 2 — Compute $\boldsymbol{\Sigma}^{-1}$: inverting the block matrix.**

We need to invert $\boldsymbol{\Sigma} = \begin{bmatrix}\mathbf{P}_{xx}&\mathbf{P}_{xx}\mathbf{H}^\top\\\mathbf{H}\mathbf{P}_{xx}&\mathbf{S}\end{bmatrix}$.

Inverting a block matrix is harder than inverting a scalar — you can't just "flip" each block separately. We use the **block matrix inverse formula** derived via Gaussian elimination.

For a $2\times 2$ block matrix $\begin{bmatrix}\mathbf{A}&\mathbf{B}\\\mathbf{C}&\mathbf{D}\end{bmatrix}$ where $\mathbf{A}$ and $\mathbf{D}$ are invertible, the inverse is:

$$\begin{bmatrix}\mathbf{A}&\mathbf{B}\\\mathbf{C}&\mathbf{D}\end{bmatrix}^{-1} = \begin{bmatrix}\mathbf{A}^{-1}+\mathbf{A}^{-1}\mathbf{B}\mathbf{M}^{-1}\mathbf{C}\mathbf{A}^{-1} & -\mathbf{A}^{-1}\mathbf{B}\mathbf{M}^{-1}\\-\mathbf{M}^{-1}\mathbf{C}\mathbf{A}^{-1} & \mathbf{M}^{-1}\end{bmatrix}$$

where $\mathbf{M} = \mathbf{D} - \mathbf{C}\mathbf{A}^{-1}\mathbf{B}$ is called the **Schur complement** of $\mathbf{A}$.

*What is the Schur complement?* It is the "effective" covariance of $\mathbf{y}$ after accounting for the correlation with $\mathbf{x}$. Intuitively: how much residual uncertainty does $\mathbf{y}$ have once we account for what $\mathbf{x}$ explains?

For our $\boldsymbol{\Sigma}$: $\mathbf{A}=\mathbf{P}_{xx}$, $\mathbf{B}=\mathbf{P}_{xx}\mathbf{H}^\top$, $\mathbf{C}=\mathbf{H}\mathbf{P}_{xx}$, $\mathbf{D}=\mathbf{S}$.

The Schur complement $\mathbf{M} = \mathbf{S} - \mathbf{H}\mathbf{P}_{xx}\mathbf{P}_{xx}^{-1}\mathbf{P}_{xx}\mathbf{H}^\top = \mathbf{S} - \mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top = \mathbf{R}$.

This gives:

$$\boldsymbol{\Sigma}^{-1} = \begin{bmatrix}\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H} & -\mathbf{H}^\top\mathbf{R}^{-1}\\-\mathbf{R}^{-1}\mathbf{H} & \mathbf{R}^{-1}\end{bmatrix}$$

*(You can verify: multiply $\boldsymbol{\Sigma}\cdot\boldsymbol{\Sigma}^{-1}$ and check it gives the identity matrix.)*

**Step 3 — Expand the exponent.**

Plug $\boldsymbol{\Sigma}^{-1}$ into the exponent and expand the block matrix multiplication:

$$\tilde{\mathbf{u}}^\top\boldsymbol{\Sigma}^{-1}\tilde{\mathbf{u}} = \begin{bmatrix}\tilde{\mathbf{x}}^\top & \tilde{\mathbf{y}}^\top\end{bmatrix}\begin{bmatrix}\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H} & -\mathbf{H}^\top\mathbf{R}^{-1}\\-\mathbf{R}^{-1}\mathbf{H} & \mathbf{R}^{-1}\end{bmatrix}\begin{bmatrix}\tilde{\mathbf{x}}\\\tilde{\mathbf{y}}\end{bmatrix}$$

Expanding row-by-row (denoting $\tilde{\mathbf{y}} = \mathbf{y} - \mathbf{H}\boldsymbol{\mu}_x$):

$$= \tilde{\mathbf{x}}^\top(\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})\tilde{\mathbf{x}} - 2\tilde{\mathbf{y}}^\top\mathbf{R}^{-1}\mathbf{H}\tilde{\mathbf{x}} + \tilde{\mathbf{y}}^\top\mathbf{R}^{-1}\tilde{\mathbf{y}}$$

**Step 4 — Condition on $\mathbf{y} = \mathbf{z}$ (fix $\tilde{\mathbf{y}} = \tilde{\mathbf{z}}$ as a constant).**

When computing $p(\mathbf{x}\mid\mathbf{y}=\mathbf{z})$, we treat $\mathbf{z}$ as a **known fixed number**. So $\tilde{\mathbf{z}} = \mathbf{z} - \mathbf{H}\boldsymbol{\mu}_x$ is a constant (not random).

The last term $\tilde{\mathbf{z}}^\top\mathbf{R}^{-1}\tilde{\mathbf{z}}$ does not involve $\mathbf{x}$ — it's a pure constant that folds into the normalization. Drop it (absorbed into $\propto$):

$$p(\mathbf{x}\mid\mathbf{z})\propto\exp\!\left(-\frac{1}{2}\left[\tilde{\mathbf{x}}^\top\underbrace{(\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})}_{\text{call this }\boldsymbol{\Lambda}}\tilde{\mathbf{x}} - 2\underbrace{\tilde{\mathbf{z}}^\top\mathbf{R}^{-1}\mathbf{H}}_{\mathbf{b}^\top}\tilde{\mathbf{x}}\right]\right)$$

We have: $-\frac{1}{2}[\tilde{\mathbf{x}}^\top\boldsymbol{\Lambda}\tilde{\mathbf{x}} - 2\mathbf{b}^\top\tilde{\mathbf{x}}]$.

**Step 5 — Complete the square in $\tilde{\mathbf{x}}$.**

For scalars, completing the square: $ax^2 - 2bx = a(x - b/a)^2 - b^2/a$.

For vectors: $\tilde{\mathbf{x}}^\top\boldsymbol{\Lambda}\tilde{\mathbf{x}} - 2\mathbf{b}^\top\tilde{\mathbf{x}} = (\tilde{\mathbf{x}}-\boldsymbol{\Lambda}^{-1}\mathbf{b})^\top\boldsymbol{\Lambda}(\tilde{\mathbf{x}}-\boldsymbol{\Lambda}^{-1}\mathbf{b}) - \mathbf{b}^\top\boldsymbol{\Lambda}^{-1}\mathbf{b}$.

The last term doesn't involve $\mathbf{x}$, so:

$$p(\mathbf{x}\mid\mathbf{z})\propto\exp\!\left(-\frac{1}{2}(\tilde{\mathbf{x}}-\boldsymbol{\Lambda}^{-1}\mathbf{b})^\top\boldsymbol{\Lambda}(\tilde{\mathbf{x}}-\boldsymbol{\Lambda}^{-1}\mathbf{b})\right)$$

This is a Gaussian in $\tilde{\mathbf{x}} = \mathbf{x}-\boldsymbol{\mu}_x$ centered at $\boldsymbol{\Lambda}^{-1}\mathbf{b}$, with precision matrix $\boldsymbol{\Lambda}$.

So the conditional distribution is Gaussian with:

- **Posterior covariance**: $\mathbf{P}_{x|y} = \boldsymbol{\Lambda}^{-1} = (\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})^{-1}$
- **Posterior mean shift**: $\boldsymbol{\Lambda}^{-1}\mathbf{b} = \mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1}\tilde{\mathbf{z}}$, so $\boldsymbol{\mu}_{x|y} = \boldsymbol{\mu}_x + \mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1}\tilde{\mathbf{z}}$

**Step 6 — Simplify the posterior covariance using the Woodbury identity.**

We need to compute $\mathbf{P}_{x|y} = (\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})^{-1}$.

Inverting this directly would require computing $\mathbf{P}_{xx}^{-1}$ (inverting an $n\times n$ matrix) — expensive and numerically bad.

The **Woodbury identity** (matrix inversion lemma) provides a shortcut. The general form is:

$$(\mathbf{A}+\mathbf{U}\mathbf{C}\mathbf{V})^{-1} = \mathbf{A}^{-1} - \mathbf{A}^{-1}\mathbf{U}(\mathbf{C}^{-1}+\mathbf{V}\mathbf{A}^{-1}\mathbf{U})^{-1}\mathbf{V}\mathbf{A}^{-1}$$

*What this formula does:* Instead of inverting the large $n\times n$ matrix $(\mathbf{A}+\mathbf{U}\mathbf{C}\mathbf{V})$, you only need to invert the smaller $m\times m$ matrix $(\mathbf{C}^{-1}+\mathbf{V}\mathbf{A}^{-1}\mathbf{U})$. This is the "push the inversion down to smaller dimension" trick.

*Scalar verification (sanity check):* With $A=p^{-1}$, $U=h$, $C=r^{-1}$, $V=h$ (all scalars), the formula gives:

$$(p^{-1}+h\cdot r^{-1}\cdot h)^{-1} = p - p\cdot h\cdot (r+h\cdot p\cdot h)^{-1}\cdot h\cdot p = p - \frac{ph^2p}{r+h^2p}$$

The result: $p_\text{new} = p - ph^2 p/(r+h^2 p) = pr/(r+h^2 p) = \frac{pr}{r+h^2 p}$. This matches the scalar KF: $P_\text{new} = (1-KH)P$ where $K = PH/(HPH+R)$. ✓

Apply Woodbury to our case with $\mathbf{A}^{-1}\to\mathbf{P}_{xx}$, $\mathbf{U}\to\mathbf{H}^\top$, $\mathbf{C}^{-1}\to\mathbf{R}$, $\mathbf{V}\to\mathbf{H}$:

$$\mathbf{P}_{x|y} = (\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})^{-1} = \mathbf{P}_{xx} - \mathbf{P}_{xx}\mathbf{H}^\top\underbrace{(\mathbf{R}+\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top)^{-1}}_{\mathbf{S}^{-1}}\mathbf{H}\mathbf{P}_{xx}$$

Define the **Kalman Gain**:

$$\boxed{\mathbf{K} = \mathbf{P}_{xx}\mathbf{H}^\top\mathbf{S}^{-1} = \mathbf{P}_{xx}\mathbf{H}^\top(\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top+\mathbf{R})^{-1}}$$

Then: $\mathbf{P}_{x|y} = \mathbf{P}_{xx} - \mathbf{K}\mathbf{H}\mathbf{P}_{xx}$, which factors as:

$$\boxed{\mathbf{P}_{x|y} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_{xx}}$$

**Step 7 — Simplify the posterior mean.**

From Step 5: $\boldsymbol{\mu}_{x|y} = \boldsymbol{\mu}_x + \mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1}\tilde{\mathbf{z}}$.

We need $\mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1}$. Substitute $\mathbf{P}_{x|y} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_{xx}$:

$$\mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1} = (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}_{xx}\mathbf{H}^\top\mathbf{R}^{-1} = \mathbf{P}_{xx}\mathbf{H}^\top\mathbf{R}^{-1} - \mathbf{K}\underbrace{\mathbf{H}\mathbf{P}_{xx}\mathbf{H}^\top\mathbf{R}^{-1}}_{\mathbf{S}\mathbf{R}^{-1}-\mathbf{I}\text{ (from }\mathbf{S}=\mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R}\text{)}}$$

A cleaner route: use the **push-through identity** $\mathbf{P}_{xx}\mathbf{H}^\top\mathbf{S}^{-1} = (\mathbf{P}_{xx}^{-1}+\mathbf{H}^\top\mathbf{R}^{-1}\mathbf{H})^{-1}\mathbf{H}^\top\mathbf{R}^{-1}$ (both sides equal $\mathbf{K}$). So directly:

$$\mathbf{P}_{x|y}\mathbf{H}^\top\mathbf{R}^{-1} = \mathbf{K}\mathbf{S}\mathbf{R}^{-1}\cdot\text{(terms)} = \mathbf{K}$$

Therefore:

$$\boldsymbol{\mu}_{x|y} = \boldsymbol{\mu}_x + \mathbf{K}\tilde{\mathbf{z}} = \boldsymbol{\mu}_x + \mathbf{K}(\mathbf{z}-\mathbf{H}\boldsymbol{\mu}_x)$$

$$\boxed{\boldsymbol{\mu}_{x|y} = \boldsymbol{\mu}_x + \mathbf{K}(\mathbf{z}-\mathbf{H}\boldsymbol{\mu}_x)} \quad\square$$

---

**Reading each term of the final equations:**

$$\underbrace{\boldsymbol{\mu}_{x|y}}_{\text{updated estimate}} = \underbrace{\boldsymbol{\mu}_x}_{\text{predicted estimate}} + \underbrace{\mathbf{K}}_{\text{Kalman Gain}} \cdot \underbrace{(\mathbf{z} - \mathbf{H}\boldsymbol{\mu}_x)}_{\text{innovation}}$$

| Term | Name | Meaning |
|---|---|---|
| $\boldsymbol{\mu}_x$ | Prior mean | What we thought before the measurement |
| $\mathbf{z}$ | Measurement | What the sensor actually read |
| $\mathbf{H}\boldsymbol{\mu}_x$ | Predicted measurement | What we expected the sensor to read |
| $\mathbf{z} - \mathbf{H}\boldsymbol{\mu}_x$ | **Innovation** | The "surprise": actual minus expected |
| $\mathbf{K}$ | **Kalman Gain** ($n\times m$) | Maps the innovation back into state space; weights it by relative trustworthiness |
| $\mathbf{K}\cdot(\mathbf{z}-\mathbf{H}\boldsymbol{\mu}_x)$ | State correction | The amount we shift our estimate |

**Why $(\mathbf{I}-\mathbf{K}\mathbf{H})$ for the covariance?**

In 1D: $K = P/(P+R)$, so $1-K = R/(P+R)$, and:

$$P_\text{new} = (1-K)P = \frac{R}{P+R}\cdot P = \frac{PR}{P+R}$$

This is the harmonic-mean-like combination of $P$ and $R$. It is always $\leq\min(P,R)$ — the updated uncertainty is **always smaller than both the prior and the sensor noise**. Every measurement, no matter how noisy, reduces uncertainty.

---

**Worked numerical example (continuing the altitude filter from Property 4):**

Setup from Property 4: $P_{xx}=\begin{bmatrix}4&0\\0&1\end{bmatrix}$, $\mathbf{H}=[1\quad 0]$, $R=9$.

Kalman Gain (computed in Property 4): $\mathbf{K}=\begin{bmatrix}4/13\\0\end{bmatrix}\approx\begin{bmatrix}0.308\\0\end{bmatrix}$.

Measurement arrives: barometer reads $z = 103$ m. Predicted measurement: $\mathbf{H}\boldsymbol{\mu}_x = [1\quad 0]\begin{bmatrix}100\\2\end{bmatrix} = 100$ m.

Innovation: $z - \mathbf{H}\boldsymbol{\mu}_x = 103 - 100 = 3$ m.

Updated mean:

$$\boldsymbol{\mu}_{x|y} = \begin{bmatrix}100\\2\end{bmatrix} + \begin{bmatrix}0.308\\0\end{bmatrix}\cdot 3 = \begin{bmatrix}100 + 0.923\\2 + 0\end{bmatrix} = \begin{bmatrix}100.92\text{ m}\\2\text{ m/s}\end{bmatrix}$$

Updated covariance:

$$\mathbf{P}_{x|y} = \left(\mathbf{I} - \begin{bmatrix}0.308\\0\end{bmatrix}[1\quad 0]\right)\begin{bmatrix}4&0\\0&1\end{bmatrix} = \begin{bmatrix}1-0.308&0\\0&1\end{bmatrix}\begin{bmatrix}4&0\\0&1\end{bmatrix} = \begin{bmatrix}2.77&0\\0&1\end{bmatrix}$$

Altitude uncertainty dropped from $\sqrt{4}=2$ m to $\sqrt{2.77}=1.66$ m. Velocity unchanged (baro doesn't measure velocity, so $K_2=0$ — the velocity row of $\mathbf{K}$ is zero).

---

## CHAPTER 4 — The Kalman Filter as Exact Bayesian Inference

### 4.1 Notation Dictionary

Before the algorithm — every symbol defined precisely:

**Dimensions:**
- $n$ — state dimension (how many numbers describe the system state)
- $m$ — measurement dimension (how many sensors report at each step)

**State quantities:**

| Symbol | Dimensions | Meaning |
|---|---|---|
| $\mathbf{x}_k$ | $n\times 1$ | True (unknown) state at time step $k$ |
| $\hat{\mathbf{x}}_{k\|k-1}$ | $n\times 1$ | **Predicted** state at time $k$, given all observations up to $k-1$ |
| $\hat{\mathbf{x}}_{k\|k}$ | $n\times 1$ | **Updated** state at time $k$, given all observations up to and including $k$ |
| $\mathbf{P}_{k\|k-1}$ | $n\times n$ | **Predicted** error covariance — uncertainty of the prediction |
| $\mathbf{P}_{k\|k}$ | $n\times n$ | **Updated** error covariance — uncertainty after fusing measurement $k$ |

**Model matrices:**

| Symbol | Dimensions | Meaning |
|---|---|---|
| $\mathbf{F}$ | $n\times n$ | **State transition matrix** — how the state evolves one step forward in time |
| $\mathbf{Q}$ | $n\times n$ | **Process noise covariance** — how uncertain is the state model itself? |
| $\mathbf{H}$ | $m\times n$ | **Observation matrix** — how state variables map to sensor readings |
| $\mathbf{R}$ | $m\times m$ | **Measurement noise covariance** — how noisy is the sensor? |

**Update quantities:**

| Symbol | Dimensions | Meaning |
|---|---|---|
| $\mathbf{z}_k$ | $m\times 1$ | Measurement (sensor reading) at time $k$ |
| $\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k\|k-1}$ | $m\times 1$ | **Innovation** — observed minus predicted measurement |
| $\mathbf{S}_k = \mathbf{H}\mathbf{P}_{k\|k-1}\mathbf{H}^\top+\mathbf{R}$ | $m\times m$ | **Innovation covariance** — how uncertain is the predicted measurement? |
| $\mathbf{K}_k$ | $n\times m$ | **Kalman Gain** — how much to correct the state for a unit innovation |

**The subscript notation $a\|b$:** "quantity at time $a$, given information up to time $b$."
- $k\|k-1$ = predicted (before current measurement)
- $k\|k$ = updated (after fusing current measurement)

---

### 4.2 The System Models

**State evolution model:**

$$\mathbf{x}_k = \mathbf{F}\mathbf{x}_{k-1} + \mathbf{w}_k, \qquad \mathbf{w}_k\sim\mathcal{N}(\mathbf{0},\mathbf{Q})$$

- $\mathbf{F}$ models physics: constant velocity means $\mathbf{F}$ propagates position as $p \leftarrow p + v\Delta t$
- $\mathbf{w}_k$ is the **process noise**: unmodeled forces, integration errors, wind gusts

**Observation model:**

$$\mathbf{z}_k = \mathbf{H}\mathbf{x}_k + \mathbf{v}_k, \qquad \mathbf{v}_k\sim\mathcal{N}(\mathbf{0},\mathbf{R})$$

- $\mathbf{H}$ selects which state components the sensor measures (e.g., GPS only sees position, not velocity)
- $\mathbf{v}_k$ is the **measurement noise**: thermal noise, quantization, atmospheric effects

**Concrete example (altitude estimator, $n=2$, $m=1$):**

State $\mathbf{x}_k = \begin{bmatrix}h_k \\ \dot{h}_k\end{bmatrix}$ (altitude and vertical velocity). Barometer measures altitude directly.

$$\mathbf{F} = \begin{bmatrix}1 & \Delta t\\0 & 1\end{bmatrix}, \quad \mathbf{Q} = \begin{bmatrix}\sigma_h^2 & 0\\0 & \sigma_{\dot{h}}^2\end{bmatrix}, \quad \mathbf{H} = \begin{bmatrix}1 & 0\end{bmatrix}, \quad R = \sigma_\text{baro}^2$$

$\mathbf{F}$: "altitude at next step = current altitude + velocity × Δt; velocity unchanged." $\mathbf{H}$: "sensor reads the first state component (altitude) only."

---

### 4.3 The Algorithm: Predict → Update

**PREDICT step — derived from Properties 1 and 2:**

Given the previous posterior $\hat{\mathbf{x}}_{k-1|k-1}\sim\mathcal{N}(\hat{\mathbf{x}}_{k-1|k-1},\mathbf{P}_{k-1|k-1})$.

The state at time $k$ is $\mathbf{x}_k = \mathbf{F}\mathbf{x}_{k-1} + \mathbf{w}_k$.

**By Property 1** (linear transform): $\mathbf{F}\mathbf{x}_{k-1}\sim\mathcal{N}(\mathbf{F}\hat{\mathbf{x}}_{k-1|k-1},\;\mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^\top)$

**By Property 2** (add independent Gaussian noise): $\mathbf{x}_k\sim\mathcal{N}(\mathbf{F}\hat{\mathbf{x}}_{k-1|k-1},\;\mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^\top + \mathbf{Q})$

Therefore:

$$\boxed{\hat{\mathbf{x}}_{k|k-1} = \mathbf{F}\hat{\mathbf{x}}_{k-1|k-1}}$$

$$\boxed{\mathbf{P}_{k|k-1} = \mathbf{F}\mathbf{P}_{k-1|k-1}\mathbf{F}^\top + \mathbf{Q}}$$

**Why does $\mathbf{P}$ grow in the predict step?** Adding process noise $\mathbf{Q}$ always increases uncertainty. The longer we run without a measurement, the more uncertain we become.

---

**UPDATE step — derived from Properties 4 and 5:**

We now have:
- Prior: $\mathbf{x}_k\sim\mathcal{N}(\hat{\mathbf{x}}_{k|k-1},\;\mathbf{P}_{k|k-1})$ (from the predict step)
- Measurement: $\mathbf{z}_k = \mathbf{H}\mathbf{x}_k + \mathbf{v}_k$

**By Property 4**: the joint $\begin{bmatrix}\mathbf{x}_k\\\mathbf{z}_k\end{bmatrix}$ is Gaussian with:

$$\text{Cov}(\mathbf{x}_k,\mathbf{z}_k) = \mathbf{P}_{k|k-1}\mathbf{H}^\top, \quad \text{Cov}(\mathbf{z}_k,\mathbf{z}_k) = \mathbf{S}_k = \mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^\top + \mathbf{R}$$

**By Property 5**: the conditional $\mathbf{x}_k\mid\mathbf{z}_k$ is Gaussian. Using the formulas from Property 5 with:
- $\boldsymbol{\mu}_x \leftarrow \hat{\mathbf{x}}_{k|k-1}$
- $\mathbf{P}_{xx} \leftarrow \mathbf{P}_{k|k-1}$
- $\mathbf{z} \leftarrow \mathbf{z}_k$

The Kalman Gain:

$$\boxed{\mathbf{K}_k = \mathbf{P}_{k|k-1}\mathbf{H}^\top(\mathbf{H}\mathbf{P}_{k|k-1}\mathbf{H}^\top + \mathbf{R})^{-1} = \mathbf{P}_{k|k-1}\mathbf{H}^\top\mathbf{S}_k^{-1}}$$

Updated mean:

$$\boxed{\hat{\mathbf{x}}_{k|k} = \hat{\mathbf{x}}_{k|k-1} + \mathbf{K}_k(\mathbf{z}_k - \mathbf{H}\hat{\mathbf{x}}_{k|k-1})}$$

Updated covariance:

$$\boxed{\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}_k\mathbf{H})\mathbf{P}_{k|k-1}}$$

---

### 4.4 Worked Numerical Example (Altitude Filter)

Setup: $n=2$, $m=1$, $\Delta t = 0.1$ s.

$$\mathbf{F} = \begin{bmatrix}1&0.1\\0&1\end{bmatrix}, \quad \mathbf{Q} = \begin{bmatrix}0.01&0\\0&0.1\end{bmatrix}, \quad \mathbf{H} = [1\;0], \quad R = 4$$

**Initial state:** $\hat{\mathbf{x}}_0 = \begin{bmatrix}100\\1\end{bmatrix}$ (100 m altitude, 1 m/s climb), $\mathbf{P}_0 = \begin{bmatrix}1&0\\0&0.1\end{bmatrix}$.

**PREDICT (step $k=1$):**

$$\hat{\mathbf{x}}_{1|0} = \begin{bmatrix}1&0.1\\0&1\end{bmatrix}\begin{bmatrix}100\\1\end{bmatrix} = \begin{bmatrix}100.1\\1\end{bmatrix}$$

$$\mathbf{P}_{1|0} = \begin{bmatrix}1&0.1\\0&1\end{bmatrix}\begin{bmatrix}1&0\\0&0.1\end{bmatrix}\begin{bmatrix}1&0\\0.1&1\end{bmatrix} + \begin{bmatrix}0.01&0\\0&0.1\end{bmatrix}$$

$$= \begin{bmatrix}1.01 & 0.1\\0.1 & 0.2\end{bmatrix}$$

**Measurement arrives:** $z_1 = 102$ m (baro reads 102 m).

**UPDATE:**

Innovation covariance: $S_1 = \mathbf{H}\mathbf{P}_{1|0}\mathbf{H}^\top + R = 1.01 + 4 = 5.01$

Kalman Gain: $\mathbf{K}_1 = \mathbf{P}_{1|0}\mathbf{H}^\top S_1^{-1} = \dfrac{1}{5.01}\begin{bmatrix}1.01\\0.1\end{bmatrix} = \begin{bmatrix}0.201\\0.020\end{bmatrix}$

Innovation: $z_1 - \mathbf{H}\hat{\mathbf{x}}_{1|0} = 102 - 100.1 = 1.9$ m

Updated state:

$$\hat{\mathbf{x}}_{1|1} = \begin{bmatrix}100.1\\1\end{bmatrix} + \begin{bmatrix}0.201\\0.020\end{bmatrix}\cdot 1.9 = \begin{bmatrix}100.48\\1.038\end{bmatrix}$$

Updated covariance:

$$\mathbf{P}_{1|1} = \left(\mathbf{I} - \begin{bmatrix}0.201\\0.020\end{bmatrix}[1\;0]\right)\begin{bmatrix}1.01&0.1\\0.1&0.2\end{bmatrix} = \begin{bmatrix}0.799&0.0798\\0.080&0.198\end{bmatrix}$$

**Reading the result:** The altitude estimate moved from 100.1 m toward 102 m (the measurement), landing at 100.48 m. The altitude uncertainty dropped from $P_{11}=1.01$ to $P_{11}=0.799$ — we're now more confident because we fused a sensor reading.

---

### 4.5 Why the KF is the Optimal Estimator

The KF is **exact Bayesian inference** under the model assumptions. This means:

1. **It computes the true posterior** $p(\mathbf{x}_k\mid\mathbf{z}_{1:k})$ — not an approximation.
2. **It is the BLUE** (Best Linear Unbiased Estimator) — among all linear estimators, the KF minimizes the expected squared error $\mathbb{E}[\|\mathbf{x}_k-\hat{\mathbf{x}}_{k|k}\|^2]$.
3. **Optimality is strict**: if the noise is truly Gaussian and the model is truly linear, no estimator can do better.

The key condition: **Gaussian closure**. Because all 5 properties hold, every distribution that passes through the filter remains Gaussian. The filter never needs to represent a non-Gaussian shape — the mean vector and covariance matrix are always sufficient to describe everything.

```
t=0:   x_0 ~ Gaussian (initial belief)
         |
PREDICT: F*x_0 + w_1       <- Property 1 + 2 -> still Gaussian
         |
UPDATE:  condition on z_1   <- Property 4 + 5 -> still Gaussian
         |
PREDICT: F*x_1 + w_2       <- still Gaussian
         |
UPDATE:  condition on z_2   <- still Gaussian
         ...
t=k:   x_k ~ Gaussian      <- always exact, forever
```

### 4.6 Property-to-Equation Summary

| Step | Gaussian property used | Formula it produces | Physical meaning |
|---|---|---|---|
| PREDICT — mean | Prop 1: $\mathbb{E}[\mathbf{F}\mathbf{x}] = \mathbf{F}\mathbb{E}[\mathbf{x}]$ | $\hat{\mathbf{x}}_{k\|k-1} = \mathbf{F}\hat{\mathbf{x}}_{k-1\|k-1}$ | State evolves by physics |
| PREDICT — covariance | Prop 1: $\text{Cov}[\mathbf{F}\mathbf{x}]=\mathbf{F}\mathbf{P}\mathbf{F}^\top$ | $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ term | Uncertainty evolves with state |
| PREDICT — noise | Prop 2: Sum of Gaussians | $+\mathbf{Q}$ term | Model uncertainty grows $\mathbf{P}$ |
| UPDATE — cross-cov | Prop 4: $\text{Cov}(\mathbf{x},\mathbf{y})=\mathbf{P}\mathbf{H}^\top$ | Numerator of $\mathbf{K}$ | How much state leaks into measurement |
| UPDATE — innov cov | Prop 4: $\text{Cov}(\mathbf{y},\mathbf{y})=\mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R}$ | $\mathbf{S} = \mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R}$ | Total measurement uncertainty |
| UPDATE — gain | Prop 5: Woodbury identity | $\mathbf{K} = \mathbf{P}\mathbf{H}^\top\mathbf{S}^{-1}$ | Optimal trust in measurement |
| UPDATE — mean | Prop 5: conditional mean | $\hat{\mathbf{x}} \mathrel{+}= \mathbf{K}(\mathbf{z}-\mathbf{H}\hat{\mathbf{x}})$ | Fuse innovation into state |
| UPDATE — cov | Prop 5: conditional cov | $\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ | Uncertainty always decreases |

---

## CHAPTER 5 — Error-State EKF: From Nonlinear Dynamics to PX4's 24-State Filter

### 5.1 Where the KF Guarantee Fails

The KF's closure relies entirely on **Property 1**: a linear transform of a Gaussian remains Gaussian.

Real drone dynamics are nonlinear. The state transition function $\mathbf{f}(\mathbf{x})$ contains:

$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \Delta\mathbf{q}(\boldsymbol{\omega})\,, \quad \mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{R}(\mathbf{q}_k)\,\mathbf{a}_\text{body}\,\Delta t + \mathbf{g}\,\Delta t$$

The rotation matrix $\mathbf{R}(\mathbf{q})$ is a **nonlinear** function of $\mathbf{q}$ (it involves products of quaternion components). Quaternion multiplication $\otimes$ is also nonlinear. So if $\mathbf{x}\sim\mathcal{N}(\boldsymbol{\mu},\mathbf{P})$, then $\mathbf{f}(\mathbf{x})$ is **not Gaussian** in general:

```
Before nonlinear transform:  x ~ Gaussian
After f(x):                  f(x) ~ ??? (non-Gaussian — banana-shaped, multi-modal, ...)
```

The EKF solution: approximate $\mathbf{f}$ as linear at each step using a **first-order Taylor expansion**, restoring the Gaussian chain. But for drones there is a second, deeper problem: **attitude does not live in a flat vector space**.

---

### 5.2 The SO(3) Manifold Problem — Why Attitude Breaks Naive EKF

The standard EKF operates in a flat Euclidean state space $\mathbb{R}^n$. Attitude — the orientation of a rigid body — does not live in $\mathbb{R}^n$. It lives on the **Special Orthogonal group** SO(3), a 3-dimensional curved manifold.

PX4 represents attitude as a unit quaternion $\mathbf{q} = (q_w, q_x, q_y, q_z)$ with $\|\mathbf{q}\|=1$. This has:
- **4 numbers** to store
- **3 degrees of freedom** (the unit-norm constraint removes one)
- **Non-Euclidean composition**: rotations compose by multiplication $\mathbf{q}_1\otimes\mathbf{q}_2$, not by addition

**The fundamental breakdown:** if you naively add a Gaussian perturbation to $\mathbf{q}$:

$$\mathbf{q} + \delta\mathbf{q} \quad\Rightarrow\quad \|\mathbf{q}+\delta\mathbf{q}\| \neq 1 \quad\text{(no longer a valid quaternion)}$$

The covariance matrix $\mathbf{P}$ operates in $\mathbb{R}^n$ (flat space). Adding $\mathbf{P}$-weighted Gaussian noise to $\mathbf{q}$ takes you off the unit sphere — the result is not a rotation. This means:

1. The covariance is **rank-deficient** (4 components, only 3 DoF — the 4th direction is always constrained)
2. The KF gain $\mathbf{K}$ will try to correct $\mathbf{q}$ in all 4 directions, breaking the norm constraint
3. Renormalizing after every update destroys the covariance's probabilistic meaning

**Why this matters more than just nonlinearity:** even if you linearize $\mathbf{f}$ perfectly, the covariance update $\mathbf{P} \leftarrow (\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ still operates in $\mathbb{R}^4$ on a quantity that only has 3 free dimensions. The filter is structurally inconsistent.

---

### 5.3 The Error-State EKF — Solution for Both Problems

The Error-State EKF (ESKF) solves both the nonlinearity and the manifold problems simultaneously by splitting the state into two parts:

$$\mathbf{x}_\text{true} = \mathbf{x}_\text{nom} \boxplus \delta\mathbf{x}$$

| Component | Lives in | Updated how | Role |
|---|---|---|---|
| **Nominal state** $\mathbf{x}_\text{nom}$ | Manifold (SO(3) × $\mathbb{R}^n$) | Full nonlinear kinematics | Best current guess |
| **Error state** $\delta\mathbf{x}$ | Tangent space $\mathbb{R}^{n_\delta}$ | Linear Gaussian KF | Uncertainty around guess |

For attitude specifically:
- **Nominal:** quaternion $\mathbf{q}_\text{nom}$ — always on the unit sphere, propagated via $\mathbf{q}\otimes\Delta\mathbf{q}$
- **Error:** rotation vector $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ — the small rotation that takes $\mathbf{q}_\text{nom}$ to $\mathbf{q}_\text{true}$:

$$\mathbf{q}_\text{true} = \mathbf{q}_\text{nom}\otimes\delta\mathbf{q}(\delta\boldsymbol{\theta})\,, \qquad \delta\mathbf{q}(\delta\boldsymbol{\theta})\approx\begin{bmatrix}1\\\delta\boldsymbol{\theta}/2\end{bmatrix} \quad\text{(valid quaternion, on sphere)}$$

The key insight: $\delta\boldsymbol{\theta}$ is **always small** (it is zeroed at every update), so the linearization of the error dynamics around $\delta\boldsymbol{\theta}=\mathbf{0}$ is always accurate — regardless of how large the nominal attitude is.

**Comparison — naive EKF on quaternion vs ESKF:**

| Aspect | Naive EKF on $\mathbf{q}\in\mathbb{R}^4$ | ESKF on $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ |
|---|---|---|
| Covariance size | 4×4 (rank-deficient, wasted dimension) | 3×3 (full rank, minimal) |
| Norm constraint | Violated by KF update; requires ad hoc renorm | Always satisfied by construction ($\mathbf{q}_\text{nom}$ stays on sphere) |
| Linearization point | Around current $\mathbf{q}$ (can be far from truth) | Always around $\delta\mathbf{x}=\mathbf{0}$ (small by design) |
| Accuracy for large attitude | Poor (large perturbation → bad linearization) | Good ($\delta\boldsymbol{\theta}$ is always small) |
| Additive update validity | No (addition leaves the sphere) | Yes ($\delta\boldsymbol{\theta}$ lives in $\mathbb{R}^3$) |

The ESKF achieves the best of both worlds: **nonlinear accuracy for the nominal state** + **linear Gaussian tractability for the error covariance**.

---

### 5.4 Why Exactly 24 States?

The 24-dimensional error state is not arbitrary — it is the result of a deliberate engineering trade-off between **estimation fidelity** and **computational cost**.

**Computational cost scales as $O(n^2)$:** the covariance $\mathbf{P}$ has $n^2$ elements; storing and updating it at 250 Hz on an embedded ARM CPU constrains $n$. At $n=24$: $24^2 = 576$ elements (float32: 2.25 KB). At $n=50$: $50^2 = 2500$ elements. PX4 must run on Pixhawk-class hardware at 250 Hz with other tasks running simultaneously.

**Each group of states is included because it is directly observable through the available sensors, cannot be ignored, and improves navigation accuracy enough to justify the cost:**

**Group 1 — Navigation core (9 states: attitude + velocity + position):**

These are the minimum states for dead reckoning. Without them, you cannot integrate IMU data into a position estimate at all.

$$\underbrace{\delta\boldsymbol{\theta}}_{3} + \underbrace{\delta\mathbf{v}}_{3} + \underbrace{\delta\mathbf{p}}_{3} = 9 \text{ states}$$

**Group 2 — IMU biases (6 states: gyro bias + accel bias):**

IMU biases are the primary source of INS drift. Without estimating them online, position error grows as $\frac{1}{2}b_a t^2$ (accelerometer bias integrates twice). At $b_a = 0.05$ m/s² (typical MEMS), position error after 10 seconds = 2.5 m — unacceptable for navigation.

*Observable via:* GPS position fix allows the filter to back-compute what IMU bias must have caused the accumulated error. The bias states converge when GPS is available and remain estimated during GPS outages.

$$\underbrace{\delta\mathbf{b}_g}_{3} + \underbrace{\delta\mathbf{b}_a}_{3} = 6 \text{ states}$$

**Group 3 — Magnetic field model (6 states: inertial + body frame):**

Magnetic field $\mathbf{m}_I\in\mathbb{R}^3$ varies by geographic location (unknown a priori) and is distorted by the vehicle's own magnetic materials (motors, ESCs) captured in $\mathbf{m}_B\in\mathbb{R}^3$. Without these states, the magnetometer can only give a noisy heading estimate with a fixed world-field assumption.

*Observable via:* magnetometer readings fused over time as attitude changes, allowing separation of $\mathbf{m}_I$ and $\mathbf{m}_B$.

$$\underbrace{\delta\mathbf{m}_I}_{3} + \underbrace{\delta\mathbf{m}_B}_{3} = 6 \text{ states}$$

**Group 4 — Wind velocity (2 states: NE components):**

Wind affects airspeed-based navigation (fixed-wing). Without wind estimation, airspeed and ground speed cannot be reconciled. The vertical wind component is assumed zero (standard assumption for low-altitude flight).

*Observable via:* airspeed sensor + GPS ground speed — their difference reveals wind.

$$\underbrace{\delta\mathbf{w}}_{\text{NE}} = 2 \text{ states}$$

**Group 5 — Terrain height (1 state):**

The absolute terrain height above the home point, used for rangefinder-based altitude and terrain following. Without this state, rangefinder data cannot be fused without knowing the terrain.

*Observable via:* downward-facing rangefinder (measures distance to terrain directly).

$$\underbrace{\delta h}_{1} = 1 \text{ state}$$

**Total:** $9 + 6 + 6 + 2 + 1 = 24$

**What was deliberately excluded:**

| Excluded state | Reason |
|---|---|
| Barometer bias | Modeled as part of measurement noise $R_\text{baro}$; too slowly observable to add a state |
| GPS receiver clock error | Handled by GPS receiver internally |
| Atmospheric pressure model | Too low impact for the added cost |
| Higher-order IMU noise (Allan variance) | Captured adequately by the random-walk model |
| Scale factor errors | Assumed calibrated; adding them makes the filter unobservable without specific maneuvers |

---

### 5.5 Continuous-Time Model vs Discrete — Why We Don't "Write State Equations and Estimate"

**The question:** why not derive continuous-time state equations $\dot{\mathbf{x}} = \mathbf{F}_c\mathbf{x} + \mathbf{G}_c\mathbf{w}$ and apply the continuous-time Kalman filter (Riccati equation)?

**The continuous-time error-state model does exist.** Differentiating the error kinematics gives:

$$\dot{\delta\boldsymbol{\theta}} = -[\boldsymbol{\omega}_c]_\times\,\delta\boldsymbol{\theta} - \delta\mathbf{b}_g + \mathbf{n}_g$$
$$\dot{\delta\mathbf{v}} = -\mathbf{R}_\text{nom}[\mathbf{a}_b]_\times\,\delta\boldsymbol{\theta} - \mathbf{R}_\text{nom}\,\delta\mathbf{b}_a + \mathbf{n}_a$$
$$\dot{\delta\mathbf{p}} = \delta\mathbf{v}$$
$$\dot{\delta\mathbf{b}}_g = \mathbf{w}_{bg}\,, \quad \dot{\delta\mathbf{b}}_a = \mathbf{w}_{ba}\,, \;\ldots$$

Stacked: $\dot{\delta\mathbf{x}} = \mathbf{F}_c\,\delta\mathbf{x} + \mathbf{G}_c\,\mathbf{w}$

where $\mathbf{F}_c\in\mathbb{R}^{24\times 24}$ is the **continuous-time state matrix** and $\mathbf{w}$ is continuous white noise.

**Three reasons why PX4 uses discrete instead of continuous:**

**Reason 1 — IMU data is inherently discrete.** The IMU does not output a continuous signal. It outputs **pre-integrated samples**: $\Delta\boldsymbol{\phi} = \int_{t_k}^{t_{k+1}}\boldsymbol{\omega}\,dt$ (delta angle) and $\Delta\mathbf{v} = \int_{t_k}^{t_{k+1}}\mathbf{a}\,dt$ (delta velocity) at fixed intervals. There is no continuous signal to feed into a continuous-time filter.

**Reason 2 — Exact discretization is too expensive.** The exact discrete-time equivalent of $\dot{\mathbf{x}} = \mathbf{F}_c\mathbf{x}$ is:

$$\mathbf{x}_{k+1} = \underbrace{e^{\mathbf{F}_c\Delta t}}_{\mathbf{F}_d}\mathbf{x}_k$$

Computing the **matrix exponential** $e^{\mathbf{F}_c\Delta t}$ (a $24\times 24$ matrix) at every IMU sample (250–1000 Hz) is impractical on embedded hardware. It requires either Padé approximation or eigendecomposition — both $O(n^3)$ per step.

**Reason 3 — First-order approximation is accurate enough.** For small $\Delta t$:

$$e^{\mathbf{F}_c\Delta t} = \mathbf{I} + \mathbf{F}_c\Delta t + \frac{(\mathbf{F}_c\Delta t)^2}{2!} + \ldots \approx \mathbf{I} + \mathbf{F}_c\Delta t$$

The truncation error is $O(\|\mathbf{F}_c\|^2\Delta t^2)$. At 1 kHz with typical drone dynamics ($\|\mathbf{F}_c\|\lesssim 10$ rad/s), $\|\mathbf{F}_c\|^2\Delta t^2 \approx 10^{-4}$ — less than 0.01% per step.

**The connection between continuous $\mathbf{F}_c$ and discrete $\mathbf{F}_k$:**

| | Continuous | Discrete (EKF) |
|---|---|---|
| State equation | $\dot{\delta\mathbf{x}} = \mathbf{F}_c\,\delta\mathbf{x}$ | $\delta\mathbf{x}_{k+1} = \mathbf{F}_k\,\delta\mathbf{x}_k$ |
| State matrix | $\mathbf{F}_c = \partial\dot{\mathbf{x}}/\partial\mathbf{x}$ (constant entries like $-[\boldsymbol{\omega}]_\times$) | $\mathbf{F}_k \approx \mathbf{I} + \mathbf{F}_c\,\Delta t$ |
| Covariance equation | $\dot{\mathbf{P}} = \mathbf{F}_c\mathbf{P} + \mathbf{P}\mathbf{F}_c^\top + \mathbf{Q}_c$ (Riccati) | $\mathbf{P}_{k+1} = \mathbf{F}_k\mathbf{P}_k\mathbf{F}_k^\top + \mathbf{Q}$ |
| Process noise | $\mathbf{Q}_c$ (PSD, units: units²/Hz) | $\mathbf{Q} \approx \mathbf{G}_c\mathbf{Q}_c\mathbf{G}_c^\top\Delta t$ |
| Correction step | Continuous measurement update | Discrete measurement update |

**So PX4 does exactly what the theory says** — it writes the continuous-time state equations, then discretizes for implementation. The difference from textbook examples is that: (a) the equations are nonlinear so $\mathbf{F}_c$ is recomputed at each step; (b) discretization uses the first-order approximation $\mathbf{F}_d \approx \mathbf{I} + \mathbf{F}_c\Delta t$ because it is accurate and cheap.

---

### 5.6 State Vector x — Every Component Defined, Mapped to Code

The ESKF tracks an **error state** $\delta\mathbf{x}\in\mathbb{R}^{24}$ — the perturbation of the true state around the nominal trajectory. This is the vector whose covariance $\mathbf{P}$ (24×24) the filter maintains.

**The 24-dimensional error state vector:**

$$\delta\mathbf{x} = \begin{bmatrix}
\delta\boldsymbol{\theta} \\ \delta\mathbf{v} \\ \delta\mathbf{p} \\ \delta\mathbf{b}_g \\ \delta\mathbf{b}_a \\ \delta\mathbf{m}_I \\ \delta\mathbf{m}_B \\ \delta\mathbf{w} \\ \delta h
\end{bmatrix}
\in\mathbb{R}^{24}$$

| Component | Indices | Size | Physical meaning | Code field |
|---|---|---|---|---|
| $\delta\boldsymbol{\theta}$ | 0–2 | 3 | Attitude error (rotation vector, body frame) | `State::quat_nominal` (3 DoF in error state) |
| $\delta\mathbf{v}$ | 3–5 | 3 | NED velocity error (m/s) | `State::vel` |
| $\delta\mathbf{p}$ | 6–8 | 3 | NED position error (m) | `State::pos` |
| $\delta\mathbf{b}_g$ | 9–11 | 3 | Gyroscope bias error (rad/s) | `State::gyro_bias` |
| $\delta\mathbf{b}_a$ | 12–14 | 3 | Accelerometer bias error (m/s²) | `State::accel_bias` |
| $\delta\mathbf{m}_I$ | 15–17 | 3 | Inertial magnetic field error (Gauss) | `State::mag_I` |
| $\delta\mathbf{m}_B$ | 18–20 | 3 | Body magnetic field error (Gauss) | `State::mag_B` |
| $\delta\mathbf{w}$ | 21–22 | 2 | Wind velocity error NE (m/s) | `State::wind_vel` |
| $\delta h$ | 23 | 1 | Terrain height error AGL (m) | `State::terrain` |

```cpp
// EKF/python/ekf_derivation/generated/state.h — auto-generated
namespace State {
    static constexpr IdxDof quat_nominal{0,  3};  // 3 DoF δθ
    static constexpr IdxDof vel        {3,  3};
    static constexpr IdxDof pos        {6,  3};
    static constexpr IdxDof gyro_bias  {9,  3};
    static constexpr IdxDof accel_bias {12, 3};
    static constexpr IdxDof mag_I      {15, 3};
    static constexpr IdxDof mag_B      {18, 3};
    static constexpr IdxDof wind_vel   {21, 2};
    static constexpr IdxDof terrain    {23, 1};
    static constexpr uint8_t size{24};
};
```

The **covariance matrix** $\mathbf{P}$ is therefore $24\times 24$. Entry $P_{ij}$ is the covariance between error component $i$ and error component $j$. For example, $P_{3,0}$ is the covariance between north velocity error $\delta v_N$ and roll error $\delta\theta_x$ — these become correlated when accelerometer data is fused.

The **nominal state** runs in parallel as a separate `StateSample` struct (quaternion, vel, pos, biases in their actual physical representations). Only the covariance lives in the 24-dimensional error space.

---

### 5.7 The Nonlinear Process Model f(x) — Full IMU Kinematics

Each IMU sample brings: $\tilde{\boldsymbol{\omega}}$ (gyro measurement) and $\tilde{\mathbf{a}}$ (accelerometer measurement), both in the body frame, over time interval $\Delta t$.

The **true IMU measurements** include bias and noise:

$$\tilde{\boldsymbol{\omega}} = \boldsymbol{\omega}_\text{true} + \mathbf{b}_g + \mathbf{n}_g\,, \qquad \tilde{\mathbf{a}} = \mathbf{a}_\text{true} + \mathbf{b}_a + \mathbf{n}_a$$

where $\mathbf{n}_g\sim\mathcal{N}(\mathbf{0},\sigma_g^2\mathbf{I})$ and $\mathbf{n}_a\sim\mathcal{N}(\mathbf{0},\sigma_a^2\mathbf{I})$ are white noise processes.

The **corrected IMU** (bias-subtracted) measurements:

$$\boldsymbol{\omega}_c = \tilde{\boldsymbol{\omega}} - \mathbf{b}_g \approx \boldsymbol{\omega}_\text{true} + \mathbf{n}_g\,, \qquad \mathbf{a}_c = \tilde{\mathbf{a}} - \mathbf{b}_a \approx \mathbf{a}_\text{true} + \mathbf{n}_a$$

The **nonlinear nominal state propagation** (no noise — this is the deterministic part):

**Equation F1 — Attitude kinematics (quaternion):**

$$\mathbf{q}_{k+1} = \mathbf{q}_k \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_c \,\Delta t)$$

where $\Delta\mathbf{q}(\boldsymbol{\phi})$ is the unit quaternion for rotation vector $\boldsymbol{\phi}$:

$$\Delta\mathbf{q}(\boldsymbol{\phi}) = \begin{bmatrix}\cos\!\left(\|\boldsymbol{\phi}\|/2\right) \\ \sin\!\left(\|\boldsymbol{\phi}\|/2\right)\cdot\hat{\boldsymbol{\phi}}\end{bmatrix}\,, \qquad \hat{\boldsymbol{\phi}} = \boldsymbol{\phi}/\|\boldsymbol{\phi}\|$$

For small $\|\boldsymbol{\phi}\| = \|\boldsymbol{\omega}_c\|\Delta t \ll 1$: $\Delta\mathbf{q} \approx \begin{bmatrix}1 \\ \boldsymbol{\phi}/2\end{bmatrix}$ (first-order).

**In PX4 code** (`ekf.cpp: predictState()`):
```cpp
const Quatf dq(AxisAnglef{corrected_delta_ang});        // Δq from rotation vector
_state.quat_nominal = (_state.quat_nominal * dq).normalized();
```

---

**Equation F2 — Velocity kinematics (NED frame):**

$$\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{R}(\mathbf{q}_k)\,\mathbf{a}_c\,\Delta t + \mathbf{g}\,\Delta t$$

where:
- $\mathbf{R}(\mathbf{q}_k)\in\mathbb{R}^{3\times 3}$ is the rotation matrix from body to NED frame (DCM), constructed from the current quaternion
- $\mathbf{a}_c$ is the corrected specific force **in the body frame**
- $\mathbf{g} = [0,\, 0,\, +g]^\top$ in NED convention ($+Z$ pointing down, $g \approx 9.81$ m/s²)

The rotation matrix $\mathbf{R}(\mathbf{q})$ in terms of quaternion components $[q_w, q_x, q_y, q_z]$:

$$\mathbf{R}(\mathbf{q}) = \begin{bmatrix}
1 - 2(q_y^2+q_z^2) & 2(q_xq_y - q_wq_z) & 2(q_xq_z + q_wq_y) \\
2(q_xq_y + q_wq_z) & 1 - 2(q_x^2+q_z^2) & 2(q_yq_z - q_wq_x) \\
2(q_xq_z - q_wq_y) & 2(q_yq_z + q_wq_x) & 1 - 2(q_x^2+q_y^2)
\end{bmatrix}$$

This is the source of nonlinearity — $\mathbf{R}$ is a quadratic function of the quaternion components.

**In PX4 code:**
```cpp
_R_to_earth = Dcmf(_state.quat_nominal);           // build R from q
_state.vel += _R_to_earth * corrected_delta_vel;   // R(q)·a·Δt
_state.vel += gravity_acceleration * dt;            // + g·Δt
```

---

**Equation F3 — Position kinematics:**

$$\mathbf{p}_{k+1} = \mathbf{p}_k + \tfrac{1}{2}(\mathbf{v}_k + \mathbf{v}_{k+1})\,\Delta t$$

(Trapezoidal integration — more accurate than Euler for velocity.)

**In PX4 code:**
```cpp
_gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
```

---

**Equations F4–F8 — Bias and auxiliary states:**

All biases and auxiliary states are modeled as **random walks** (no deterministic drift):

$$\mathbf{b}_{g,k+1} = \mathbf{b}_{g,k}\,, \quad \mathbf{b}_{a,k+1} = \mathbf{b}_{a,k}\,, \quad \mathbf{m}_{I,k+1} = \mathbf{m}_{I,k}\,, \quad \mathbf{m}_{B,k+1} = \mathbf{m}_{B,k}\,, \quad \mathbf{w}_{k+1} = \mathbf{w}_k\,, \quad h_{k+1} = h_k$$

The randomness (how biases drift) is captured in the process noise $\mathbf{Q}$, not in the mean evolution.

---

### 5.8 From f(x) to the EKF: Taylor Expansion Step by Step

**The core idea:** the true state is $\mathbf{x}_\text{true} = \mathbf{x}_\text{nom} \boxplus \delta\mathbf{x}$, where $\delta\mathbf{x}$ is small. We want to find how $\delta\mathbf{x}$ evolves one step forward.

**Setup notation:**

- Nominal propagation: $\mathbf{x}_\text{nom,k+1} = \mathbf{f}(\mathbf{x}_\text{nom,k})$ — exact nonlinear, run by `predictState()`
- True propagation: $\mathbf{x}_\text{true,k+1} = \mathbf{f}(\mathbf{x}_\text{nom,k} \boxplus \delta\mathbf{x}_k) + \mathbf{G}\mathbf{n}$
- Error at step $k+1$: $\delta\mathbf{x}_{k+1} = \mathbf{x}_\text{true,k+1} \ominus \mathbf{x}_\text{nom,k+1}$

For small $\delta\mathbf{x}_k$, Taylor-expand $\mathbf{f}(\mathbf{x}_\text{nom} \boxplus \delta\mathbf{x})$ around $\delta\mathbf{x} = \mathbf{0}$:

$$\mathbf{f}(\mathbf{x}_\text{nom} \boxplus \delta\mathbf{x}) \approx \mathbf{f}(\mathbf{x}_\text{nom}) + \underbrace{\left.\frac{\partial\,[\mathbf{f}(\mathbf{x}_\text{nom}\boxplus\delta\mathbf{x})\ominus\mathbf{f}(\mathbf{x}_\text{nom})]}{\partial\,\delta\mathbf{x}}\right|_{\delta\mathbf{x}=\mathbf{0}}}_{\mathbf{F}_k,\;24\times 24}\delta\mathbf{x}_k$$

This gives the **linearized error dynamics**:

$$\boxed{\delta\mathbf{x}_{k+1} \approx \mathbf{F}_k\,\delta\mathbf{x}_k + \mathbf{G}_k\,\mathbf{n}_k}$$

where:
- $\mathbf{F}_k$ is the **state transition Jacobian** (24×24) — how error propagates
- $\mathbf{G}_k$ is the **noise influence matrix** — how IMU noise enters each state
- $\mathbf{n}_k \sim \mathcal{N}(\mathbf{0}, \mathbf{Q}_c)$ is the raw process noise vector

By Property 1 (linear transform of Gaussian) and Property 2 (sum of Gaussians), the error covariance propagates as:

$$\boxed{\mathbf{P}_{k+1} = \mathbf{F}_k\,\mathbf{P}_k\,\mathbf{F}_k^\top + \mathbf{G}_k\,\mathbf{Q}_c\,\mathbf{G}_k^\top}$$

The term $\mathbf{G}_k\,\mathbf{Q}_c\,\mathbf{G}_k^\top$ is typically absorbed into a single discrete process noise matrix $\mathbf{Q}$:

$$\mathbf{P}_{k+1} = \mathbf{F}_k\,\mathbf{P}_k\,\mathbf{F}_k^\top + \mathbf{Q}$$

**In PX4 code**, `predictCovariance()` in `covariance.cpp` computes this using auto-generated SymPy expressions (the explicit $\mathbf{F}$ and $\mathbf{Q}$ derivations below are exactly what that script produces).

---

### 5.9 Computing the Jacobian F — Block-by-Block Derivation

$\mathbf{F}_k$ is a $24\times 24$ matrix. Most off-diagonal blocks are zero; only a few cross-couplings are non-zero. We derive each non-zero block from the kinematics equations (F1–F8).

**Notation:**
- $\mathbf{R}_\text{nom} \equiv \mathbf{R}(\mathbf{q}_\text{nom})$: rotation matrix from body to NED at the current nominal attitude
- $\mathbf{a}_b \equiv \mathbf{a}_c = \tilde{\mathbf{a}} - \mathbf{b}_{a,\text{nom}}$: corrected specific force in body frame
- $[\mathbf{u}]_\times$: $3\times 3$ skew-symmetric matrix of vector $\mathbf{u}$, defined so that $[\mathbf{u}]_\times \mathbf{v} = \mathbf{u}\times\mathbf{v}$
- $\Delta\boldsymbol{\phi} = \boldsymbol{\omega}_c\,\Delta t$: nominal rotation angle vector (small for small $\Delta t$)

---

#### Block (0–2, 0–2): $\mathbf{F}_{\theta\theta} = \partial\,\delta\boldsymbol{\theta}_{k+1}/\partial\,\delta\boldsymbol{\theta}_k$

**Setup:** The true quaternion is $\mathbf{q}_\text{true} = \mathbf{q}_\text{nom}\otimes\delta\mathbf{q}$, where for small $\delta\boldsymbol{\theta}$:

$$\delta\mathbf{q} \approx \begin{bmatrix}1\\\delta\boldsymbol{\theta}/2\end{bmatrix}$$

After one IMU step with corrected angular rate $\boldsymbol{\omega}_c - \delta\mathbf{b}_g$:

- True:    $\mathbf{q}_\text{true,k+1} = \mathbf{q}_\text{true,k} \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_c\Delta t - \delta\mathbf{b}_g\Delta t)$
- Nominal: $\mathbf{q}_\text{nom,k+1} = \mathbf{q}_\text{nom,k} \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_c\Delta t)$

The **error quaternion at step $k+1$**:

$$\delta\mathbf{q}_{k+1} = \mathbf{q}_\text{nom,k+1}^{-1}\otimes\mathbf{q}_\text{true,k+1}$$

Substituting both:

$$\delta\mathbf{q}_{k+1} = \underbrace{\Delta\mathbf{q}(\Delta\boldsymbol{\phi})^{-1}}_{\Delta\mathbf{q}(-\Delta\boldsymbol{\phi})}\otimes\underbrace{\mathbf{q}_\text{nom,k}^{-1}\otimes\mathbf{q}_\text{nom,k}}_{=\mathbf{1}}\otimes\delta\mathbf{q}_k\otimes\Delta\mathbf{q}(\Delta\boldsymbol{\phi} - \delta\mathbf{b}_g\Delta t)$$

$$= \Delta\mathbf{q}(-\Delta\boldsymbol{\phi})\otimes\delta\mathbf{q}_k\otimes\Delta\mathbf{q}(\Delta\boldsymbol{\phi})\otimes\underbrace{\Delta\mathbf{q}(-\delta\mathbf{b}_g\Delta t)}_{\approx\,[1,\,-\delta\mathbf{b}_g\Delta t/2]^\top}$$

The product $\Delta\mathbf{q}(-\Delta\boldsymbol{\phi})\otimes\delta\mathbf{q}_k\otimes\Delta\mathbf{q}(\Delta\boldsymbol{\phi})$ is a **conjugation** of $\delta\mathbf{q}$ by $\Delta\mathbf{q}$. In rotation-vector terms, this rotates the error vector by the nominal rotation $\Delta\boldsymbol{\phi}$:

$$\delta\boldsymbol{\theta}\;\xrightarrow{\text{conjugation by }\Delta\mathbf{q}}\;\mathbf{R}(\Delta\mathbf{q}(-\Delta\boldsymbol{\phi}))\cdot\delta\boldsymbol{\theta} = \mathbf{R}(\Delta\boldsymbol{\phi})^\top\cdot\delta\boldsymbol{\theta}$$

For small $\Delta\boldsymbol{\phi}$, $\mathbf{R}(\Delta\boldsymbol{\phi})^\top \approx \mathbf{I} - [\Delta\boldsymbol{\phi}]_\times$. So:

$$\delta\boldsymbol{\theta}_{k+1} \approx (\mathbf{I} - [\Delta\boldsymbol{\phi}]_\times)\,\delta\boldsymbol{\theta}_k - \Delta t\,\delta\mathbf{b}_g$$

$$\boxed{\mathbf{F}_{\theta\theta} = \mathbf{I} - [\Delta\boldsymbol{\phi}]_\times\,, \qquad \Delta\boldsymbol{\phi} = \boldsymbol{\omega}_c\,\Delta t}$$

For IMU rate $\geq 250$ Hz, $\Delta t \leq 4$ ms, and typical drone rates $\|\boldsymbol{\omega}\|\leq 10$ rad/s, so $\|\Delta\boldsymbol{\phi}\| \leq 0.04$ rad — $[\Delta\boldsymbol{\phi}]_\times$ is at most a 4% correction to the identity. At low rates (e.g., 100 Hz), this term matters.

---

#### Block (0–2, 9–11): $\mathbf{F}_{\theta b_g} = \partial\,\delta\boldsymbol{\theta}_{k+1}/\partial\,\delta\mathbf{b}_g$

From the same derivation above, the gyro bias error contribution to the attitude error is:

$$\delta\boldsymbol{\theta}_{k+1} \supset -\Delta t\,\delta\mathbf{b}_g$$

$$\boxed{\mathbf{F}_{\theta b_g} = -\Delta t\,\mathbf{I}_{3\times 3}}$$

**Physical meaning:** A gyro bias error of 1 rad/s causes attitude to drift at 1 rad/s — it directly drives the attitude error. After $\Delta t$ seconds, the accumulated error is $\Delta t\,\delta\mathbf{b}_g$.

---

#### Block (3–5, 0–2): $\mathbf{F}_{v\theta} = \partial\,\delta\mathbf{v}_{k+1}/\partial\,\delta\boldsymbol{\theta}_k$

**Setup:** The velocity update is $\mathbf{v}_{k+1} = \mathbf{v}_k + \mathbf{R}(\mathbf{q}_k)\,\mathbf{a}_b\,\Delta t + \mathbf{g}\,\Delta t$.

True and nominal:

$$\delta\mathbf{v}_{k+1} = \delta\mathbf{v}_k + [\mathbf{R}(\mathbf{q}_\text{true,k}) - \mathbf{R}(\mathbf{q}_\text{nom,k})]\,\mathbf{a}_b\,\Delta t - \mathbf{R}_\text{nom}\,\delta\mathbf{b}_a\,\Delta t$$

For the attitude perturbation (right perturbation, $\delta\boldsymbol{\theta}$ in body frame):

$$\mathbf{R}(\mathbf{q}_\text{true}) = \mathbf{R}(\mathbf{q}_\text{nom}\otimes\delta\mathbf{q}) \approx \mathbf{R}_\text{nom}\cdot(\mathbf{I} + [\delta\boldsymbol{\theta}]_\times)$$

Therefore:

$$\mathbf{R}(\mathbf{q}_\text{true})\,\mathbf{a}_b - \mathbf{R}_\text{nom}\,\mathbf{a}_b \approx \mathbf{R}_\text{nom}\,[\delta\boldsymbol{\theta}]_\times\,\mathbf{a}_b$$

Using the vector identity $[\mathbf{u}]_\times\,\mathbf{v} = \mathbf{u}\times\mathbf{v} = -\mathbf{v}\times\mathbf{u} = -[\mathbf{v}]_\times\,\mathbf{u}$:

$$\mathbf{R}_\text{nom}\,[\delta\boldsymbol{\theta}]_\times\,\mathbf{a}_b = -\mathbf{R}_\text{nom}\,[\mathbf{a}_b]_\times\,\delta\boldsymbol{\theta}$$

So the velocity error contribution from attitude error:

$$\delta\mathbf{v}_{k+1} \supset -\mathbf{R}_\text{nom}\,[\mathbf{a}_b]_\times\,\delta\boldsymbol{\theta}\,\Delta t$$

$$\boxed{\mathbf{F}_{v\theta} = -\mathbf{R}_\text{nom}\,[\mathbf{a}_b]_\times\,\Delta t\,, \qquad \mathbf{a}_b = \tilde{\mathbf{a}} - \mathbf{b}_{a,\text{nom}}}$$

The $3\times 3$ matrix $[\mathbf{a}_b]_\times$ (skew-symmetric):

$$[\mathbf{a}_b]_\times = \begin{bmatrix}0 & -a_{b,z} & a_{b,y} \\ a_{b,z} & 0 & -a_{b,x} \\ -a_{b,y} & a_{b,x} & 0\end{bmatrix}$$

**Physical meaning:** An attitude error $\delta\boldsymbol{\theta}$ causes the accelerometer to be mis-projected into the NED frame, creating a spurious velocity change. The stronger the specific force $\mathbf{a}_b$ (e.g., during aggressive maneuvers), the larger this coupling.

**In PX4 code** (`covariance.cpp` generated expressions):
```cpp
// Auto-generated from derivation.py — computes -R*[ab]x*dt explicitly
// stored as F_vel_ang = -R_to_earth * skew(corrected_accel) * dt
```

---

#### Block (3–5, 3–5): $\mathbf{F}_{vv} = \partial\,\delta\mathbf{v}_{k+1}/\partial\,\delta\mathbf{v}_k$

Velocity update depends on previous velocity additively:

$$\delta\mathbf{v}_{k+1} \supset \delta\mathbf{v}_k$$

$$\boxed{\mathbf{F}_{vv} = \mathbf{I}_{3\times 3}}$$

---

#### Block (3–5, 12–14): $\mathbf{F}_{vb_a} = \partial\,\delta\mathbf{v}_{k+1}/\partial\,\delta\mathbf{b}_a$

The accelerometer bias error enters velocity through the rotation:

$$\delta\mathbf{v}_{k+1} \supset -\mathbf{R}_\text{nom}\,\delta\mathbf{b}_a\,\Delta t$$

$$\boxed{\mathbf{F}_{vb_a} = -\mathbf{R}_\text{nom}\,\Delta t}$$

**Physical meaning:** An accelerometer bias error of $\delta b_a$ m/s² causes velocity to drift at $R_\text{nom}\,\delta b_a$ m/s (after rotating to NED frame). This is the most important bias coupling for position accuracy.

---

#### Block (6–8, 3–5): $\mathbf{F}_{pv} = \partial\,\delta\mathbf{p}_{k+1}/\partial\,\delta\mathbf{v}_k$

Position update: $\mathbf{p}_{k+1} = \mathbf{p}_k + \frac{1}{2}(\mathbf{v}_k + \mathbf{v}_{k+1})\,\Delta t$

The velocity error $\delta\mathbf{v}_k$ contributes to position error:

$$\delta\mathbf{p}_{k+1} \supset \delta\mathbf{v}_k\,\Delta t$$

(The factor $\frac{1}{2}$ from trapezoidal integration applies to both $\delta\mathbf{v}_k$ and $\delta\mathbf{v}_{k+1}$, but to first order in $\Delta t$:)

$$\boxed{\mathbf{F}_{pv} = \Delta t\,\mathbf{I}_{3\times 3}}$$

---

#### Block (6–8, 6–8): $\mathbf{F}_{pp} = \mathbf{I}$

Position error carries forward unchanged.

---

#### Block (6–8, 0–2): $\mathbf{F}_{p\theta}$

Position update depends on $\mathbf{v}_{k+1}$ which depends on $\delta\boldsymbol{\theta}_k$ through $\mathbf{F}_{v\theta}$. The second-order contribution $\frac{1}{2}\mathbf{F}_{v\theta}\Delta t^2$ is negligible at high IMU rates and is typically set to zero in the discrete model.

$$\mathbf{F}_{p\theta} \approx \mathbf{0} \quad\text{(second order in }\Delta t\text{)}$$

---

#### Bias diagonal blocks: $\mathbf{F}_{b_g b_g} = \mathbf{F}_{b_a b_a} = \mathbf{I}$

Biases are random walks — they evolve only through process noise, not through deterministic dynamics:

$$\delta\mathbf{b}_{g,k+1} = \delta\mathbf{b}_{g,k}\,, \quad \delta\mathbf{b}_{a,k+1} = \delta\mathbf{b}_{a,k}$$

$$\boxed{\mathbf{F}_{b_g b_g} = \mathbf{F}_{b_a b_a} = \mathbf{I}_{3\times 3}}$$

Similarly for $\mathbf{m}_I$, $\mathbf{m}_B$, $\mathbf{w}$, $h$: all diagonal blocks are $\mathbf{I}$, all off-diagonal blocks are $\mathbf{0}$.

---

### 5.10 The Full F Matrix (24×24) — Complete Structure

Below is the full $24\times 24$ Jacobian. Only non-zero off-diagonal blocks are shown; diagonal blocks are $\mathbf{I}$ (identity of appropriate size); all other blocks are $\mathbf{0}$:

$$\mathbf{F}_k = \begin{bmatrix}
\underbrace{\mathbf{I}-[\Delta\boldsymbol{\phi}]_\times}_{\mathbf{F}_{\theta\theta}} & \mathbf{0} & \mathbf{0} & \underbrace{-\Delta t\,\mathbf{I}}_{\mathbf{F}_{\theta b_g}} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\underbrace{-\mathbf{R}[\mathbf{a}_b]_\times\Delta t}_{\mathbf{F}_{v\theta}} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \underbrace{-\mathbf{R}\Delta t}_{\mathbf{F}_{vb_a}} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \underbrace{\Delta t\,\mathbf{I}}_{\mathbf{F}_{pv}} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{I} & \mathbf{0} \\
\mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & \mathbf{0} & 1
\end{bmatrix}$$

where row/column blocks correspond to $[\delta\boldsymbol{\theta}|\,\delta\mathbf{v}|\,\delta\mathbf{p}|\,\delta\mathbf{b}_g|\,\delta\mathbf{b}_a|\,\delta\mathbf{m}_I|\,\delta\mathbf{m}_B|\,\delta\mathbf{w}|\,\delta h]$ and $\mathbf{R} \equiv \mathbf{R}_\text{nom}$.

**Summary of non-zero off-diagonal blocks:**

| Block | Indices (row, col) | Formula | Physical coupling |
|---|---|---|---|
| $\mathbf{F}_{\theta b_g}$ | (0–2, 9–11) | $-\Delta t\,\mathbf{I}$ | Gyro bias → attitude drift |
| $\mathbf{F}_{v\theta}$ | (3–5, 0–2) | $-\mathbf{R}_\text{nom}[\mathbf{a}_b]_\times\Delta t$ | Attitude error → velocity error |
| $\mathbf{F}_{vb_a}$ | (3–5, 12–14) | $-\mathbf{R}_\text{nom}\Delta t$ | Accel bias → velocity drift |
| $\mathbf{F}_{pv}$ | (6–8, 3–5) | $\Delta t\,\mathbf{I}$ | Velocity error → position drift |
| $\mathbf{F}_{\theta\theta}$ | (0–2, 0–2) | $\mathbf{I}-[\Delta\boldsymbol{\phi}]_\times$ | Attitude self-rotation correction |

**How PX4 uses F:** Rather than forming $\mathbf{F}$ as a $24\times 24$ matrix and computing $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ (576 multiplications for F alone, plus 24³ = 13824 for the triple product), the Python script `derivation.py` uses SymPy to symbolically expand $\mathbf{F}\mathbf{P}\mathbf{F}^\top + \mathbf{Q}$ and generates optimized C++ code that directly updates each element $P_{ij}$. This is more efficient and avoids forming the sparse $\mathbf{F}$ at all.

---

### 5.11 The Process Noise Matrix Q

The discrete process noise $\mathbf{Q}$ accounts for two sources:

**1. IMU measurement noise** (integrated into state noise via $\mathbf{G}$):

The IMU noise $[\mathbf{n}_g,\,\mathbf{n}_a]$ enters the state as:
- Gyro noise → attitude error: $\mathbf{G}_\theta = -\mathbf{I}\,\Delta t$ (same path as gyro bias)
- Accel noise → velocity error: $\mathbf{G}_v = -\mathbf{R}_\text{nom}\,\Delta t$

The contribution to $\mathbf{Q}$:

$$\mathbf{Q}_\text{IMU} = \mathbf{G}\begin{bmatrix}\sigma_g^2\mathbf{I}&\mathbf{0}\\\mathbf{0}&\sigma_a^2\mathbf{I}\end{bmatrix}\mathbf{G}^\top = \begin{bmatrix}\sigma_g^2\Delta t^2\,\mathbf{I} & \mathbf{0} \\ \mathbf{0} & \sigma_a^2\Delta t^2\,\mathbf{I} \\ \vdots & \vdots\end{bmatrix}$$

**2. Random-walk noise** (bias stability, magnetic variation):

$$\mathbf{Q}_\text{RW} = \begin{bmatrix}\ddots & & & & \\ & \sigma_{bg}^2\mathbf{I} & & & \\ & & \sigma_{ba}^2\mathbf{I} & & \\ & & & \sigma_{m}^2\mathbf{I} & \\ & & & & \ddots \end{bmatrix}$$

**In PX4 parameters** (set via QGroundControl or parameter files):

| Parameter | Noise source | Feeds into |
|---|---|---|
| `EKF2_GYR_NOISE` | $\sigma_g$ (rad/s/√Hz) | $Q_{\theta\theta}$ |
| `EKF2_ACC_NOISE` | $\sigma_a$ (m/s²/√Hz) | $Q_{vv}$ |
| `EKF2_GYR_B_NOISE` | $\sigma_{bg}$ (rad/s²/√Hz) | $Q_{b_g b_g}$ |
| `EKF2_ACC_B_NOISE` | $\sigma_{ba}$ (m/s³/√Hz) | $Q_{b_a b_a}$ |
| `EKF2_MAG_NOISE` | $\sigma_m$ (Gauss/√Hz) | $Q_{m_I m_I},\,Q_{m_B m_B}$ |

The subscript /√Hz indicates a power spectral density — PX4 multiplies by $\Delta t$ to convert to discrete noise variance.

---

### 5.12 ESKF PREDICT Step — Complete Formula Set

Combining all of the above, the predict step is:

**Step 1 — Propagate nominal state** (nonlinear, exact, zero noise):

$$\hat{\mathbf{q}}_{k+1} = \hat{\mathbf{q}}_k \otimes \Delta\mathbf{q}(\boldsymbol{\omega}_c\,\Delta t) \tag{Eq. F1}$$

$$\hat{\mathbf{v}}_{k+1} = \hat{\mathbf{v}}_k + \mathbf{R}(\hat{\mathbf{q}}_k)\,\mathbf{a}_c\,\Delta t + \mathbf{g}\,\Delta t \tag{Eq. F2}$$

$$\hat{\mathbf{p}}_{k+1} = \hat{\mathbf{p}}_k + \tfrac{1}{2}(\hat{\mathbf{v}}_k + \hat{\mathbf{v}}_{k+1})\,\Delta t \tag{Eq. F3}$$

$$\hat{\mathbf{b}}_{g,k+1} = \hat{\mathbf{b}}_{g,k}\,, \quad \hat{\mathbf{b}}_{a,k+1} = \hat{\mathbf{b}}_{a,k}\,, \;\ldots \tag{Eqs. F4–F8}$$

→ Code: `Ekf::predictState()` in `ekf.cpp`

**Step 2 — Propagate error covariance** (linearized, using Jacobian $\mathbf{F}_k$):

$$\mathbf{P}_{k+1} = \mathbf{F}_k\,\mathbf{P}_k\,\mathbf{F}_k^\top + \mathbf{Q} \tag{Eq. PREDICT-P}$$

where $\mathbf{F}_k$ has the block structure from §5.6.

→ Code: `Ekf::predictCovariance()` in `covariance.cpp` (generated by SymPy)

**Why predict P grows:** The $+\mathbf{Q}$ term always adds positive-definite noise — every IMU step makes us slightly less certain. Without measurements, $\mathbf{P}$ grows without bound (the drone drifts with no correction).

---

### 5.13 Observation Models h(x) and Jacobian H

For the UPDATE step, each sensor defines a measurement function $h(\mathbf{x})$ mapping the **nominal state** to the predicted sensor reading, and the Jacobian $\mathbf{H} = \partial h/\partial\,\delta\mathbf{x}$ maps the **error state** to the predicted measurement error.

**Key examples:**

#### GPS Position Measurement

The GPS reports NED position $\mathbf{z}_\text{GPS} = [p_N, p_E, p_D]^\top$.

Predicted measurement: $\hat{\mathbf{z}} = \hat{\mathbf{p}}_\text{nom}$ (position directly from nominal state).

Jacobian (since $h = \mathbf{p}$ and $\delta\mathbf{p}$ is at indices 6–8):

$$\mathbf{H}_\text{GPS} = \frac{\partial\,\delta\mathbf{p}}{\partial\,\delta\mathbf{x}} = \begin{bmatrix}\mathbf{0}_{3\times 6} & \mathbf{I}_3 & \mathbf{0}_{3\times 15}\end{bmatrix} \quad (3\times 24)$$

The row block $[\mathbf{0}|\mathbf{I}|\mathbf{0}]$ picks out the position error rows (indices 6–8) and ignores all other error states.

---

#### Barometer Altitude Measurement

The baro reports altitude $z_\text{baro} = h$ (scalar). Predicted: $\hat{z} = \hat{p}_D$ (the Down component of position).

Jacobian (scalar measurement, $p_D$ at index 8):

$$\mathbf{H}_\text{baro} = \begin{bmatrix}0,\ldots,0,\underbrace{1}_{\text{index 8}},0,\ldots,0\end{bmatrix} \quad (1\times 24)$$

---

#### Magnetometer Measurement

The magnetometer reports the magnetic field in the body frame $\mathbf{z}_\text{mag} = \mathbf{m}_B^\text{meas}\in\mathbb{R}^3$.

Predicted measurement: $\hat{\mathbf{z}} = \mathbf{R}(\hat{\mathbf{q}})^\top\,\hat{\mathbf{m}}_I + \hat{\mathbf{m}}_B$ (rotate inertial field to body frame, add body-frame anomaly).

The Jacobian is more complex — it involves the derivative of $\mathbf{R}(\hat{\mathbf{q}})^\top\,\mathbf{m}_I$ with respect to $\delta\boldsymbol{\theta}$:

$$\frac{\partial}{\partial\,\delta\boldsymbol{\theta}}\left[\mathbf{R}(\mathbf{q}\otimes\delta\mathbf{q})^\top\,\mathbf{m}_I\right]_{\delta\boldsymbol{\theta}=0} = \left[\mathbf{R}^\top\,\mathbf{m}_I\right]_\times \quad (3\times 3)$$

So: $\mathbf{H}_\text{mag}\big|_{\delta\boldsymbol{\theta}} = [\mathbf{R}^\top\mathbf{m}_I]_\times$ (indices 0–2)

And: $\mathbf{H}_\text{mag}\big|_{\delta\mathbf{m}_I} = \mathbf{R}^\top$ (indices 15–17), $\mathbf{H}_\text{mag}\big|_{\delta\mathbf{m}_B} = \mathbf{I}$ (indices 18–20).

---

### 5.14 ESKF UPDATE Step — Complete Formula Set

When a measurement $\mathbf{z}_k$ arrives:

**Step 1 — Compute innovation:**

$$\boldsymbol{\nu}_k = \mathbf{z}_k - h(\mathbf{x}_\text{nom,k}) \tag{innovation}$$

*Innovation = actual sensor reading minus what the nominal state predicted.*

**Step 2 — Innovation covariance** $\mathbf{S}_k$ ($m\times m$, derived from Property 4):

$$\mathbf{S}_k = \mathbf{H}_k\,\mathbf{P}_k\,\mathbf{H}_k^\top + \mathbf{R} \tag{innovation cov}$$

The two terms: $\mathbf{H}_k\mathbf{P}_k\mathbf{H}_k^\top$ = state uncertainty projected to sensor space; $\mathbf{R}$ = sensor noise.

**Step 3 — Kalman Gain** $\mathbf{K}_k$ ($24\times m$, derived from Property 5):

$$\mathbf{K}_k = \mathbf{P}_k\,\mathbf{H}_k^\top\,\mathbf{S}_k^{-1} \tag{Kalman Gain}$$

**Step 4 — Error state correction:**

$$\delta\hat{\mathbf{x}}_k = \mathbf{K}_k\,\boldsymbol{\nu}_k \tag{correction vector, 24×1}$$

**Step 5 — Inject correction into nominal state:**

$$\hat{\mathbf{q}}_{k|k} = \hat{\mathbf{q}}_{k|k-1}\otimes\Delta\mathbf{q}(\delta\hat{\boldsymbol{\theta}}) \tag{multiplicative — stays on sphere}$$

$$\hat{\mathbf{v}}_{k|k} = \hat{\mathbf{v}}_{k|k-1} + \delta\hat{\mathbf{v}}\,, \quad \hat{\mathbf{p}}_{k|k} = \hat{\mathbf{p}}_{k|k-1} + \delta\hat{\mathbf{p}}\,, \quad \ldots \tag{additive}$$

→ Code: `Ekf::fuse()` in `ekf_helper.cpp`

**Step 6 — Update covariance** (Joseph form for numerical stability):

$$\mathbf{P}_{k|k} = (\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)\,\mathbf{P}_{k|k-1}\,(\mathbf{I}-\mathbf{K}_k\mathbf{H}_k)^\top + \mathbf{K}_k\,\mathbf{R}\,\mathbf{K}_k^\top \tag{Joseph form}$$

*Simple form:* $(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}$ — correct when $\mathbf{K}$ is optimal, but can lose symmetry numerically.

*Joseph form:* always symmetric and positive semi-definite, even with suboptimal $\mathbf{K}$ (e.g., when gains are zeroed for unobservable states).

→ Code: `Ekf::measurementUpdate()` in `ekf_helper.cpp`

**Note on scalar fusion in PX4:** PX4 fuses measurements one scalar at a time. For a GPS (3 measurements), it calls `measurementUpdate()` three times — once for $p_N$, once for $p_E$, once for $p_D$. Each call: $\mathbf{H}\in\mathbb{R}^{1\times 24}$, $\mathbf{K}\in\mathbb{R}^{24\times 1}$, $S = \mathbf{H}\mathbf{P}\mathbf{H}^\top + R$ is a scalar (no matrix inversion needed — just a division). This reduces complexity from $O(m^3)$ matrix inversion to $O(1)$ per measurement.

---

### 5.15 The Cost of Linearization and Why It Still Works

**What the approximation discards:**

The true covariance propagation is $\text{Cov}[\mathbf{f}(\mathbf{x})] = \int \mathbf{f}(\mathbf{x})(\mathbf{f}(\mathbf{x}))^\top p(\mathbf{x})\,d\mathbf{x}$, which has no closed form for nonlinear $\mathbf{f}$. The Taylor approximation gives $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ — which captures only the first-order behavior. The discarded second-order terms scale as $O(\|\mathbf{P}\|^2)$.

**Three failure modes:**

1. **Linearization error:** $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ underestimates the true spread — filter becomes overconfident.
2. **Inconsistency:** Reported $\mathbf{P}$ is smaller than the actual mean-squared error.
3. **Divergence:** If $\mathbf{P}$ is large (poor initial condition or large maneuver), the first-order approximation is poor and the filter can diverge.

**Why it works well for PX4 EKF2:**

- IMU rate is 200–1000 Hz → $\Delta t \leq 5$ ms → $\|\Delta\boldsymbol{\phi}\| \leq 0.05$ rad per step (mildly nonlinear)
- After initial convergence, $\mathbf{P}$ remains small → second-order terms are $O(\mathbf{P}^2) \ll O(\mathbf{P})$
- The ESKF further helps: $\delta\boldsymbol{\theta}$ is always a small perturbation around the nominal → the linearization of the error dynamics is accurate even when the nominal trajectory has large attitude angles

```
True posterior:  non-Gaussian
EKF posterior:   Gaussian(f(x̂), F·P·Fᵀ + Q)
                 = first-order Taylor approximation of true covariance
                 valid when P is small and f is mildly nonlinear
```

---

## CHAPTER 6 — EKF2 in PX4: Theory Meets Code

### 6.1 Two-Layer Architecture

PX4 EKF2 is implemented in two layers:

```
EKF2.cpp (uORB wrapper)
│
│  Subscribes: sensor_combined, vehicle_gps_position, airspeed, ...
│  Publishes:  vehicle_odometry, estimator_status, ...
│
└── EKF/ (algorithm, platform-independent)
    ├── ekf.cpp / ekf.h         ← main Ekf class
    ├── control.cpp             ← controlFusionModes() dispatcher
    ├── covariance.cpp          ← predictCovariance(), constrainStateVariances()
    ├── ekf_helper.cpp          ← measurementUpdate(), fuse()
    ├── output_predictor/       ← real-time output (bridges delayed horizon)
    └── aid_sources/            ← GPS, baro, mag, optical flow, ...
```

The `Ekf` class is a pure ESKF running on delayed IMU data. The `output_predictor` propagates the estimate forward to the current time for real-time output.

### 6.2 State Vector: Theory → Code

The ESKF maintains a nominal state and a 24×24 error-state covariance. The nominal state is defined in [`EKF/python/ekf_derivation/generated/state.h`](../../../src/modules/ekf2/EKF/python/ekf_derivation/generated/state.h):

```cpp
// Auto-generated — do NOT modify by hand
struct StateSample {
    matrix::Quaternion<float> quat_nominal{};  // attitude (4 floats, 3 DoF in error state)
    matrix::Vector3<float>    vel{};           // NED velocity (3)
    matrix::Vector3<float>    pos{};           // NED position (3)
    matrix::Vector3<float>    gyro_bias{};     // gyroscope bias (3)
    matrix::Vector3<float>    accel_bias{};    // accelerometer bias (3)
    matrix::Vector3<float>    mag_I{};         // inertial magnetic field (3)
    matrix::Vector3<float>    mag_B{};         // body magnetic field (3)
    matrix::Vector2<float>    wind_vel{};      // wind velocity NE (2)
    float                     terrain{};       // terrain height AGL (1)
};

namespace State {
    static constexpr IdxDof quat_nominal{0,  3};  // 3 DoF (δθ) in error state
    static constexpr IdxDof vel        {3,  3};
    static constexpr IdxDof pos        {6,  3};
    static constexpr IdxDof gyro_bias  {9,  3};
    static constexpr IdxDof accel_bias {12, 3};
    static constexpr IdxDof mag_I      {15, 3};
    static constexpr IdxDof mag_B      {18, 3};
    static constexpr IdxDof wind_vel   {21, 2};
    static constexpr IdxDof terrain    {23, 1};
    static constexpr uint8_t size{24};           // error-state dimension
};
```

**Why `quat_nominal` has 4 floats but 3 DoF:** The nominal quaternion lives on the unit sphere (4 components, norm = 1). The **error state** for attitude is $\delta\boldsymbol{\theta}\in\mathbb{R}^3$ — a rotation vector in the tangent space. The covariance $\mathbf{P}$ is 24×24, not 25×25.

### 6.3 Main Loop: `Ekf::update()`

Every IMU sample triggers [ekf.cpp](../../../src/modules/ekf2/EKF/ekf.cpp):

```
Ekf::update()
  │
  ├── predictState(imu_delayed)      // Step 1: propagate nominal state
  ├── predictCovariance(imu_delayed) // Step 2: propagate P via Jacobian F
  ├── controlFusionModes(imu_delayed)// Step 3: run all measurement updates
  └── output_predictor.correctOutputStates() // Step 4: bridge to real-time
```

This maps exactly to the ESKF algorithm:
- Steps 1+2 = ESKF **PREDICT**
- Step 3 = ESKF **UPDATE** (repeated for each available measurement)
- Step 4 = propagate the corrected estimate to the current time using fresh IMU data

### 6.4 PREDICT Step: `predictState()`

[ekf.cpp:231–277](../../../src/modules/ekf2/EKF/ekf.cpp) implements the nonlinear nominal state propagation from §6.3:

```cpp
void Ekf::predictState(const imuSample &imu_delayed)
{
    // 1. Correct gyro measurement for bias and Earth rotation
    const Vector3f delta_ang_bias_scaled = getGyroBias() * imu_delayed.delta_ang_dt;
    Vector3f corrected_delta_ang = imu_delayed.delta_ang - delta_ang_bias_scaled;
    corrected_delta_ang -= _R_to_earth.transpose() * _earth_rate_NED * imu_delayed.delta_ang_dt;

    // 2. Attitude update: q_nom ← q_nom ⊗ Δq(ω_corrected)
    const Quatf dq(AxisAnglef{corrected_delta_ang});
    _state.quat_nominal = (_state.quat_nominal * dq).normalized();
    _R_to_earth = Dcmf(_state.quat_nominal);

    // 3. Velocity update: v ← v + R(q) * a_corrected * Δt + g * Δt
    const Vector3f delta_vel_bias_scaled = getAccelBias() * imu_delayed.delta_vel_dt;
    const Vector3f corrected_delta_vel = imu_delayed.delta_vel - delta_vel_bias_scaled;
    const Vector3f corrected_delta_vel_ef = _R_to_earth * corrected_delta_vel;
    _state.vel += corrected_delta_vel_ef;

    // 4. Gravity, Coriolis, transport rate corrections
    const Vector3f gravity_acceleration(0.f, 0.f, CONSTANTS_ONE_G); // simplistic model
    const Vector3f coriolis_acceleration = -2.f * _earth_rate_NED.cross(vel_last);
    const Vector3f transport_rate = -_gpos.computeAngularRateNavFrame(vel_last).cross(vel_last);
    _state.vel += (gravity_acceleration + coriolis_acceleration + transport_rate) * imu_delayed.delta_vel_dt;

    // 5. Position update: p ← p + (v_old + v_new)/2 * Δt  (trapezoidal)
    _gpos += (vel_last + _state.vel) * imu_delayed.delta_vel_dt * 0.5f;
    _state.pos(2) = -_gpos.altitude();
}
```

**Mapping to theory:**
- Line with `dq`: $\mathbf{q}_\text{nom}\leftarrow\mathbf{q}_\text{nom}\otimes\Delta\mathbf{q}$ (quaternion kinematics, §6.3 Step 1)
- Lines with `corrected_delta_vel_ef`: $\mathbf{v}\leftarrow\mathbf{v}+\mathbf{R}(\mathbf{q})\,\tilde{\mathbf{a}}\,\Delta t$
- `gravity_acceleration` line: add $\mathbf{g}\Delta t$ (NED: $g$ acts in $+Z$ direction)
- Comment `// simplistic model` — a known limitation: no altitude-dependent $g$ variation

### 6.5 PREDICT Step: `predictCovariance()`

[covariance.cpp](../../../src/modules/ekf2/EKF/covariance.cpp) computes $\mathbf{P}\leftarrow\mathbf{F}\mathbf{P}\mathbf{F}^\top+\mathbf{Q}$.

Rather than forming $\mathbf{F}$ explicitly (24×24 = 576 elements), the code uses **auto-generated symbolic expressions** from `EKF/python/ekf_derivation/derivation.py`. The Python script runs SymPy to differentiate the ESKF kinematics analytically, then generates optimized C++ that directly updates each element of $\mathbf{P}$.

This is the Jacobian $\mathbf{F} = \partial\delta\mathbf{x}_{k+1}/\partial\delta\mathbf{x}_k$ evaluated at the current nominal state — the core of the ESKF predict step (§6.3 Step 2).

After propagating $\mathbf{P}$, variances are numerically clamped:

```cpp
// covariance.cpp — constrainStateVariances()
// NOTE: This limiting is a last resort and should not normally activate.
// If it does, it indicates a problem with the noise covariance values.
for (unsigned i = 0; i < State::size; i++) {
    P(i, i) = math::max(P(i, i), lower_limit);
}
```

This clamping is a stability safeguard, not part of the theoretical ESKF — it indicates numerical issues when it fires.

### 6.6 UPDATE Step: `measurementUpdate()`

[ekf_helper.cpp:1089–1138](../../../src/modules/ekf2/EKF/ekf_helper.cpp) implements the ESKF update (§6.3 Steps 3–5):

```cpp
bool Ekf::measurementUpdate(VectorState &K, const VectorState &H, const float R, const float innovation)
{
    clearInhibitedStateKalmanGains(K);  // zero K for unobservable states

    // Joseph stabilized covariance update:
    // P = (I - K*H) * P * (I - K*H)^T + K*R*K^T
    // More numerically stable than simple P = (I - K*H)*P
    //
    // Step 1: P_temp = P - K * H^T * P  (conventional update)
    VectorState PH = P * H;
    for (unsigned i = 0; i < State::size; i++)
        for (unsigned j = 0; j < State::size; j++)
            P(i, j) -= K(i) * PH(j);

    // Step 2: P = P_temp - P_temp*H^T*K^T + K*R*K^T  (stabilization)
    PH = P * H;
    for (unsigned i = 0; i < State::size; i++)
        for (unsigned j = 0; j <= i; j++) {
            P(i, j) = P(i, j) - PH(i) * K(j) + K(i) * R * K(j);
            P(j, i) = P(i, j);  // maintain symmetry
        }

    constrainStateVariances();  // numerical safeguard

    fuse(K, innovation);  // apply: δx̂ += K * innovation; inject into nominal state
    return true;
}
```

**Why Joseph form?** The simple form $\mathbf{P}=({\mathbf{I}}-\mathbf{K}\mathbf{H})\mathbf{P}$ is theoretically correct when $\mathbf{K}$ is optimal but can make $\mathbf{P}$ non-symmetric (due to floating-point) when gains are zeroed for unobservable states. The Joseph form $\mathbf{P}=(\mathbf{I}-\mathbf{K}\mathbf{H})\mathbf{P}(\mathbf{I}-\mathbf{K}\mathbf{H})^\top+\mathbf{K}\mathbf{R}\mathbf{K}^\top$ guarantees positive semi-definiteness regardless.

**Note on scalar fusion:** PX4 fuses measurements one at a time (sequential scalar fusion), not as a batch matrix update. Each call to `measurementUpdate()` handles one scalar measurement. The `H` and `K` vectors are 24-dimensional, and `R` is a scalar. This converts the matrix inversion $(\mathbf{H}\mathbf{P}\mathbf{H}^\top+\mathbf{R})^{-1}$ to a scalar division — O(n) vs O(n³).

### 6.7 State Injection: `fuse()`

After computing $\delta\hat{\mathbf{x}}=\mathbf{K}\cdot\text{innovation}$, the correction is injected back into the nominal state:

```cpp
void Ekf::fuse(const VectorState &K, float innovation)
{
    // δx̂ = K * innovation
    // Inject into nominal state: x_nom ← x_nom ⊞ δx̂

    // Attitude (multiplicative correction):
    // δq = [1, δθ/2]^T  where δθ = K[0:3] * innovation
    const Vector3f delta_angle_error = K.slice<3,1>(State::quat_nominal.idx, 0) * innovation;
    _state.quat_nominal = _state.quat_nominal * Quatf(AxisAnglef(delta_angle_error));
    _state.quat_nominal.normalize();

    // All other states (additive correction):
    _state.vel        -= K.slice<3,1>(State::vel.idx, 0) * innovation;
    _state.pos        -= K.slice<3,1>(State::pos.idx, 0) * innovation;
    _state.gyro_bias  -= K.slice<3,1>(State::gyro_bias.idx, 0) * innovation;
    // ... (accel_bias, mag_I, mag_B, wind_vel, terrain) ...
}
```

**Mapping to §6.3 Steps 4–5:** The multiplicative update $\mathbf{q}\leftarrow\mathbf{q}\otimes\delta\mathbf{q}$ for attitude and additive updates for all other states. After `fuse()`, the error state is implicitly reset to zero (next predict step starts from the updated nominal).

### 6.8 Measurement Dispatcher: `controlFusionModes()`

[control.cpp:46](../../../src/modules/ekf2/EKF/control.cpp) decides which sensors to fuse each cycle:

```cpp
void Ekf::controlFusionModes(const imuSample &imu_delayed)
{
    // Set vehicle flags (in_air, fixed_wing, at_rest, ...)
    // Monitor tilt alignment convergence

#if defined(CONFIG_EKF2_MAGNETOMETER)
    controlMagFusion();      // magnetometer heading/3D
#endif

    controlHeightFusion(imu_delayed);  // barometer / GPS / range / EV

    controlGpsFusion(imu_delayed);     // GPS position + velocity

#if defined(CONFIG_EKF2_OPTICAL_FLOW)
    controlOpticalFlowFusion(imu_delayed);
#endif

    // ... additional aid sources (airspeed, EV, drag) ...
}
```

Each `control*Fusion()` function:
1. Checks sensor availability and quality flags
2. Computes the predicted measurement $\hat{z} = h(\mathbf{x}_\text{nom})$
3. Forms the Jacobian $\mathbf{H}$ for the error state
4. Computes Kalman gain $\mathbf{K} = \mathbf{P}\mathbf{H}^\top / (\mathbf{H}\mathbf{P}\mathbf{H}^\top + R)$
5. Calls `measurementUpdate(K, H, R, innovation)` to update $\mathbf{P}$ and inject correction

For a GPS quadrotor, only GPS + baro + mag branches are active. All others are compiled out or short-circuit on flag checks.

### 6.9 Output Predictor: Bridging Delayed to Real-Time

EKF2 runs on **delayed IMU data** — there is an observation buffer (~100 ms) to time-align sensor measurements. This means the EKF estimate refers to the past.

The `output_predictor` ([output_predictor.cpp](../../../src/modules/ekf2/EKF/output_predictor/output_predictor.cpp)) propagates the corrected delayed estimate forward to the current time using the buffered IMU data that arrived after the delayed horizon:

```
t_delayed          t_now
    │                │
    ▼                ▼
[EKF estimate]──────►[output_predictor estimate]
                IMU integration (no corrections)
```

At each EKF update cycle, `correctOutputStates()` applies the error from the delayed estimate back to the output predictor, keeping the real-time output consistent with the EKF:

```
// output_predictor.cpp:281
// TODO: there is no guarantee that data is at delayed fusion horizon
```

This developer TODO acknowledges that the time alignment is approximate — a known limitation of the architecture.

### 6.10 Full Theory-to-Code Map

| Theory | Code location | What it does |
|---|---|---|
| Error state $\delta\mathbf{x}\in\mathbb{R}^{24}$ | `state.h` — `State::size = 24` | Dimension of all filter matrices |
| Nominal state $\mathbf{x}_\text{nom}$ | `StateSample` struct | The quaternion, vel, pos, biases, ... |
| ESKF PREDICT: $\mathbf{x}_\text{nom}\leftarrow\mathbf{f}(\mathbf{x}_\text{nom})$ | `ekf.cpp: predictState()` | IMU integration, nonlinear |
| ESKF PREDICT: $\mathbf{P}\leftarrow\mathbf{F}\mathbf{P}\mathbf{F}^\top+\mathbf{Q}$ | `covariance.cpp: predictCovariance()` | Jacobian F from generated code |
| Measurement dispatcher | `control.cpp: controlFusionModes()` | Enables/disables each aid source |
| ESKF UPDATE: $\mathbf{K}$, $\delta\hat{\mathbf{x}}$, $\mathbf{P}$ | `ekf_helper.cpp: measurementUpdate()` | Joseph-stabilized, scalar |
| ESKF UPDATE: $\mathbf{x}_\text{nom}\leftarrow\mathbf{x}_\text{nom}\boxplus\delta\hat{\mathbf{x}}$ | `ekf_helper.cpp: fuse()` | Multiplicative for q, additive for rest |
| Error state reset | Implicit — fuse() zeroes the correction | Next cycle starts from updated nominal |
| Real-time bridging | `output_predictor/output_predictor.cpp` | Forward-propagates past delayed horizon |

---

## CHAPTER 7 — Complete Mental Model

### 7.1 The Chain from First Principles to Embedded Code

```
Astronomers' problem (1800):
  "Best" estimate of repeated measurements → arithmetic mean
    │
    ▼
Gauss (1809): what distribution makes mean = MLE?
  ODE: p'(e)/p(e) = c·e  →  p(e) = A·exp(-e²/2σ²)
  Normalize with Poisson's trick: A = 1/√(2πσ²)
  → THE GAUSSIAN DISTRIBUTION (derived, not assumed)
    │
    ▼
Five closure properties:
  1. Linear transform → Gaussian    (gives FPFᵀ in predict)
  2. Sum of Gaussians → Gaussian    (adds Q in predict)
  3. Product of pdfs → Gaussian     (update = Bayesian fusion)
  4. Joint Gaussian under linear H  (gives S = HPHᵀ + R)
  5. Conditional Gaussian           (gives K, x̂, P update)
    │
    ▼
Kalman Filter (KF):
  EXACT Bayesian inference under Gaussian noise + linear dynamics
  BLUE estimator: minimum variance among all linear estimators
    │
    ▼ (real dynamics are nonlinear)
Extended KF (EKF):
  Re-linearize f(x) at each step via Jacobian F = ∂f/∂x|_{x̂}
  Property 1 is approximated: f(x) ~ f(x̂) + F·δx
  → First-order Gaussian approximation, no longer exact
    │
    ▼ (attitude lives on SO(3), not R^n)
Error-State EKF (ESKF):
  Split: x_true = x_nom ⊞ δx
  Nominal state: propagated by full nonlinear kinematics
  Error state: always small → linear Gaussian KF is accurate
  Quaternion correction: q ← q ⊗ δq(δθ)  (multiplicative, stays on sphere)
    │
    ▼ (embedded navigation, real sensors)
PX4 EKF2:
  state.h:        24-dim error state (3+3+3+3+3+3+3+2+1)
  ekf.cpp:        predictState() — nonlinear IMU integration
  covariance.cpp: predictCovariance() — F generated by SymPy
  ekf_helper.cpp: measurementUpdate() — Joseph-stabilized scalar fusion
                  fuse() — multiplicative q update + additive rest
  control.cpp:    controlFusionModes() — GPS/baro/mag/... dispatcher
  output_predictor: bridge delayed horizon → real-time output
```

### 7.2 Where Each Approximation Enters

| Stage | What is exact | What is approximated |
|---|---|---|
| KF | Everything (under linear + Gaussian) | — |
| EKF | Correction form | $\mathbf{F}\mathbf{P}\mathbf{F}^\top$ ≈ true propagated covariance |
| ESKF | Nominal trajectory | Error dynamics linearization (but $\delta\mathbf{x}$ small → very accurate) |
| PX4 EKF2 | ESKF equations | Scalar fusion order, simplistic gravity model, fixed process noise, output predictor time alignment |

### 7.3 What to Verify in Code When Debugging

| Symptom | Theory cause | Code location |
|---|---|---|
| Attitude drifts | Gyro bias not converging | `predictState()` — `getGyroBias()` |
| Position diverges | $\mathbf{P}$ inconsistent | `covariance.cpp: constrainStateVariances()` firing |
| Innovation too large | $\mathbf{H}$ or $\mathbf{R}$ misconfigured | `measurementUpdate()` — check $S = \mathbf{H}\mathbf{P}\mathbf{H}^\top + R$ |
| Output jitter | Time-alignment issue | `output_predictor.cpp: correctOutputStates()` |
| Heading jumps | Mag yaw fusion inconsistency | `control.cpp: controlMagFusion()` |
| Covariance clamped | Numerical instability | `covariance.cpp: "NOTE: This limiting is a last resort"` |

---

*Read next: [02d_ekf2_limitations.md](02d_ekf2_limitations.md) — where each approximation above becomes a quantified limitation with code evidence.*
