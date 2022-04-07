---
title: Homework 1
author: Alexander Brown
date: \today
---

# 1.1
## Given
A mathematical model that describes a wide variety of physical nonlinear systems is the nth-order differential equation

$$
y^{n} = g(t, y, \dot{y},...,y^{n-1}, u)
$$

where $u$ and $y$ are scalar variables. With $u$ as input and $y$ as output, find a sate model.

## Solution
Suppose that $y = x_1$, $\dot{y} = x_2$, etc. Continuing to differentiate gives:

$$
\begin{array}{l}
	\dot{x_1} = x_2 = \dot{y}                       \\
	\dot{x_2} = x_3                                 \\
	...                                             \\
	\dot{x}_{n-1} = x_n                             \\
	\dot{x}_{n}   = g(t, y, \dot{y},...,y^{n-1}, u) \\
\end{array}
$$

# 1.2
## Given
Consider a single-input-single-output system described by the $n$th-order differential equation

$$
y^{n} = g_1(t, y, \dot{y},...,y^{n-1}, u) + g_2(t, y, \dot{y},...,y^{n-1})\dot{u}
$$

where $g_2$ is a differentiable function of its arguments. With $u$ as input and $y$ as output, find a state model.

*Hint*: Take $x_n = y^{n-1} - g_2(t,y,\dot{y},...,\dot{y}^{n-1})u$

## Solution
Lets begin by defining $x_1 = y$, $x_2 = y^{(1)}$, ..., $x_{n-1} = y^{(n-2)}$, $x_n = y^{(n-1)} - g_2(t,y,\dot{y},...,y^{(n-2)})u$. Therefore

$$
\begin{array}{l}
	\dot{x}_1 = y^{(1)} = x_2                                                                       \\
	\dot{x}_2 = x_3                                                                                 \\
	...                                                                                             \\
	\dot{x}_{n-1} = y^{(n-1)} = x_{n} + g_2(t,x_1,x_2,...,x_{n-1})u                                 \\
	\dot{x}_{n}   = y^{(n)} - \dot{g}_2(t,x_1,x_2,...,x_{n-q})u - g_2(t,x_1,x_2,...,x_{n-q})\dot{u} \\
\end{array}
$$

Where $\dot{x}_{n-1}$ is found by solving $x_n = y^{n-1} - g_2(t,y,\dot{y},...,\dot{y}^{n-1})u$ for $y^{(n-1)}$ and $\dot{x}_{n}   = y^{(n)} - \dot{g}_2(t,x_1,x_2,...,x_{n-q})u - g_2(t,x_1,x_2,...,x_{n-q})\dot{u}$ is found by using the chain rule and product rule.

> Chain Rule: $h'(x) = f'(g(x))g'(x)$

> Product Rule: $(u\cdot v)' = u' \cdot v + u' \cdot v$

$$
\begin{array}{l}
	\dot{x}_{n}   = y^{(n)} - \dot{g}_2(t,x_1,x_2,...,x_{n-q})u - g_2(t,x_1,x_2,...,x_{n-q})\dot{u}  = \\
	g_1(t,x_1,x_2,...,x_n + g_2(t,x_2,x_3,...,x_{n-2})u,u) - \\
	(\frac{\partial g_2}{\partial t} + \frac{\partial g_2}{\partial x_1}\dot{x}_1 + \frac{\partial g_2}{\partial x_2}\dot{x}_2 + ... + \frac{\partial g_2}{\partial x_{n-1}}\dot{x}_{n-1})u -
	g_2(t,x_1,x_2,...,x_{n-q})\dot{u} \\
\end{array}
$$

# 1.5
## Given
The nonlinear dynamic equations for a single-link manipulator with flexible joints, damping ignored, is given by

$$
\begin{array}{l}
I\ddot{q}_1 + MgLsin(q_1) + k(q_1 - q_2) = 0 \\
J\ddot{q}_2 - k(q_1 - q_2) = u               \\
\end{array}
$$

## Solution
Begin by defining $x_1 = q_1$, $x_2 = \dot{q}_1$, $x_3 = q_2$, $x_4 = \dot{q}_2$:

$$
\begin{array}{l}
	\dot{x}_1 = \dot{q}_1 \\
	\dot{x}_2 = \ddot{q}_1 =  -\frac{MgL}{I} sin x_1 - \frac{k(x_1 - x_3)}{I}\\
	\dot{x}_3 = \dot{q}_2 \\
	\dot{x}_4 = \ddot{q}_2 = \frac{k(x_1 - x_3)}{J} + \frac{u}{J}\\
\end{array}
$$


# 3.
## Given
For each of the functions find whether

* Continuously differentiable
* Locally Lipschitz
* Continuous
* Globally Lipschitz

1. $f(x) = x^2 + \left| x \right|$
2. $f(x) = x + sgn(x)$
3. $f(x) =sin(x)sgn(x)$
4. $f(x) = -x + asin(x)$
5. $f(x) = -x + 2 \left|x\right|$
6. $f(x) = tan(x)$
7. $f(x) = \begin{bmatrix} ax_1 + tanh(bx_1) - tanh(bx_2) \\ ax_2 + tanh(bx_1) + tanh(bx_2) \end{bmatrix}$
8. $f(x) = \begin{bmatrix} -x_1 + a|x_2| \\ -(a+b)x_1 + bx_1^2 - x_1 x_2 \end{bmatrix}$

## Solution
## 1.
As referenced in the [class notes](https://profmattharris.files.wordpress.com/2022/01/nac_v01.pdf), $x^2$ is locally, but not globally, Lipschitz. $|x|$ on the other hand, it not continuous at 0. Furthermore:

$$
\forall x,y \in \mathbb{R} \; \left| |x| - |y|  \right| \leq \left| x-y \right| \\
$$

By choosing $\delta = \frac{\epsilon}{3}$:

$$
\begin{array}{l}
	\left| x^2 + |x| - y^2 - |y| \right| \leq L \left|x-y\right| \\
	\left|x^2 - y^2 \right| + \left| |x| - |y| \right|           \\
	2\left|x - y \right| + \left| x - y \right|                  \\
	3\left| x - y \right|                                        \\
	\text{Note that } \left| x - y \right| = \delta              \\
	3\delta = \frac{3\epsilon}{3} = \epsilon                     \\
\end{array}
$$

This shows that there $f(x)$ is locally Lipschitz. Therefore, the following can be said about the function

* Not continuously differentiable
* Locally Lipschitz
* Not continuous
* Not globally Lipschitz

## 2.
Consider Figure \ref{fg:sgn}:

![Plot of $sgn(x)$\label{fg:sgn}](https://staff.itee.uq.edu.au/janetw/cmc/chapters/Hopfield/sgn.gif){width=50%}

$sgn(x)$ is not continuous or continuously differentiable. However it is bounded at $\pm 1$ therefore it is globally Lipschitz (implying local as well). $x$ is continuous and continuously differentiable as well as globally Lipschitz. Therefore:

* Continuously differentiable
* Locally Lipschitz
* Continuous
* Globally Lipschitz

## 3.
Consider the following

$$
\frac{\partial f}{\partial x} = cos(x)sgn(x)
$$

Furthermore, consider Figures \ref{fg:sgnsin} and \ref{fg:sgncos}. It can be seen that for $f(x)$, it is piecewise continuous, but not continuously differentiable.

![Plot of $sin(x)sgn(x)$\label{fg:sgnsin}](./3.1.3b.pdf){width=70%}

![Plot of $cos(x)sgn(x)$\label{fg:sgncos}](./3.1.3a.pdf){width=70%}

Given the function and its derivative we can conclude:

* Not continuously differentiable
* Locally Lipschitz
* Piecewise continuous
* Globally Lipschitz

## 4.
$sin(x)$ and $x$ continuous, continuously differentiable, and globally Lipschitz therefore

* Continuously differentiable
* Locally Lipschitz
* Continuous
* Globally Lipschitz

## 5.
$-x$ is continuous, continuously differentiable, and globally Lipschitz. However, we have seen the $|x|$ is not continuous or continuously differentiable, but it is locally and globally Lipschitz. Therefore:

$$
\begin{array}{l}
	\left| -x + 2|x| + y - 2|y| \right|       \\
	\left|x-y\right| + \left| 2x - 2y \right| \\
	3\left|x-y\right|
\end{array}
$$

Implying L = 3. Note that this holds for $\mathbb{R}$.

* Not continuously differentiable
* Locally Lipschitz
* Not continuous
* Globally Lipschitz

## 6.
From Figure \ref{fg:tan}, we can see that the plot is continuous over $(-\pi/2,\pi/2)$, similarly for Figure \ref{fg:sec2}. However, there is a clear asymptote at $\pm \pi/2$. From this we can say

* Continuously differentiable over $(-\pi/2,\pi/2)$
* Locally Lipschitz
* Continuous over $(-\pi/2,\pi/2)$
* Not globally Lipschitz

![Plot of $tan(x)$\label{fg:tan}](https://www.efunda.com/math/trig_functions/images/tan_plot.gif){width=50%}

![Plot of $sec^2(x)$\label{fg:sec2}](http://intmstat.com/trigonometric-graphs/sec.gif){width=50%}

## 7.
From Figure \ref{fg:tanh}, it is easily noticed that the function is continuous, continuously differentiable, and globally Lipschitz. $ax_i$ hold similar properties. Therefore we can say the following

* Continuously differentiable
* Locally Lipschitz
* Continuous
* Globally Lipschitz

![Plot of $tanh(x)$\label{fg:tanh}](https://mathworld.wolfram.com/images/interactive/TanhReal.gif){width=50%}

## 8.
$x_i$ is continuous, continuously differentiable and globally Lipschitz. However, $|x|$ is not continuous or continuously differentiable, and $x^2$ is not globally Lipschitz (as shown in the [class notes](https://profmattharris.files.wordpress.com/2022/01/nac_v01.pdf)). Therefore

* Not continuously differentiable
* Locally Lipschitz
* Not continuous
* Not globally Lipschitz

# 3.3
## Given
Show that if $f_1: R \rightarrow R$ and $f_2: R \rightarrow R$ are locally Lipschitz, then $f_1 + f_2$, $f_1 f_2$, and $f_1 \circ f_2$ are locally Lipschitz

## Solution
Refer to Theorems Lemmas and Definitions for the Definition of locally Lipschitz.

### a) $f_1 + f_2$
Suppose $f_3 := f_1 + f_2$. Using the definition of locally Lipschitz

$$
\begin{array}{l}
	\left| f_1(x) + f_2(x) - f_1(y) - f_2(y) \right| = \\
	\left| f_1(x) - f_1(y) \right| + \left| f_2(x) - f_2(y) \right| = \\
	L_1 \left| x-y \right| + L_2 \left| x-y \right|
\end{array}
$$

### b) $f_1 f_2$
Suppose $f_3 := f_1 f_2$. Using the definition of locally Lipschitz

$$
\begin{array}{l}
	\left| f_1(x) f_2(x) - f_1(y) f_2(y) \right| = \\
	\left| f_1(x) f_2(y) + f_1(x)f_2(y) - f_1(x)f_2(y) - f_1(x) f_2(y) \right| = \\
	\left| f_1 (f_2(x) - f_2(y)) + f_2(y)(f_1(x) - f_1(y)) \right| = \\
	C_2 L_1 \left| x-y \right| + C_1 L_2\left| x-y \right|
\end{array}
$$

### c) $f_1 \circ f_2$
Suppose $f_3 := f_1 \circ f_2 = f_2(f_1(x))$. Using the definition of locally Lipschitz

$$
\begin{array}{l}
	\left| f_2(f_1(x)) - f_2(f_2(y)) \right| = \\
	L_2 \left| f_1(x) - f_1(y) \right| = \\
	L_1 L_2 \left| x - y \right|
\end{array}
$$

# 3.7
## Given
Let $g: R^n \rightarrow R^n$ be continuously differentiable for all $x \in R^n$ and define $f(x)$ by

$$
f(x) = \frac{g(x)}{1+g^T(x)g(x)}
$$

Show that $\dot{x} = f(x)$, with $x(0) = x_0$, has a unique solution defined for all $t \geq 0$.

## Solution
Uniqueness can be shown by showing $f(x)$ is globally Lipschitz (Theorems Lemmas and Definitions). Lets begin by substituting $g(x) := y$.

$$
\begin{array}{l}
	f(x) = \frac{y}{1+y^2} \\
	\frac{\partial f}{\partial y} = -\frac{y^2 - 1}{y^4 + 2y^2 + 1}
\end{array}
$$

which shows that $f(x)$ is continuous and continuously differentiable. Therefore, [by definition, section][Locally Lipschitz], the function can be identified as locally Lipschitz. Furthermore taking the limit to $\infty$, $\frac{\partial f}{\partial x} \rightarrow 0$ showing that $\frac{\partial f(x)}{\partial x}$ uniformly bounded. Therefore, [by definition, section][Globally Lipschitz], $f(x)$ can be said to be globally Lipschitz.

# 3.20
## Given
Show that if $f: R^n \rightarrow R^n$ is Lipschitz on $W \subset R^n$, then $f(x)$ is uniformly continuous on $W$.

## Solution
Lets begin with the definition:

$$
\forall x,y \in \mathbb{R}^n\; ||f(t,x) - f(t,y)|| \leq L ||x-y||
$$

Therefore, if $W \subset \mathbb{R}^n$ (assuming the set is compact)

$$
\forall x,y \in W\; ||f(t,x) - f(t,y)|| \leq \forall x,y \in \mathbb{R}^n\; ||f(t,x) - f(t,y)|| \leq L ||x-y||
$$

must be true because every subset of $\mathbb{R}^n$ must also satisfy the Lipschitz conditions.

# Canvas Problem
## Given
Give the definitions and examples of open sets, closed sets, and compact sets in 2D.

## Solution
### Open Sets
> *Definition*: An open set in a metric space $(X,d)$ is a subset $U$ of $X$ with the following property: for any $x\in U$ there is a real number $\epsilon$ such that any point $X$ that is a distance less than $\epsilon$ from $x$ is also contained in $U$. \newline
For any point $x \in X$, define $B(x,\epsilon)$ to be the open ball of radius $\epsilon$, which is the set of all points in $X$ that are within distance $\epsilon$ from $x$. Then a set $U$ is open iff for each point $x\in U$, there is an $\epsilon > 0$ such that $B(x,\epsilon)$ is completely contained in $U$

![Example of an open set](https://ds055uzetaobb.cloudfront.net/brioche/uploads/yE4nedzIk9-openset.png){width=50%}

The open interval $(2,5)$ is an open set.

### Closed Sets
> *Definition*: A closed set in a metric space $(X,d)$ is a subset $Z$ of $X$ with the following property: for any point $x \not{\epsilon} Z$, there is a ball $B(x,\epsilon)$ around $x$, for some $\epsilon > 0$, which is joint from $Z$.

The closed set $[2,5]$ is a closed set.

### Compact Sets
> *Definition*: A cover of a set $X$ is a collection of sets whose union includes $X$ as a subset. Formally, if $C = \{U_{\alpha}:\alpha \in A\}$ is an indexed family of sets $U_{\alpha}$, then $C$ is a cover of $X$ if

$$
X \subseteq \bigcup_{\alpha \in A} U_{\alpha}
$$

> *Definition*: $Z$ is compact if every open cover has a finite subcover.

The closed unit interval $[0,1]$ is compact.

# Theorems Lemmas and Definitions
## Locally Lipschitz
> *Theorems*: If $f:I\times D \rightarrow \mathbb{R}^n$ is locally Lipschitz at $x_0 \in \mathbb{R}^n$, then $\exists t' \in (t_o, t_1]$ such that the solution of $\dot{x} = f(t,x), x(t_0) = x_0$ exists and is unique on $[t_0, t']$.

> *Lemma*: If $f$ and $\frac{\partial f_i}{\partial x_j}$ are continuous (i.e., $f$ is continuously differentiable) on $I\times x_0$, then $f$ is locally Lipschitz at $x_0$

## Globally Lipschitz
> *Definition*: A function $f: I \times \mathbb{R}^n \rightarrow \mathbb{R}^n$ is globally Lipschitz if:

> 1. $\forall x \in D, f(\cdot, x)$ is piecewise continuous on $I$
> 2. $\exists L \in \mathbb{R}^+$ such that
>	  * $\forall t\in I, \forall x, y \in \mathbb{R}^n$
>	  * $||f(t,x) - f(t,y)|| \leq L ||x-y||$

> *Theorem*: If $f:I\times D \rightarrow \mathbb{R}^n$ is globally Lipschitz then the solution of $\dot{x} = f(t,x), x(t_0) = x_0$ exists and is unique on $I$

> *Lemma*: Suppose $f$ and $\frac{\partial f}{\partial x}$ are continuous on $I \times \mathbb{R}^n$. $f$ is globally Lipschitz iff $\frac{\partial f}{\partial x}$ is uniformly bound on $I \times \mathbb{R}^n$, i.e.
>
> $\exists L\in \mathbb{R}^+$ s.t. $\forall t \in I, \forall x \in \mathbb{R}^n$
$$
|| \frac{\partial f(t,x)}{\partial x}|| \leq L
$$

> *Theorem*: If $I\times R^n \rightarrow R^n$ is globally Lipschitz then the solution of $\dot{x} = f(t,x)$, $x(t_0) = x_0$ exists and is unique on $I$.

# Sources
* [Class notes](https://profmattharris.files.wordpress.com/2022/01/nac_v01.pdf)
* [Chain Rule](https://en.wikipedia.org/wiki/Chain_rule)
* [Product Rule](https://en.wikipedia.org/wiki/Product_rule)
* [Open Sets](https://brilliant.org/wiki/open-sets/)
* [Closed Sets](https://brilliant.org/wiki/closed-sets/)
* Compact Sets
	* [Brilliant](https://brilliant.org/wiki/compact-space/)
	* [Wikipedia](https://en.wikipedia.org/wiki/Compact_space)
