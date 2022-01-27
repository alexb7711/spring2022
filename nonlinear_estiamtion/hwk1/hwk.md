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

# Sources
[Chain Rule](https://en.wikipedia.org/wiki/Chain_rule)
[Product Rule](https://en.wikipedia.org/wiki/Product_rule)
