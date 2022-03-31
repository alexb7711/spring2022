# 4.54.2)
## Given:
$$
\dot{x} = -(1+u)x^3 - x^5
$$

## Find:
Investigate Input-to-State stability

## Solution

Suppose $V(x) = \frac{x^2}{2} = \alpha_1(x) = \alpha_2(x)$

$$
\begin{array}{r}
	\dot{V}(x) = x\dot{x} = x(-(1+u)x^3 - x^5) \\
	= -x^4 - ux^4 - x^6
\end{array}
$$

If $u > -\frac{x^6}{x^4} = -x^2$

$$
\sqrt{u} < |x|
$$

Then $\dot{V}(x)$ will be negative definite and if $-W_3(x) = -x^4$

$$
\dot{V}(x) < -W_3(x)
$$

Therefore the system is input-to-stable by (1)
