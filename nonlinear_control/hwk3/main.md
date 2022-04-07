---
header-includes: |
        \usepackage{listings}
        \usepackage{color}

        \definecolor{dkgreen}{rgb}{0,0.6,0}
        \definecolor{gray}{rgb}{0.5,0.5,0.5}
        \definecolor{mauve}{rgb}{0.58,0,0.82}

        \lstset{frame=tb,
          language=Java,
          aboveskip=3mm,
          belowskip=3mm,
          showstringspaces=false,
          columns=flexible,
          basicstyle={\small\ttfamily},
          numbers=none,
          numberstyle=\tiny\color{gray},
          keywordstyle=\color{blue},
          commentstyle=\color{dkgreen},
          stringstyle=\color{mauve},
          breaklines=true,
          breakatwhitespace=true,
          tabsize=3
        }
---

# 4.32.4)

## Given
$$
\begin{array}{l}
\dot{x}_1 = -x_1 \\
\dot{x}_2 = -x_1 - x_2 - x_3 - x_1 x_3
\dot{x}_3 = (x_1 + 1)x_2
\end{array}
$$

## Find
Investigate whether the origin is stable, asympotically stable, or unstable.

\hrule

## Solution
$$
J = \frac{\partial f}{\partial x} =
\left. \begin{bmatrix}
        -1     & 0     & 0      \\
        -x_3-1 & -1    & -x_1-1 \\
        x_2    & x_1+1 & 0      \\
\end{bmatrix}\right|_{x=[0]} =  \\
\begin{bmatrix}
        -1     & 0     & 0      \\
        -1     & -1    & 1      \\
         0     & 1     & 0      \\
\end{bmatrix}
$$

Finding the eigenvalues by using `eivals(*)` in Maxima

```lisp
/* Define equation */
xd1: -x1;
xd2: -x1 - x2 - x3 - x1*x3;
xd3: (x1 + 1)*x2;

/* Solve for roots */
solve([xd1=0, xd2=0, xd3=0])

/* Calculate jacobian */
J: jacobian([xd1, xd1, xd3], [x1, x2, x3]);

/* Evaluate jacobian at roots */
float(eivals(psubst([x1=0, x2=0, x3=0], J)))

```

> *Theorem*: Let $x=0$ be an equilibrium point for
$$
\dot{x} = f(x)
$$
	where $f:D \rightarrow \mathbb{R}^n$ is continuously differentiable and $D$ is in a neighborhood of the origin. Let
$$
A = \frac{\partial f}{\partial x}(0)
$$
    Let $\lambda_i$ denote an eigenvalue of $A$

1. If $\forall \lambda_i Re(\lambda_i) < 0$, then the origin is asympotically stable
2. If $\exists \lambda_i$ such that $Re(\lambda_i) > 0$ then the origin is not stable

Therefore, by the previous stated theorem, the system is asympotically stable.
