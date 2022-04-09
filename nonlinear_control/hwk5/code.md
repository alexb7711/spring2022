---
geometry: margin=3cm
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
          tabsize=3,
          numbers=left
        }
---

# Code/Plots
The problems were integrated using [`solve_ivp`](https://docs.scipy.org/doc/scipy/reference/generated/scipy.integrate.solve_ivp.html) from the `scipy` library in Python.

## 12.2.2.1)
```{.python include=p12-2-2-1.py}

```
\newpage

## 12.2.2.2)
```{.python include=p12-2-2-2.py}

```
\newpage

## 14.31)
```{.python include=p14-31.py}

```
\newpage

## 14.34)
```{.python include=p14-34.py}

```
\newpage

## 14.15 a)
```{.python include=p14-15-a.py}

```
\newpage

## 14.15 b)
```{.python include=p14-15-b.py}

```
\newpage

## Plotter
```{.python include=plot.py}

```
\newpage

## Script Runner
```{.shell include=run_plots.sh}

```
\newpage

![12.2.2.1](./img/12.2.2.1.png)
![12.2.2.2](./img/12.2.2.2.png)
![14.31](./img/14.31.png)
![14.34](./img/14.34.png)
![14.15_a](./img/14.15_a.png)
![14.15_b](./img/14.15_b.png)

# A note on `14-15-a` and `14-15-b`
Comparing the two plots, it can be noted that `a` has a better state response with more control applied. `b` converges slower, but has less overshoot with and less control applied.
