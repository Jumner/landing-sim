# Finding the right decay values
I define $dt[t]=dt_0e^{\tau t}$ where $dt_0$ is your rate, $\tau$ is decay rate.
If you've done something like this before please reach out to me. It seems super obvious but I've never seen it done and none of the grad students I talked to have seen it.
We can expand the sum where $T$ is the total horizon length in seconds and $N$ is horizon in steps.
$$
T=dt_0 \frac{1-e^{\tau(N-1)}}{1-e^{\tau}}
$$
I then solve this with my calculator haha.

# 2s Caught
No data

# 1s Caught
$1e^{0.28x}$

# 0.5s Caught
$0.5e^{0.38x}$
37.5s -> 19m3s424ms

# 0.25s Caught
$0.25e^{0.48x}$
38s -> 39m42s822ms (fail)
I imposed the y[t+1] >= y[t] constraint to prevent it from compromising. It tripled the runtime?????
39.25s -> 1h56m3s201ms (caught)
# 0.1s Caught
$0.1e^{0.62x}$
 ->
