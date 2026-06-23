# Phase 6 benchmark: stub vs DWA vs TEB

Lower jerk + fewer intimate/personal intrusions + larger min_dist is better; succ=1 means the goal was reached.

| scenario | controller | runs | succ | t_goal[s] | path[m] | jerk_mean | jerk_rms | min_dist[m] | intim% | pers% | soc% | coll |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| crossing | stub | 2 | 1 | 2.8 | 1.67 | 0.65 | 0.89 | 0.83 | 0.0 | 75.7 | 24.3 | 0 |
| crossing | dwa | 1 | 1 | 23.0 | 3.78 | 8.67 | 30.31 | 0.67 | 0.0 | 57.9 | 42.1 | 0 |
| crossing | teb | 1 | 1 | 17.5 | 9.52 | 7.71 | 25.44 | 0.80 | 0.0 | 24.4 | 58.1 | 0 |
| group | stub | 1 | 1 | 13.8 | 4.57 | 0.66 | 0.87 | 1.49 | 0.0 | 0.0 | 100.0 | 0 |
| group | dwa | 1 | 1 | 20.3 | 3.80 | 11.52 | 28.70 | 0.74 | 0.0 | 55.3 | 44.7 | 0 |
| group | teb | 1 | 1 | 36.2 | 9.49 | 52.47 | 87.60 | 0.79 | 0.0 | 22.6 | 77.4 | 0 |
| head_on | stub | 3 | 1 | 3.2 | 1.86 | 1.36 | 1.79 | 0.88 | 0.0 | 63.8 | 36.2 | 0 |
| head_on | dwa | 1 | 1 | 20.9 | 3.70 | 7.22 | 26.51 | 0.66 | 0.0 | 28.3 | 71.7 | 0 |
| head_on | teb | 1 | 1 | 15.4 | 5.74 | 14.76 | 35.80 | 0.66 | 0.0 | 41.9 | 58.1 | 0 |
