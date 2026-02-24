\# YOLO A/B Comparison: v5n vs v5s (CPU, estimate-mode follow)



\## Context

Both runs use the same setup and differ only by YOLO weights:

\- Device: `cpu` (AMD GPU machine, no CUDA)

\- Image topic: `/dji0/camera0/image\_raw`

\- Mode: `LEADER\_MODE=estimate`

\- Estimator outputs:

&nbsp; - `/coord/leader\_estimate` (`PoseStamped`)

&nbsp; - `/coord/leader\_estimate\_status` (`String`)



\## Compared Runs

\- `runs/estimator\_pose/run\_yolo\_v5n\_0001\_cpu/bag`

\- `runs/estimator\_pose/run\_yolo\_v5s\_0001\_cpu/bag`



\## Bag-Based Comparison



| Metric | v5n (CPU) | v5s (CPU) | Notes |

|---|---:|---:|---|

| Duration (s) | 205.06 | 211.79 | Similar run lengths |

| `/coord/leader\_estimate` count | 371 | 149 | v5n produced more estimates |

| `/coord/leader\_estimate` avg rate | ~1.81 Hz | ~0.70 Hz | `count / duration` |

| `/coord/leader\_estimate\_status` count | 449 | 180 | v5n emitted more status updates |

| `/coord/leader\_estimate\_status` avg rate | ~2.19 Hz | ~0.85 Hz | Status publishes on timer + estimator ticks |

| `/dji0/pose\_cmd` count | 371 | 150 | v5n drove follow updates more often |

| `/coord/follow\_dist\_cmd` count | 371 | 150 | Mirrors follow activity |

| `/coord/follow\_tracking\_error\_cmd` count | 371 | 150 | Mirrors follow activity |

| Live status sample | `state=OK`, `yolo=enabled`, `device=cpu`, `conf≈0.704` | `state=OK`, `yolo=enabled`, `device=cpu`, `conf≈0.664` | Single samples only; not statistical |

| Visual behavior | Following worked | Following worked | v5s appeared more intermittent |



\## Quick Interpretation

\- `v5n` performed better in this setup (CPU + estimate-mode following):

&nbsp; - higher estimate publish count/rate

&nbsp; - more follow command updates

&nbsp; - better continuity for online following

\- `v5s` also works end-to-end, but appears slower/more intermittent on CPU

&nbsp; - likely due heavier inference load

&nbsp; - lower estimate update rate reduces follow update opportunities



\## Practical Conclusion

\- On this AMD/CPU setup, `v5n` is the better default for online following.

\- `v5s` remains a valid option if detection robustness is preferred over update rate (or on faster hardware).



\## Notes

\- `YOLO\_DEVICE=cuda` is not usable on this AMD machine; CPU fallback was used for a fair A/B comparison.

\- These A/B runs used constant-range estimation (no validated depth topic configured), which keeps the comparison controlled.



