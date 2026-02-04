# FAPP Reproduction & Extensions

This repository contains a reproduction of the FAPP (Fast and Adaptive Perception and Planning) system from the paper published in T-RO 2024, along with several additions and fixes.

**Original Paper**: FAPP: Fast and Adaptive Perception and Planning for UAVs in Dynamic Cluttered Environments  
**Original Code**: https://github.com/arclab-hku/FAPP/

---
## The structure of the origin code
![alt text](https://github.com/Solitary2005/FAPP/blob/main/pics/pic1.png?raw=true)
## What This Repository Contains

### 1. Bug Fix: Sparse Object Scene Handling

**Problem**: The original open-source code crashes when the number of dynamic objects is small (e.g., 1-2 objects). This occurs when `cluster size = 1` but `ikd-tree size = 0`, causing an out-of-bounds access during nearest neighbor queries.

**Fix**: Added null checks before accessing nearest neighbor results:
- Skip discrimination when the ikd-tree is empty
- Directly add points when nearest neighbor search returns empty results

See the fix in the point cloud management module.

### 2. Implementation: Covariance-Adaptive Estimation

**Note**: The covariance-adaptive estimation described in the paper (Section III-B).

**Implementation**: Based on the paper's description, I implemented the process noise covariance (`Q`) update mechanism:
- Maintains a sliding window of innovation covariance
- Dynamically updates `Q` based on observation errors over time
- Adjustable window size `W` (tested with W=1, 10, 100)

**Key parameters used**:
- Initial `Q` diagonal: 0.1 for position, 0.1 for velocity
- Measurement noise `R` diagonal: 0.09 for position, 0.4 for velocity
- Window size: configurable (default 100)

### 3. Ablation Study Reproduction

Reproduced the ablation experiments from the paper using three simulated environments:

| Environment | Description |
|-------------|-------------|
| Env 1 | Constant velocity (5 m/s) along x-axis |
| Env 2 | Rapid direction change: acc=3 m/s² (0-1.0s), acc=-30 m/s² (1.0-1.2s), acc=3 m/s² (1.2-2.0s) |
| Env 3 | Sinusoidal velocity: period=1s, amplitude=6.28 m/s |

**Results**: The adaptive covariance update generally improves estimation accuracy, especially for non-constant velocity motions. However, results are sensitive to initial parameter settings and relative positioning between UAV and objects.

### 4. Planning Module Verification

Created simulation environments to qualitatively verify the planning module:

- **Environment 1**: 50m×50m field with 100 static boxes, 100 static cylinders, and 50/100/150 dynamic objects with mixed motion patterns
- **Environment 2**: 40m×3m narrow corridor with 10/30/50 dynamic objects moving in opposite directions
- **Environment 3**: Corridor blocked by 5 objects moving in convoy (0.6 m/s)

### 5. Personal Extension: State-Space Process Noise Update
![alt text](https://github.com/Solitary2005/FAPP/blob/main/pics/pic2.png?raw=true)
**Idea**: The paper updates process noise in the observation space. Since the observation matrix `C` is identity in the simulation (observation space = state space), I explored updating `Q` directly in the state space.

**Difference**:
- Paper (observation space): `Q_hat = C * gamma * C^T - A * P * A^T - R`
- Extension (state space): `Q_hat = C * gamma * C^T - A * P * A^T` (removes R term)

**Result**: In simulation (where observation and state spaces are equivalent), the state-space approach shows slightly better estimation accuracy. This highlights the sim-real gap: methods that work well in simulation may require careful parameter tuning for real-world deployment.


---

## Limitations & Notes

1. **Parameter Sensitivity**: The covariance-adaptive estimation is sensitive to initial `Q` and `R` values, as well as window size `W`. Different scenarios may require different parameter settings.

2. **Sim-Real Gap**: The state-space extension performs better in simulation but may not generalize directly to real-world scenarios where observation and state spaces differ.




---

## Acknowledgments

This reproduction was completed as a course project for "Intelligent Robots" at DUT. I would like to express my gratitude to the original authors for open-sourcing the code of FAPP.
