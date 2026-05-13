# This script runs PCA on the HUST dataset, a dataset with exisiting data on real joint angles when a human hand moves.

import numpy as np
import glob
from sklearn.decomposition import PCA

# Find all trial files
# files = glob.glob("Directory of Database") You need to install the HUST dataset and point this to the directory containing the trial files (e.g. "HUST_Dataset/Trial_*.txt")
print(f"Found {len(files)} trial files")

all_poses = []
for f in files:
    data = np.loadtxt(f)
    # Drop column 0 (time) and column 2 (T_ABD)
    # Remaining 15 columns: T_CMC, T_MCP, T_IP, I_MCP, I_PIP, I_DIP,
    #   M_MCP, M_PIP, M_DIP, R_MCP, R_PIP, R_DIP, L_MCP, L_PIP, L_DIP
    # remove T_ABD (index 1 after dropping time)
    joints = np.delete(data[:, 1:], 1, axis=1)
    all_poses.append(joints)

Q = np.vstack(all_poses)
print(f"Total frames: {Q.shape[0]}, Joints: {Q.shape[1]}")

# Reorder from HUST order (thumb first) to your URDF order (index first, thumb last)
# HUST: T_CMC, T_MCP, T_IP, I_MCP, I_PIP, I_DIP, M_MCP, M_PIP, M_DIP, R_MCP, R_PIP, R_DIP, L_MCP, L_PIP, L_DIP
# URDF: I_MCP, I_PIP, I_DIP, M_MCP, M_PIP, M_DIP, R_MCP, R_PIP, R_DIP, L_MCP, L_PIP, L_DIP, T_CMC, T_MCP, T_IP
Q = Q[:, [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 0, 1, 2]]

# Negate joints where HUST uses positive for flexion but URDF uses negative
# mcp_idx, dip_idx, mcp_mid, pip_rng, dip_rng, pip_pnk
negate_cols = [0, 2, 3, 7, 8, 10]
Q[:, negate_cols] *= -1

# mcp_thumb needs negation + offset to land in URDF range [0.79, 2.53]
Q[:, 13] = -Q[:, 13] + 1.6

# joint limits that were specified in the URDF file (same order as reordered Q)
lower = np.array([
    -1.5708, 0.0, -1.5708,      # index: mcp, pip, dip
    -1.5708, 0.0,  0.0,          # middle: mcp, pip, dip
    0.0,   -1.5708, -1.5708,    # ring: mcp, pip, dip
    0.0,   -1.5708,  0.0,       # pinky: mcp, pip, dip
    -0.3491, 0.7854,  0.0        # thumb: cmc, mcp, ip
])
upper = np.array([
    0.0,    1.5708,  0.0,       # index
    0.0,    1.5708,  1.5708,    # middle
    1.5708, 0.0,     0.0,       # ring
    1.5708, 0.0,     1.5708,    # pinky
    2.0944, 2.5307,  1.5708     # thumb
])

# Clip to arm's joint limits
Q_clipped = np.clip(Q, lower, upper)

# Check how much clipping happened per joint
joint_names = ['mcp_idx', 'pip_idx', 'dip_idx',
               'mcp_mid', 'pip_mid', 'dip_mid',
               'mcp_rng', 'pip_rng', 'dip_rng',
               'mcp_pnk', 'pip_pnk', 'dip_pnk',
               'cmc_thm', 'mcp_thm', 'ip_thm']

for i in range(15):
    below = (Q[:, i] < lower[i]).sum()
    above = (Q[:, i] > upper[i]).sum()
    total = Q.shape[0]
    print(f"{joint_names[i]:8s}  below: {below:6d} ({below/total*100:5.1f}%)  above: {above:6d} ({above/total*100:5.1f}%)  range: [{Q[:,i].min():.3f}, {Q[:,i].max():.3f}]")

# Run PCA
pca = PCA()
pca.fit(Q_clipped)

cumvar = np.cumsum(pca.explained_variance_ratio_)
for i, v in enumerate(cumvar):
    print(f"PC {i+1}: {v:.4f} ({pca.explained_variance_ratio_[i]:.4f})")

k = np.argmax(cumvar >= 0.90) + 1
print(f"\nKeeping {k} components (explains {cumvar[k-1]:.2%} variance)")

W = pca.components_[:k].T
mu = pca.mean_
np.save("pca_matrix.npy", W)
np.save("pca_mean.npy", mu)
print(f"PCA matrix shape: {W.shape}")
print(f"Saved pca_matrix.npy and pca_mean.npy")
