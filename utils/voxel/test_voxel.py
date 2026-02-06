import numpy as np
import torch
import spconv.pytorch as spconv
from spconv.pytorch.utils import PointToVoxel
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


def load_bunny_data():

    with open('bunnyData.pts', 'r') as f:
        lines = f.readlines()

    num_points = int(lines[0].strip())
    points = []

    for i in range(1, min(num_points + 1, len(lines))):
        line = lines[i].strip()
        if line:
            coords = list(map(float, line.split()))
            if len(coords) >= 3:
                points.append(coords[:3])

    points = np.array(points, dtype=np.float32)
    print(f"Loaded {len(points)} points from bunnyData.pts")
    print(f"Range: X[{points[:, 0].min():.3f}, {points[:, 0].max():.3f}] "
          f"Y[{points[:, 1].min():.3f}, {points[:, 1].max():.3f}] "
          f"Z[{points[:, 2].min():.3f}, {points[:, 2].max():.3f}]")

    return points


def create_voxel_grid(points, voxel_size=2.0):
    """Convert point cloud to voxel grid"""
    # Scale to mm for better voxel resolution
    points_scaled = points * 1000

    min_coords = points_scaled.min(axis=0)
    max_coords = points_scaled.max(axis=0)
    padding = 5.0

    coors_range = [
        min_coords[0] - padding,
        min_coords[1] - padding,
        min_coords[2] - padding,
        max_coords[0] + padding,
        max_coords[1] + padding,
        max_coords[2] + padding]

    # Create voxel generator
    voxel_gen = PointToVoxel(
        vsize_xyz=[voxel_size, voxel_size, voxel_size],
        coors_range_xyz=coors_range,
        num_point_features=3,
        max_num_voxels=20000,
        max_num_points_per_voxel=50
    )

    # Generate voxels
    pc_tensor = torch.from_numpy(points_scaled)
    voxels, indices, num_points_per_voxel = voxel_gen(pc_tensor)

    # Convert voxel indices to world coordinates
    voxel_centers = indices.numpy() * voxel_size + \
        np.array(coors_range[:3]) + voxel_size / 2

    print(f"Created {len(voxels)} voxels from {len(points)} points")
    print(f"Voxel size: {voxel_size}mm")

    return voxel_centers, voxels, indices


def visualize_voxels(voxel_centers):
    """Create 3D visualization of voxel grid"""
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')

    # Color by Z-coordinate
    colors = voxel_centers[:, 2]

    scatter = ax.scatter(voxel_centers[:,
                                       0],
                         voxel_centers[:,
                                       1],
                         voxel_centers[:,
                                       2],
                         c=colors,
                         cmap='viridis',
                         s=30,
                         alpha=0.8)

    # Add colorbar
    cbar = plt.colorbar(scatter, ax=ax, shrink=0.5)
    cbar.set_label('Z Coordinate (mm)')

    ax.set_xlabel('X (mm)')
    ax.set_ylabel('Y (mm)')
    ax.set_zlabel('Z (mm)')
    ax.set_title('Voxel Grid Visualization')
    ax.grid(True, alpha=0.3)
    ax.view_init(elev=20, azim=45)

    plt.savefig('voxel_visualization.png', dpi=150, bbox_inches='tight')
    print("Saved visualization as 'voxel_visualization.png'")
    plt.show()


def test_basic_functionality():
    """Test basic voxel generation with synthetic data"""
    print("Testing basic voxel functionality...")

    # Generate random point cloud
    np.random.seed(42)
    points = np.random.uniform(-2, 2, size=[1000, 3]).astype(np.float32)

    voxel_gen = PointToVoxel(
        vsize_xyz=[0.1, 0.1, 0.1],
        coors_range_xyz=[-3, -3, -3, 3, 3, 3],
        num_point_features=3,
        max_num_voxels=5000,
        max_num_points_per_voxel=10
    )

    pc_tensor = torch.from_numpy(points)
    voxels, indices, num_points = voxel_gen(pc_tensor)

    print(f"Input: {len(points)} points")
    print(f"Output: {len(voxels)} voxels")
    print(f"Voxel shape: {voxels.shape}")
    print(f"Average points per voxel: {num_points.float().mean():.2f}")

    return True


def main():
    """Main execution flow"""
    print("Voxel Grid Processing")
    print("-" * 40)

    # Test basic functionality first
    test_basic_functionality()
    print()

    # Process bunny data if available
    try:
        points = load_bunny_data()
        voxel_centers, voxels, indices = create_voxel_grid(points)
        visualize_voxels(voxel_centers)
    except FileNotFoundError:
        print("bunnyData.pts not found - skipping bunny visualization")
    except Exception as e:
        print(f"Error processing bunny data: {e}")


if __name__ == "__main__":
    main()
