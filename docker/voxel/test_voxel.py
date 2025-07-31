import numpy as np
import torch
import spconv.pytorch as spconv
from spconv.pytorch.utils import PointToVoxel, gather_features_by_pc_voxel_id

def test_basic_voxel_generation():
    """Test basic voxel generation functionality"""
    print("=" * 60)
    print("Testing Basic Voxel Generation")
    print("=" * 60)
    
    np.random.seed(50051)
    
    pc = np.random.uniform(-4, 4, size=[1000, 3]).astype(np.float32)
    print(f"Generated point cloud with {pc.shape[0]} points")
    print(f"Point cloud range: {pc.min():.2f} to {pc.max():.2f}")
    
    gen = PointToVoxel(
        vsize_xyz=[0.1, 0.1, 0.1],  # voxel size
        coors_range_xyz=[-80, -80, -6, 80, 80, 6],  # coordinate range
        num_point_features=3,  # XYZ features
        max_num_voxels=5000,
        max_num_points_per_voxel=5
    )
    
    pc_th = torch.from_numpy(pc)
    
    voxels_th, indices_th, num_p_in_vx_th = gen(pc_th)
    
    print(f"Generated {voxels_th.shape[0]} voxels")
    print(f"Voxel shape: {voxels_th.shape}")
    print(f"Indices shape: {indices_th.shape}")
    print(f"First voxel content:\n{voxels_th[0]}")
    print(f"First voxel indices: {indices_th[0]}")
    print(f"Points in first voxel: {num_p_in_vx_th[0]}")
    
    return True

def test_voxel_with_mean_fill():
    """Test voxel generation with mean filling for empty slots"""
    print("\n" + "=" * 60)
    print("Testing Voxel Generation with Mean Fill")
    print("=" * 60)
    
    np.random.seed(50051)
    
    pc = np.random.uniform(-4, 4, size=[1000, 3]).astype(np.float32)
    
    gen = PointToVoxel(
        vsize_xyz=[0.1, 0.1, 0.1],
        coors_range_xyz=[-80, -80, -6, 80, 80, 6],
        num_point_features=3,
        max_num_voxels=5000,
        max_num_points_per_voxel=5
    )
    
    pc_th = torch.from_numpy(pc)
    
    voxels_th, indices_th, num_p_in_vx_th = gen(pc_th, empty_mean=True)
    
    print(f"Generated {voxels_th.shape[0]} voxels with mean fill")
    print(f"First voxel with mean fill:\n{voxels_th[0]}")
    
    return True

def test_voxel_with_id_generation():
    """Test voxel generation with point-to-voxel ID mapping"""
    print("\n" + "=" * 60)
    print("Testing Voxel Generation with ID Mapping")
    print("=" * 60)
    
    np.random.seed(50051)
    
    pc = np.random.uniform(-4, 4, size=[1000, 3]).astype(np.float32)
    
    gen = PointToVoxel(
        vsize_xyz=[0.1, 0.1, 0.1],
        coors_range_xyz=[-80, -80, -6, 80, 80, 6],
        num_point_features=3,
        max_num_voxels=5000,
        max_num_points_per_voxel=5
    )
    
    pc_th = torch.from_numpy(pc)
    
    voxels_th, indices_th, num_p_in_vx_th, pc_voxel_id = gen.generate_voxel_with_id(pc_th, empty_mean=True)
    
    print(f"Generated {voxels_th.shape[0]} voxels with ID mapping")
    print(f"Point-to-voxel ID mapping shape: {pc_voxel_id.shape}")
    print(f"First 10 point voxel IDs: {pc_voxel_id[:10]}")
    
    return True

def test_cuda_voxel_generation():
    """Test voxel generation on CUDA if available"""
    print("\n" + "=" * 60)
    print("Testing CUDA Voxel Generation")
    print("=" * 60)
    
    if not torch.cuda.is_available():
        print("CUDA not available, skipping CUDA tests")
        return True
    
    np.random.seed(50051)
    
    pc = np.random.uniform(-2, 8, size=[10000, 3]).astype(np.float32)
    
    for device in [torch.device("cuda:0"), torch.device("cpu:0")]:
        print(f"\nTesting on device: {device}")
        
        gen = PointToVoxel(
            vsize_xyz=[0.25, 0.25, 0.25],
            coors_range_xyz=[0, 0, 0, 10, 10, 10],
            num_point_features=3,
            max_num_voxels=5000,
            max_num_points_per_voxel=5,
            device=device
        )
        
        pc_th = torch.from_numpy(pc).to(device)
        
        voxels_th, indices_th, num_p_in_vx_th = gen(pc_th)
        print(f"  Generated {voxels_th.shape[0]} voxels on {device}")
        
        voxels_th, indices_th, num_p_in_vx_th = gen(pc_th, empty_mean=True)
        print(f"  Generated {voxels_th.shape[0]} voxels with mean fill on {device}")
        
        voxels_th, indices_th, num_p_in_vx_th, pc_voxel_id = gen.generate_voxel_with_id(pc_th, empty_mean=True)
        print(f"  Generated {voxels_th.shape[0]} voxels with ID mapping on {device}")
    
    return True

def test_sparse_convolution():
    """Test basic sparse convolution using generated voxels"""
    print("\n" + "=" * 60)
    print("Testing Sparse Convolution with Voxels")
    print("=" * 60)
    
    np.random.seed(50051)
    
    pc = np.random.uniform(-2, 2, size=[5000, 3]).astype(np.float32)
    
    gen = PointToVoxel(
        vsize_xyz=[0.2, 0.2, 0.2],
        coors_range_xyz=[-4, -4, -4, 4, 4, 4],
        num_point_features=3,
        max_num_voxels=10000,
        max_num_points_per_voxel=10
    )
    
    pc_th = torch.from_numpy(pc)
    voxels, indices, num_points = gen(pc_th)
    
    print(f"Generated {voxels.shape[0]} voxels for sparse convolution")
    
    spatial_shape = [40, 40, 40]  # Grid size based on coordinate range and voxel size
    batch_size = 1
    
    batch_indices = torch.zeros(indices.shape[0], 1, dtype=torch.int32)
    full_indices = torch.cat([batch_indices, indices.int()], dim=1)
    
    features = voxels.mean(dim=1)  # Average points in each voxel
    
    # Move tensors to CUDA before creating SparseConvTensor if available
    if torch.cuda.is_available():
        features = features.cuda()
        full_indices = full_indices.cuda()
    
    sparse_tensor = spconv.SparseConvTensor(
        features, full_indices, spatial_shape, batch_size
    )
    
    print(f"Created sparse tensor with {sparse_tensor.features.shape[0]} active voxels")
    print(f"Spatial shape: {sparse_tensor.spatial_shape}")
    
    conv_layer = spconv.SparseConv3d(3, 16, kernel_size=3, stride=1, padding=1)
    
    if torch.cuda.is_available():
        conv_layer = conv_layer.cuda()
        
    output = conv_layer(sparse_tensor)
    
    print(f"Convolution output shape: {output.features.shape}")
    print(f"Output indices shape: {output.indices.shape}")
    
    return True

def test_performance_benchmark():
    """Simple performance benchmark for voxel generation"""
    print("\n" + "=" * 60)
    print("Performance Benchmark")
    print("=" * 60)
    
    import time
    
    np.random.seed(50051)
    
    point_counts = [1000, 5000, 10000, 50000]
    
    for num_points in point_counts:
        pc = np.random.uniform(-5, 5, size=[num_points, 3]).astype(np.float32)
        
        gen = PointToVoxel(
            vsize_xyz=[0.1, 0.1, 0.1],
            coors_range_xyz=[-10, -10, -10, 10, 10, 10],
            num_point_features=3,
            max_num_voxels=20000,
            max_num_points_per_voxel=10
        )
        
        pc_th = torch.from_numpy(pc)
        
        start_time = time.time()
        voxels, indices, num_p = gen(pc_th)
        cpu_time = time.time() - start_time
        
        print(f"CPU - {num_points:5d} points -> {voxels.shape[0]:4d} voxels in {cpu_time:.4f}s")
        
        if torch.cuda.is_available():
            gen_cuda = PointToVoxel(
                vsize_xyz=[0.1, 0.1, 0.1],
                coors_range_xyz=[-10, -10, -10, 10, 10, 10],
                num_point_features=3,
                max_num_voxels=20000,
                max_num_points_per_voxel=10,
                device=torch.device("cuda:0")
            )
            
            pc_cuda = pc_th.cuda()
                
            _ = gen_cuda(pc_cuda)
            
            start_time = time.time()
            voxels_cuda, indices_cuda, num_p_cuda = gen_cuda(pc_cuda)
            cuda_time = time.time() - start_time
            
            speedup = cpu_time / cuda_time if cuda_time > 0 else float('inf')
            print(f"GPU - {num_points:5d} points -> {voxels_cuda.shape[0]:4d} voxels in {cuda_time:.4f}s (speedup: {speedup:.1f}x)")
    
    return True

def main():
    """Run all voxel grid tests"""
    print("SpConv Voxel Grid Test Suite")
    print("=" * 60)
    
    tests = [
        ("Basic Voxel Generation", test_basic_voxel_generation),
        ("Voxel with Mean Fill", test_voxel_with_mean_fill),
        ("Voxel with ID Generation", test_voxel_with_id_generation),
        ("CUDA Voxel Generation", test_cuda_voxel_generation),
        ("Sparse Convolution", test_sparse_convolution),
        ("Performance Benchmark", test_performance_benchmark),
    ]
    
    passed = 0
    failed = 0
    
    for test_name, test_func in tests:
        try:
            print(f"\nRunning: {test_name}")
            result = test_func()
            if result:
                print(f"✓ {test_name} PASSED")
                passed += 1
            else:
                print(f"✗ {test_name} FAILED")
                failed += 1
        except Exception as e:
            print(f"✗ {test_name} FAILED with exception: {e}")
            import traceback
            traceback.print_exc()
            failed += 1
    
    print("\n" + "=" * 60)
    print("Test Results Summary")
    print("=" * 60)
    print(f"Passed: {passed}")
    print(f"Failed: {failed}")
    print(f"Total:  {passed + failed}")
    
    if failed == 0:
        print("\n🎉 All tests passed! SpConv voxel grid functionality is working correctly.")
    else:
        print(f"\n⚠️  {failed} test(s) failed. Check the output above for details.")
    
    return failed == 0

if __name__ == "__main__":
    success = main()
    exit(0 if success else 1) 
    