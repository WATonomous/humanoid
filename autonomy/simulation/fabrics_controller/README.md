# Fabrics Controller

Geometric fabric controller for the humanoid arm + hand with PCA-compressed hand control. 15 DOF hand compressed to 7 PCA dimensions.

## Setup

```bash
# Initialize the FABRICS submodule (only needed once)
git submodule update --init

# Install FABRICS
cd FABRICS
pip install -e .
chmod +x urdfpy_patch.sh
./urdfpy_patch.sh
cd ..

# Copy params file to where FABRICS expects it
cp humanoid_hand_params.yaml FABRICS/src/fabrics_sim/fabric_params/
```

## Run

```bash
cd autonomy/simulation/fabrics_controller
python3 run_example.py --batch_size=1
```