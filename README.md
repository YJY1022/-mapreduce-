# Spectrum Sensing Factor Graph Neural Network (FGNN)

This repository implements a Factor Graph Neural Network (FGNN) for cooperative spectrum sensing in UAV networks, inspired by the LDPC FGNN architecture but adapted for spectrum sensing applications.

## Overview

The system models spectrum sensing as a factor graph where:
- **Variable nodes** represent UAV occupancy states across time, frequency, and spatial domains
- **Factor nodes** represent constraints between variable nodes (spatial, temporal, frequency correlations)
- **Message passing** enables cooperative information fusion across UAVs

## Architecture

### Variable Nodes
Each variable node represents the primary user (PU) occupancy state for a specific UAV at a given time and frequency:
- **Node count**: `num_uavs × num_time_steps × num_freq_bins`
- **Features**: Time-domain, frequency-domain, and spatial features extracted from received signals

### Factor Nodes
Three types of factor nodes capture different correlations:

1. **Spatial Factor Nodes**: Capture spatial correlations between UAVs at the same time and frequency
2. **Temporal Factor Nodes**: Capture temporal continuity between adjacent time steps  
3. **Frequency Factor Nodes**: Capture frequency selectivity between adjacent frequency bins

### Message Passing
The FGNN performs iterative message passing between variable and factor nodes to refine occupancy predictions through cooperative information fusion.

## Files Structure

```
├── spectrum_sensing_data.py          # Dataset generation
├── spectrum_sensing_model.py         # FGNN model implementation
├── spectrum_sensing_train.py         # Training and testing scripts
├── spectrum_sensing_utils.py         # Utility functions
├── example_usage.py                  # Example usage script
└── README.md                         # This file
```

## Quick Start

### 1. Run the Complete Example

The easiest way to get started is using the example script:

```bash
python example_usage.py --dataset_size 500 --epochs 50
```

This will:
1. Generate a dataset with 500 samples
2. Train the FGNN model for 50 epochs  
3. Test the trained model and display results

### 2. Step-by-Step Usage

#### Generate Dataset

```bash
python spectrum_sensing_data.py \
    --num 1000 \
    --seed 42 \
    --save_path data/spectrum_dataset.pt \
    --num_uavs 10 \
    --num_time_steps 20 \
    --num_freq_bins 16 \
    --pu_on_prob 0.3
```

#### Train Model

```bash
python spectrum_sensing_train.py \
    --train \
    --data_path data/spectrum_dataset.pt \
    --n_epochs 100 \
    --batch_size 32 \
    --lr 0.001 \
    --hidden_dim 64 \
    --num_message_passing_steps 5
```

#### Test Model

```bash
python spectrum_sensing_train.py \
    --test \
    --data_path data/spectrum_dataset.pt \
    --model_path models_spectrum_sensing/path/to/model.pt
```

## Dataset Generation

The dataset generation simulates:

### Signal Model
- **Modulation**: BPSK with root raised cosine (RRC) filtering
- **Channel**: Air-to-ground (A2G) path loss with LoS/NLoS states
- **Fading**: Nakagami-m (LoS) and Rayleigh (NLoS) small-scale fading
- **Noise**: AWGN with realistic noise figure

### UAV Deployment
- **Positions**: Random 3D positions with realistic altitude constraints
- **Mobility**: Static positions per sample (can be extended to mobile scenarios)

### PU Activity
- **State Model**: Markov chain with ON/OFF states
- **Transitions**: Configurable transition probabilities

### Feature Extraction
For each UAV and time step, the system extracts:

- **Time-domain**: Energy, PAPR, variance, kurtosis
- **Frequency-domain**: Power spectral density across frequency bins
- **Spatial**: Distance to PU, elevation angle, azimuth angle

## Model Architecture

### FGNN Components

```python
class SpectrumSensingFGNN(nn.Module):
    def __init__(self, 
                 num_uavs=10,
                 num_time_steps=20, 
                 num_freq_bins=16,
                 hidden_dim=64,
                 num_message_passing_steps=5):
```

Key components:
- **Feature encoders**: Separate encoders for time, frequency, and spatial features
- **Factor networks**: Neural networks representing spatial, temporal, and frequency factors
- **Message passing**: Iterative variable-to-factor and factor-to-variable message updates
- **Classifier**: Final binary classification for PU occupancy

### Training

- **Loss function**: Binary cross-entropy with logits
- **Optimizer**: Adam with learning rate scheduling
- **Metrics**: Accuracy, precision, recall, F1-score, detection/false alarm probabilities

## Performance Metrics

The system evaluates performance using standard spectrum sensing metrics:

- **Detection Probability (P_d)**: Probability of correctly detecting PU presence
- **False Alarm Probability (P_fa)**: Probability of false PU detection  
- **Miss Detection Probability (P_md)**: Probability of missing PU presence
- **ROC Curves**: P_d vs P_fa trade-offs across detection thresholds

## Configuration Parameters

### Dataset Generation
- `--num`: Number of samples to generate
- `--num_uavs`: Number of UAVs in the network
- `--num_time_steps`: Number of time steps per sample
- `--num_freq_bins`: Number of frequency bins
- `--pu_on_prob`: Probability of PU being active

### Model Training
- `--hidden_dim`: Hidden dimension for neural networks
- `--num_message_passing_steps`: Number of message passing iterations
- `--lr`: Learning rate
- `--batch_size`: Training batch size
- `--n_epochs`: Number of training epochs

## Requirements

```bash
pip install torch torchvision numpy scipy tensorboardX tqdm
```

## Extensions

The framework can be extended to support:

1. **Mobile UAVs**: Time-varying UAV positions
2. **Multiple PUs**: Multiple primary users with different characteristics  
3. **Heterogeneous UAVs**: Different UAV capabilities and sensing parameters
4. **Advanced Channels**: More sophisticated channel models (Rician, shadowing correlation)
5. **Real Data**: Integration with real spectrum sensing datasets

## Comparison with Baselines

The repository includes baseline implementations:
- **Energy Detection**: Simple energy-based detection
- **Cyclostationary Detection**: Feature-based detection
- **Matched Filter**: Template-based detection

## Visualization

Training progress can be monitored using TensorBoard:

```bash
tensorboard --logdir=./tf_logs
```

This displays:
- Training/validation loss curves
- Accuracy and F1-score trends
- Learning rate schedules

## Citation

If you use this code in your research, please cite:

```bibtex
@article{spectrum_sensing_fgnn,
  title={Factor Graph Neural Networks for Cooperative Spectrum Sensing in UAV Networks},
  author={Your Name},
  journal={Your Journal},
  year={2024}
}
```

## License

MIT License - see LICENSE file for details.

## Contact

For questions or issues, please open a GitHub issue or contact [your-email@domain.com].