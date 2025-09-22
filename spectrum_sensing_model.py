import torch
import torch.nn as nn
import torch.nn.functional as F
import numpy as np
from typing import Tuple, List, Dict, Optional


class SpectrumSensingFGNN(nn.Module):
    """
    Factor Graph Neural Network for Spectrum Sensing
    
    Variable nodes: represent UAV network occupancy status in time, frequency, and space domains
    Factor nodes: represent constraints between variable nodes (spatial, temporal, frequency)
    """
    
    def __init__(self, 
                 num_uavs: int = 10,
                 num_time_steps: int = 20, 
                 num_freq_bins: int = 16,
                 time_feature_dim: int = 4,
                 freq_feature_dim: int = 16,
                 spatial_feature_dim: int = 3,
                 hidden_dim: int = 64,
                 num_message_passing_steps: int = 5,
                 dropout: float = 0.1):
        super(SpectrumSensingFGNN, self).__init__()
        
        self.num_uavs = num_uavs
        self.num_time_steps = num_time_steps
        self.num_freq_bins = num_freq_bins
        self.hidden_dim = hidden_dim
        self.num_message_passing_steps = num_message_passing_steps
        
        # Feature encoders
        self.time_encoder = nn.Sequential(
            nn.Linear(time_feature_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, hidden_dim // 4)
        )
        
        self.freq_encoder = nn.Sequential(
            nn.Linear(freq_feature_dim, hidden_dim // 2),
            nn.ReLU(), 
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, hidden_dim // 4)
        )
        
        self.spatial_encoder = nn.Sequential(
            nn.Linear(spatial_feature_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, hidden_dim // 4)
        )
        
        # Variable node embedding
        self.variable_embedding = nn.Sequential(
            nn.Linear(3 * (hidden_dim // 4), hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim)
        )
        
        # Factor node networks
        self.spatial_factor_net = FactorNetwork(hidden_dim, hidden_dim)
        self.temporal_factor_net = FactorNetwork(hidden_dim, hidden_dim) 
        self.frequency_factor_net = FactorNetwork(hidden_dim, hidden_dim)
        
        # Message passing networks
        self.var_to_factor_net = nn.Sequential(
            nn.Linear(hidden_dim + hidden_dim, hidden_dim),  # variable + edge features
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim)
        )
        
        self.factor_to_var_net = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim, hidden_dim)
        )
        
        # Output classifier
        self.classifier = nn.Sequential(
            nn.Linear(hidden_dim, hidden_dim // 2),
            nn.ReLU(),
            nn.Dropout(dropout),
            nn.Linear(hidden_dim // 2, 1)
        )
        
        # Edge feature networks
        self.spatial_edge_net = nn.Sequential(
            nn.Linear(1, hidden_dim // 4),  # distance feature
            nn.ReLU(),
            nn.Linear(hidden_dim // 4, hidden_dim)
        )
        
        self.temporal_edge_net = nn.Sequential(
            nn.Linear(1, hidden_dim // 4),  # time difference
            nn.ReLU(), 
            nn.Linear(hidden_dim // 4, hidden_dim)
        )
        
        self.frequency_edge_net = nn.Sequential(
            nn.Linear(1, hidden_dim // 4),  # frequency difference
            nn.ReLU(),
            nn.Linear(hidden_dim // 4, hidden_dim)
        )

    def forward(self, time_features: torch.Tensor, freq_features: torch.Tensor, 
                spatial_features: torch.Tensor, uav_positions: torch.Tensor) -> torch.Tensor:
        """
        Forward pass
        
        Args:
            time_features: (batch_size, num_time_steps, num_uavs, time_feature_dim)
            freq_features: (batch_size, num_time_steps, num_uavs, freq_feature_dim) 
            spatial_features: (batch_size, num_time_steps, num_uavs, spatial_feature_dim)
            uav_positions: (batch_size, 3, num_uavs)
            
        Returns:
            predictions: (batch_size, num_time_steps)
        """
        batch_size = time_features.size(0)
        
        # Encode features
        time_encoded = self.time_encoder(time_features)  # (B, T, U, H/4)
        freq_encoded = self.freq_encoder(freq_features)  # (B, T, U, H/4) 
        spatial_encoded = self.spatial_encoder(spatial_features)  # (B, T, U, H/4)
        
        # Combine features for variable nodes
        var_features = torch.cat([time_encoded, freq_encoded, spatial_encoded], dim=-1)  # (B, T, U, 3*H/4)
        var_embeddings = self.variable_embedding(var_features)  # (B, T, U, H)
        
        # Message passing
        for step in range(self.num_message_passing_steps):
            var_embeddings = self.message_passing_step(var_embeddings, uav_positions)
        
        # Global pooling across UAVs for each time step
        # Aggregate information from all UAVs to predict PU state at each time step
        pooled_features = torch.mean(var_embeddings, dim=2)  # (B, T, H)
        
        # Classification
        predictions = self.classifier(pooled_features).squeeze(-1)  # (B, T)
        
        return predictions

    def message_passing_step(self, var_embeddings: torch.Tensor, 
                           uav_positions: torch.Tensor) -> torch.Tensor:
        """
        Single message passing step
        
        Args:
            var_embeddings: (batch_size, num_time_steps, num_uavs, hidden_dim)
            uav_positions: (batch_size, 3, num_uavs)
            
        Returns:
            updated_var_embeddings: (batch_size, num_time_steps, num_uavs, hidden_dim)
        """
        batch_size, num_time_steps, num_uavs, hidden_dim = var_embeddings.shape
        
        # Spatial message passing
        spatial_messages = self.spatial_message_passing(var_embeddings, uav_positions)
        
        # Temporal message passing  
        temporal_messages = self.temporal_message_passing(var_embeddings)
        
        # Frequency message passing
        frequency_messages = self.frequency_message_passing(var_embeddings)
        
        # Aggregate messages
        total_messages = spatial_messages + temporal_messages + frequency_messages
        
        # Update variable embeddings
        updated_embeddings = var_embeddings + total_messages
        
        return updated_embeddings

    def spatial_message_passing(self, var_embeddings: torch.Tensor, 
                              uav_positions: torch.Tensor) -> torch.Tensor:
        """
        Spatial message passing between UAVs at same time and frequency
        """
        batch_size, num_time_steps, num_uavs, hidden_dim = var_embeddings.shape
        
        # Compute spatial distances
        spatial_distances = self.compute_spatial_distances(uav_positions)  # (B, U, U)
        
        messages = torch.zeros_like(var_embeddings)
        
        for t in range(num_time_steps):
            for u in range(num_uavs):
                # Collect messages from other UAVs
                uav_messages = []
                for v in range(num_uavs):
                    if u != v:
                        # Edge features (distance)
                        edge_feat = spatial_distances[:, u, v].unsqueeze(-1)  # (B, 1)
                        edge_embedding = self.spatial_edge_net(edge_feat)  # (B, H)
                        
                        # Variable to factor message
                        var_feat = var_embeddings[:, t, v, :]  # (B, H)
                        combined = torch.cat([var_feat, edge_embedding], dim=-1)  # (B, 2H)
                        message = self.var_to_factor_net(combined)  # (B, H)
                        uav_messages.append(message)
                
                if uav_messages:
                    # Aggregate messages from spatial factor
                    aggregated = torch.stack(uav_messages, dim=1).mean(dim=1)  # (B, H)
                    factor_message = self.spatial_factor_net(aggregated)  # (B, H)
                    messages[:, t, u, :] = self.factor_to_var_net(factor_message)
        
        return messages

    def temporal_message_passing(self, var_embeddings: torch.Tensor) -> torch.Tensor:
        """
        Temporal message passing between adjacent time steps
        """
        batch_size, num_time_steps, num_uavs, hidden_dim = var_embeddings.shape
        
        messages = torch.zeros_like(var_embeddings)
        
        for u in range(num_uavs):
            for t in range(num_time_steps):
                uav_messages = []
                
                # Previous time step
                if t > 0:
                    edge_feat = torch.ones(batch_size, 1, device=var_embeddings.device)  # Time diff = 1
                    edge_embedding = self.temporal_edge_net(edge_feat)  # (B, H)
                    
                    var_feat = var_embeddings[:, t-1, u, :]  # (B, H)
                    combined = torch.cat([var_feat, edge_embedding], dim=-1)
                    message = self.var_to_factor_net(combined)
                    uav_messages.append(message)
                
                # Next time step  
                if t < num_time_steps - 1:
                    edge_feat = torch.ones(batch_size, 1, device=var_embeddings.device)
                    edge_embedding = self.temporal_edge_net(edge_feat)
                    
                    var_feat = var_embeddings[:, t+1, u, :]  # (B, H)
                    combined = torch.cat([var_feat, edge_embedding], dim=-1)
                    message = self.var_to_factor_net(combined)
                    uav_messages.append(message)
                
                if uav_messages:
                    aggregated = torch.stack(uav_messages, dim=1).mean(dim=1)
                    factor_message = self.temporal_factor_net(aggregated)
                    messages[:, t, u, :] = self.factor_to_var_net(factor_message)
        
        return messages

    def frequency_message_passing(self, var_embeddings: torch.Tensor) -> torch.Tensor:
        """
        Frequency message passing between adjacent frequency bins
        Note: For simplicity, we simulate frequency correlation by averaging across UAVs
        """
        batch_size, num_time_steps, num_uavs, hidden_dim = var_embeddings.shape
        
        messages = torch.zeros_like(var_embeddings)
        
        # Simple frequency correlation: each UAV receives messages from neighboring UAVs
        # This simulates frequency selectivity effects
        for t in range(num_time_steps):
            for u in range(num_uavs):
                freq_messages = []
                
                # Messages from nearby UAVs (simulating frequency correlation)
                for v in range(max(0, u-1), min(num_uavs, u+2)):
                    if v != u:
                        freq_diff = torch.tensor([abs(u - v)], dtype=torch.float32, 
                                               device=var_embeddings.device).unsqueeze(0).repeat(batch_size, 1)
                        edge_embedding = self.frequency_edge_net(freq_diff)
                        
                        var_feat = var_embeddings[:, t, v, :]
                        combined = torch.cat([var_feat, edge_embedding], dim=-1)
                        message = self.var_to_factor_net(combined)
                        freq_messages.append(message)
                
                if freq_messages:
                    aggregated = torch.stack(freq_messages, dim=1).mean(dim=1)
                    factor_message = self.frequency_factor_net(aggregated)
                    messages[:, t, u, :] = self.factor_to_var_net(factor_message)
        
        return messages

    def compute_spatial_distances(self, uav_positions: torch.Tensor) -> torch.Tensor:
        """
        Compute pairwise distances between UAVs
        
        Args:
            uav_positions: (batch_size, 3, num_uavs)
            
        Returns:
            distances: (batch_size, num_uavs, num_uavs)
        """
        batch_size, _, num_uavs = uav_positions.shape
        
        # Expand dimensions for broadcasting
        pos_i = uav_positions.unsqueeze(3)  # (B, 3, U, 1)
        pos_j = uav_positions.unsqueeze(2)  # (B, 3, 1, U)
        
        # Compute squared distances
        diff = pos_i - pos_j  # (B, 3, U, U)
        distances = torch.sqrt(torch.sum(diff ** 2, dim=1) + 1e-8)  # (B, U, U)
        
        return distances


class FactorNetwork(nn.Module):
    """
    Neural network representing a factor node
    """
    
    def __init__(self, input_dim: int, output_dim: int):
        super(FactorNetwork, self).__init__()
        
        self.network = nn.Sequential(
            nn.Linear(input_dim, output_dim),
            nn.ReLU(),
            nn.Linear(output_dim, output_dim),
            nn.ReLU(),
            nn.Linear(output_dim, output_dim)
        )
    
    def forward(self, x: torch.Tensor) -> torch.Tensor:
        return self.network(x)


class SpectrumSensingDataset(torch.utils.data.Dataset):
    """
    Dataset class for spectrum sensing
    """
    
    def __init__(self, data_path: str, train: bool = True):
        self.data = torch.load(data_path)
        self.train = train
        
        # Split data
        total_samples = self.data['labels'].size(0)
        train_size = int(0.8 * total_samples)
        
        if train:
            self.time_features = self.data['time_features'][:train_size]
            self.freq_features = self.data['freq_features'][:train_size]
            self.spatial_features = self.data['spatial_features'][:train_size]
            self.labels = self.data['labels'][:train_size]
            self.uav_positions = self.data['uav_positions'][:train_size]
        else:
            self.time_features = self.data['time_features'][train_size:]
            self.freq_features = self.data['freq_features'][train_size:]
            self.spatial_features = self.data['spatial_features'][train_size:]
            self.labels = self.data['labels'][train_size:]
            self.uav_positions = self.data['uav_positions'][train_size:]
    
    def __len__(self):
        return self.labels.size(0)
    
    def __getitem__(self, idx):
        return {
            'time_features': self.time_features[idx],
            'freq_features': self.freq_features[idx], 
            'spatial_features': self.spatial_features[idx],
            'labels': self.labels[idx],
            'uav_positions': self.uav_positions[idx]
        }


if __name__ == '__main__':
    # Test model
    model = SpectrumSensingFGNN()
    
    # Create dummy data
    batch_size = 4
    time_features = torch.randn(batch_size, 20, 10, 4)
    freq_features = torch.randn(batch_size, 20, 10, 16)
    spatial_features = torch.randn(batch_size, 20, 10, 3)
    uav_positions = torch.randn(batch_size, 3, 10)
    
    # Forward pass
    predictions = model(time_features, freq_features, spatial_features, uav_positions)
    print(f"Predictions shape: {predictions.shape}")
    print("Model test passed!")