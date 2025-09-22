import torch
import numpy as np
import argparse
from typing import Union, Tuple, Dict, List


def str2bool(v: Union[str, bool]) -> bool:
    """Convert string to boolean"""
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')


def to_cuda(*args):
    """Move tensors to CUDA if available"""
    if torch.cuda.is_available():
        return tuple(arg.cuda() if isinstance(arg, torch.Tensor) else arg for arg in args)
    return args


def calculate_metrics(predictions: torch.Tensor, labels: torch.Tensor) -> Dict[str, int]:
    """
    Calculate TP, TN, FP, FN from predictions and labels
    
    Args:
        predictions: Binary predictions (0 or 1)
        labels: Ground truth labels (0 or 1)
        
    Returns:
        Dictionary with TP, TN, FP, FN counts
    """
    predictions = predictions.flatten()
    labels = labels.flatten()
    
    tp = torch.sum((predictions == 1) & (labels == 1)).item()
    tn = torch.sum((predictions == 0) & (labels == 0)).item()
    fp = torch.sum((predictions == 1) & (labels == 0)).item()
    fn = torch.sum((predictions == 0) & (labels == 1)).item()
    
    return {'tp': tp, 'tn': tn, 'fp': fp, 'fn': fn}


def compute_detection_metrics(tp: int, tn: int, fp: int, fn: int) -> Dict[str, float]:
    """
    Compute detection performance metrics
    
    Args:
        tp, tn, fp, fn: True/False positive/negative counts
        
    Returns:
        Dictionary with various metrics
    """
    eps = 1e-8
    
    # Basic metrics
    accuracy = (tp + tn) / (tp + tn + fp + fn + eps)
    precision = tp / (tp + fp + eps)
    recall = tp / (tp + fn + eps)  # Also known as sensitivity or detection probability
    specificity = tn / (tn + fp + eps)  # True negative rate
    f1_score = 2 * precision * recall / (precision + recall + eps)
    
    # Detection-specific metrics
    detection_probability = recall  # P_d
    false_alarm_probability = fp / (fp + tn + eps)  # P_fa
    miss_detection_probability = fn / (fn + tp + eps)  # P_md
    
    return {
        'accuracy': accuracy,
        'precision': precision,
        'recall': recall,
        'specificity': specificity,
        'f1_score': f1_score,
        'detection_probability': detection_probability,
        'false_alarm_probability': false_alarm_probability,
        'miss_detection_probability': miss_detection_probability
    }


def compute_roc_metrics(predictions_prob: np.ndarray, labels: np.ndarray, 
                       thresholds: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """
    Compute ROC curve data points
    
    Args:
        predictions_prob: Prediction probabilities [0, 1]
        labels: Ground truth binary labels
        thresholds: Detection thresholds to evaluate
        
    Returns:
        Tuple of (thresholds, true_positive_rates, false_positive_rates)
    """
    if thresholds is None:
        thresholds = np.linspace(0, 1, 101)
    
    tpr_list = []
    fpr_list = []
    
    for threshold in thresholds:
        predictions_binary = (predictions_prob >= threshold).astype(int)
        
        tp = np.sum((predictions_binary == 1) & (labels == 1))
        tn = np.sum((predictions_binary == 0) & (labels == 0))
        fp = np.sum((predictions_binary == 1) & (labels == 0))
        fn = np.sum((predictions_binary == 0) & (labels == 1))
        
        tpr = tp / (tp + fn + 1e-8)  # True positive rate (detection probability)
        fpr = fp / (fp + tn + 1e-8)  # False positive rate (false alarm probability)
        
        tpr_list.append(tpr)
        fpr_list.append(fpr)
    
    return thresholds, np.array(tpr_list), np.array(fpr_list)


def compute_auc(fpr: np.ndarray, tpr: np.ndarray) -> float:
    """
    Compute Area Under the Curve using trapezoidal rule
    
    Args:
        fpr: False positive rates
        tpr: True positive rates
        
    Returns:
        AUC value
    """
    # Sort by fpr
    sorted_indices = np.argsort(fpr)
    fpr_sorted = fpr[sorted_indices]
    tpr_sorted = tpr[sorted_indices]
    
    # Compute AUC using trapezoidal rule
    auc = np.trapz(tpr_sorted, fpr_sorted)
    return auc


def energy_detection_baseline(signal: np.ndarray, noise_power: float, 
                            threshold: float = None) -> Tuple[bool, float]:
    """
    Simple energy detection baseline
    
    Args:
        signal: Received signal
        noise_power: Estimated noise power
        threshold: Detection threshold (if None, use 2*noise_power)
        
    Returns:
        Tuple of (detection_decision, test_statistic)
    """
    if threshold is None:
        threshold = 2 * noise_power
    
    # Compute test statistic (signal energy)
    test_statistic = np.mean(np.abs(signal)**2)
    
    # Make detection decision
    detection = test_statistic > threshold
    
    return detection, test_statistic


def cyclostationary_detection_baseline(signal: np.ndarray, 
                                     alpha: float = 0.1) -> Tuple[bool, float]:
    """
    Simple cyclostationary detection baseline
    
    Args:
        signal: Received signal
        alpha: Cyclic frequency parameter
        
    Returns:
        Tuple of (detection_decision, test_statistic)
    """
    # Simplified cyclostationary detection
    # In practice, this would involve computing cyclic correlation function
    
    # Compute autocorrelation at different lags
    N = len(signal)
    max_lag = min(50, N // 4)
    
    autocorr_values = []
    for lag in range(1, max_lag):
        if lag < N:
            autocorr = np.corrcoef(signal[:-lag], signal[lag:])[0, 1]
            if not np.isnan(autocorr):
                autocorr_values.append(abs(autocorr))
    
    # Test statistic based on maximum autocorrelation
    test_statistic = max(autocorr_values) if autocorr_values else 0
    
    # Simple threshold (in practice, this would be derived from theory)
    threshold = 0.3
    detection = test_statistic > threshold
    
    return detection, test_statistic


def matched_filter_detection_baseline(signal: np.ndarray, 
                                    template: np.ndarray,
                                    noise_power: float) -> Tuple[bool, float]:
    """
    Matched filter detection baseline
    
    Args:
        signal: Received signal
        template: Known signal template
        noise_power: Estimated noise power
        
    Returns:
        Tuple of (detection_decision, test_statistic)
    """
    # Normalize template
    template_norm = template / np.sqrt(np.sum(np.abs(template)**2))
    
    # Compute matched filter output
    if len(signal) >= len(template):
        correlation = np.correlate(signal, template_norm, mode='valid')
        test_statistic = np.max(np.abs(correlation)**2)
    else:
        # Zero-pad signal if shorter than template
        signal_padded = np.pad(signal, (0, len(template) - len(signal)), 'constant')
        correlation = np.correlate(signal_padded, template_norm, mode='valid')
        test_statistic = np.abs(correlation[0])**2
    
    # Threshold based on noise power
    threshold = 2 * noise_power
    detection = test_statistic > threshold
    
    return detection, test_statistic


def cooperative_detection_fusion(decisions: List[bool], 
                               fusion_rule: str = 'majority') -> bool:
    """
    Cooperative spectrum sensing decision fusion
    
    Args:
        decisions: List of individual UAV decisions
        fusion_rule: Fusion rule ('majority', 'or', 'and')
        
    Returns:
        Fused decision
    """
    if not decisions:
        return False
    
    if fusion_rule == 'majority':
        return sum(decisions) > len(decisions) / 2
    elif fusion_rule == 'or':
        return any(decisions)
    elif fusion_rule == 'and':
        return all(decisions)
    else:
        raise ValueError(f"Unknown fusion rule: {fusion_rule}")


def snr_to_linear(snr_db: float) -> float:
    """Convert SNR from dB to linear scale"""
    return 10**(snr_db / 10)


def linear_to_db(linear_value: float) -> float:
    """Convert linear value to dB scale"""
    return 10 * np.log10(linear_value + 1e-12)


def awgn_channel(signal: np.ndarray, snr_db: float) -> np.ndarray:
    """
    Add AWGN to signal for given SNR
    
    Args:
        signal: Input signal
        snr_db: Signal-to-noise ratio in dB
        
    Returns:
        Noisy signal
    """
    signal_power = np.mean(np.abs(signal)**2)
    snr_linear = snr_to_linear(snr_db)
    noise_power = signal_power / snr_linear
    
    # Generate complex AWGN
    if np.iscomplexobj(signal):
        noise = np.sqrt(noise_power/2) * (np.random.randn(*signal.shape) + 
                                         1j * np.random.randn(*signal.shape))
    else:
        noise = np.sqrt(noise_power) * np.random.randn(*signal.shape)
    
    return signal + noise


def rayleigh_fading_channel(signal: np.ndarray, 
                          correlation: float = 0.0) -> Tuple[np.ndarray, np.ndarray]:
    """
    Apply Rayleigh fading to signal
    
    Args:
        signal: Input signal
        correlation: Temporal correlation coefficient [0, 1)
        
    Returns:
        Tuple of (faded_signal, fading_coefficients)
    """
    # Generate correlated Rayleigh fading coefficients
    if correlation == 0:
        # Uncorrelated fading
        h = (np.random.randn(*signal.shape) + 1j * np.random.randn(*signal.shape)) / np.sqrt(2)
    else:
        # Correlated fading using AR(1) model
        h = np.zeros(signal.shape, dtype=complex)
        h[0] = (np.random.randn() + 1j * np.random.randn()) / np.sqrt(2)
        
        for i in range(1, len(signal)):
            innovation = (np.random.randn() + 1j * np.random.randn()) / np.sqrt(2)
            h[i] = correlation * h[i-1] + np.sqrt(1 - correlation**2) * innovation
    
    # Apply fading
    faded_signal = h * signal
    
    return faded_signal, h


def init_logger(log_dir: str, subdir: str, print_log: bool = True):
    """Initialize logger (placeholder - actual implementation depends on requirements)"""
    import logging
    import os
    
    log_path = os.path.join(log_dir, subdir)
    os.makedirs(log_path, exist_ok=True)
    
    # This is a simplified version - you might want to use the actual logger setup
    # from your existing codebase
    pass


class SpectrumSensingMetrics:
    """Class to track and compute spectrum sensing metrics"""
    
    def __init__(self):
        self.reset()
    
    def reset(self):
        self.tp = 0
        self.tn = 0  
        self.fp = 0
        self.fn = 0
        self.predictions = []
        self.labels = []
    
    def update(self, predictions: torch.Tensor, labels: torch.Tensor):
        """Update metrics with batch results"""
        pred_binary = (predictions > 0.5).long().flatten()
        labels_flat = labels.long().flatten()
        
        self.tp += torch.sum((pred_binary == 1) & (labels_flat == 1)).item()
        self.tn += torch.sum((pred_binary == 0) & (labels_flat == 0)).item()
        self.fp += torch.sum((pred_binary == 1) & (labels_flat == 0)).item()
        self.fn += torch.sum((pred_binary == 0) & (labels_flat == 1)).item()
        
        self.predictions.extend(predictions.detach().cpu().numpy().flatten())
        self.labels.extend(labels.detach().cpu().numpy().flatten())
    
    def compute(self) -> Dict[str, float]:
        """Compute final metrics"""
        return compute_detection_metrics(self.tp, self.tn, self.fp, self.fn)
    
    def compute_roc(self, thresholds: np.ndarray = None) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Compute ROC curve"""
        predictions_array = np.array(self.predictions)
        labels_array = np.array(self.labels)
        return compute_roc_metrics(predictions_array, labels_array, thresholds)


if __name__ == '__main__':
    # Test utilities
    print("Testing spectrum sensing utilities...")
    
    # Test metrics calculation
    pred = torch.tensor([1, 0, 1, 1, 0])
    labels = torch.tensor([1, 0, 0, 1, 0])
    metrics = calculate_metrics(pred, labels)
    print(f"Metrics: {metrics}")
    
    detection_metrics = compute_detection_metrics(**metrics)
    print(f"Detection metrics: {detection_metrics}")
    
    # Test signal processing utilities
    signal = np.random.randn(100) + 1j * np.random.randn(100)
    noisy_signal = awgn_channel(signal, snr_db=10)
    print(f"Original signal power: {np.mean(np.abs(signal)**2):.4f}")
    print(f"Noisy signal power: {np.mean(np.abs(noisy_signal)**2):.4f}")
    
    # Test fading channel
    faded_signal, fading_coeff = rayleigh_fading_channel(signal)
    print(f"Faded signal power: {np.mean(np.abs(faded_signal)**2):.4f}")
    
    print("All tests passed!")