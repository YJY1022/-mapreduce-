import argparse
import numpy as np
import os
import torch
import scipy.signal
import logging
from multiprocessing import Pool
from typing import Tuple, List, Dict
import pickle


def parse_args():
    args = argparse.ArgumentParser()
    args.add_argument('--num', type=int, default=1000, help='Number of samples to generate')
    args.add_argument('--seed', type=int, default=42, help='Random seed')
    args.add_argument('--save_path', type=str, required=True, help='Path to save dataset')
    args.add_argument('--num_uavs', type=int, default=10, help='Number of UAVs')
    args.add_argument('--num_time_steps', type=int, default=20, help='Number of time steps')
    args.add_argument('--num_freq_bins', type=int, default=16, help='Number of frequency bins')
    args.add_argument('--pu_on_prob', type=float, default=0.3, help='Probability of PU being ON')
    return args.parse_args()


reseed = False


class SpectrumSensingWorker:
    def __init__(self, num_uavs, num_time_steps, num_freq_bins, pu_on_prob):
        self.num_uavs = num_uavs
        self.num_time_steps = num_time_steps
        self.num_freq_bins = num_freq_bins
        self.pu_on_prob = pu_on_prob
        
        # Signal parameters (based on MATLAB code)
        self.fs = 25e6  # Sampling frequency (Hz)
        self.BW_actual = 18e6  # Signal bandwidth (Hz)
        self.rolloff = 0.25
        self.symbol_rate = self.BW_actual / (1 + self.rolloff)
        self.oversample_factor = max(4, round(self.fs / self.symbol_rate))
        self.Ts = 8e-6  # Sensing period (s)
        self.N = int(np.floor(self.Ts * (self.fs / self.oversample_factor)) * self.oversample_factor)
        
        # PU parameters
        self.P_t_dBm = 30  # PU transmit power (dBm)
        self.P_t_W = 10**((self.P_t_dBm - 30)/10)  # PU transmit power (W)
        self.M = 2  # BPSK modulation order
        
        # Channel parameters
        self.fc = 2.4e9  # Carrier frequency (Hz)
        self.c = 3e8  # Speed of light (m/s)
        self.env_A = 9.61  # LoS probability model constant A
        self.env_B = 0.16  # LoS probability model constant B
        self.nakagami_m = 2  # Nakagami-m parameter (for LoS)
        
        # Path loss parameters
        self.d0 = 1  # Reference distance (m)
        self.n_LoS = 1.8  # Path loss exponent for LoS
        self.n_NLoS = 3.5  # Path loss exponent for NLoS
        self.sigma_LoS_dB = 3  # Shadowing std dev for LoS
        self.sigma_NLoS_dB = 8  # Shadowing std dev for NLoS
        
        # Noise parameters
        self.noise_psd_dbm_hz = -174  # Noise power spectral density (dBm/Hz)
        self.F_dB = 5  # Noise figure (dB)
        self.F_linear = 10**(self.F_dB/10)
        self.BW = self.BW_actual
        self.noise_psd_w_hz = 10**((self.noise_psd_dbm_hz - 30)/10)  # (W/Hz)
        self.noise_power_W = self.noise_psd_w_hz * self.BW * self.F_linear
        self.sigma_n = np.sqrt(self.noise_power_W / 2)
        
        # PU state transition parameters
        self.P_off_off = 0.9
        self.P_off_on = 0.1
        self.P_on_off = 0.2
        self.P_on_on = 0.8

    def generate_uav_positions(self) -> np.ndarray:
        """Generate UAV positions"""
        # Generate UAV positions based on MATLAB code
        uav_h_dist_bins = np.linspace(0, 1000, self.num_uavs)
        uav_h_dist = uav_h_dist_bins + np.random.rand(self.num_uavs) * 10
        uav_z = 100 + np.random.rand(self.num_uavs) * 200
        uav_angle = np.random.rand(self.num_uavs) * np.pi / 2
        uav_x = uav_h_dist * np.cos(uav_angle)
        uav_y = uav_h_dist * np.sin(uav_angle)
        
        return np.array([uav_x, uav_y, uav_z])  # Shape: (3, num_uavs)

    def generate_pu_state_sequence(self) -> np.ndarray:
        """Generate PU state sequence using Markov chain"""
        pu_states = np.zeros(self.num_time_steps, dtype=int)
        current_state = np.random.randint(2)
        pu_states[0] = current_state
        
        for t in range(1, self.num_time_steps):
            r = np.random.rand()
            if current_state == 0:
                if r > self.P_off_off:
                    current_state = 1
            else:
                if r > self.P_on_on:
                    current_state = 0
            pu_states[t] = current_state
            
        return pu_states

    def generate_baseband_signal(self) -> np.ndarray:
        """Generate BPSK baseband signal"""
        # Generate random BPSK symbols
        num_symbols = int(np.floor(self.Ts * (self.fs / self.oversample_factor)))
        data_in = np.random.randint(0, self.M, num_symbols)
        
        # BPSK modulation
        tx_symbols = 2 * data_in - 1  # Map 0,1 to -1,1
        
        # Generate RRC filter
        filt_span_symbols = 10
        tx_rrc_filter = self._rcosdesign(self.rolloff, filt_span_symbols, self.oversample_factor)
        tx_rrc_filter = tx_rrc_filter / np.sqrt(np.sum(tx_rrc_filter**2))
        
        # Upsample and filter
        tx_upsampled = np.repeat(tx_symbols, self.oversample_factor)
        tx_filtered = np.convolve(tx_upsampled, tx_rrc_filter, mode='same')
        
        # Normalize and set power
        tx_filtered = tx_filtered / np.sqrt(np.mean(np.abs(tx_filtered)**2))
        tx_filtered = tx_filtered * np.sqrt(self.P_t_W)
        
        # Truncate to desired length
        total_tx_samples = num_symbols * self.oversample_factor
        if len(tx_filtered) > total_tx_samples:
            start_idx = (len(tx_filtered) - total_tx_samples) // 2
            tx_filtered = tx_filtered[start_idx:start_idx + total_tx_samples]
        
        return tx_filtered

    def _rcosdesign(self, beta, span, sps):
        """Generate root raised cosine filter"""
        delay = span * sps / 2
        t = np.arange(-delay, delay + 1) / sps
        
        # Avoid division by zero
        eps = np.finfo(float).eps
        t[np.abs(t) < eps] = eps
        
        # RRC formula
        numerator = np.sin(np.pi * t * (1 - beta)) + 4 * beta * t * np.cos(np.pi * t * (1 + beta))
        denominator = np.pi * t * (1 - (4 * beta * t)**2)
        
        # Handle special cases
        mask = np.abs(denominator) < eps
        numerator[mask] = beta / np.sqrt(2) * ((1 + 2/np.pi) * np.sin(np.pi/(4*beta)) + 
                                               (1 - 2/np.pi) * np.cos(np.pi/(4*beta)))
        denominator[mask] = 1
        
        h = numerator / denominator
        return h

    def simulate_channel_and_reception(self, tx_signal: np.ndarray, uav_pos: np.ndarray, 
                                     pu_pos: np.ndarray = np.array([0, 0, 0])) -> Tuple[np.ndarray, np.ndarray]:
        """Simulate channel and signal reception for all UAVs"""
        rx_signals = []
        snr_values = []
        
        for i in range(self.num_uavs):
            # Calculate distance and elevation angle
            horiz_dist = np.sqrt(uav_pos[0, i]**2 + uav_pos[1, i]**2)
            elevation_angle = np.degrees(np.arctan2(uav_pos[2, i], horiz_dist))
            dist = np.sqrt(horiz_dist**2 + uav_pos[2, i]**2)
            
            # LoS probability
            p_los = 1 / (1 + self.env_A * np.exp(-self.env_B * (elevation_angle - self.env_A)))
            is_los = np.random.rand() < p_los
            
            # Path loss calculation
            PL_d0_dB = 20 * np.log10(4 * np.pi * self.fc * self.d0 / self.c)
            
            if is_los:
                path_loss_dB = PL_d0_dB + 10 * self.n_LoS * np.log10(dist / self.d0) + \
                              self.sigma_LoS_dB * np.random.randn()
                # Nakagami-m fading
                h_small = np.sqrt(np.random.gamma(self.nakagami_m, 1/self.nakagami_m)) * \
                         np.exp(1j * np.random.rand() * 2 * np.pi)
            else:
                path_loss_dB = PL_d0_dB + 10 * self.n_NLoS * np.log10(dist / self.d0) + \
                              self.sigma_NLoS_dB * np.random.randn()
                # Rayleigh fading
                h_small = (np.random.randn() + 1j * np.random.randn()) / np.sqrt(2)
            
            path_loss_linear = 10**(-path_loss_dB / 10)
            h_channel = h_small * np.sqrt(path_loss_linear)
            
            # Signal through channel
            rx_no_noise = h_channel * tx_signal
            
            # Add AWGN
            awgn_noise = self.sigma_n * (np.random.randn(len(tx_signal)) + 
                                        1j * np.random.randn(len(tx_signal)))
            rx_signal = rx_no_noise + awgn_noise
            
            # Truncate to sensing window
            rx_signal_slice = rx_signal[:self.N]
            
            # Calculate SNR
            signal_power = np.mean(np.abs(rx_no_noise[:self.N])**2)
            snr_value = 10 * np.log10(signal_power / self.noise_power_W)
            
            rx_signals.append(rx_signal_slice)
            snr_values.append(snr_value)
        
        return np.array(rx_signals), np.array(snr_values)

    def extract_features(self, rx_signals: np.ndarray, uav_pos: np.ndarray) -> Dict:
        """Extract time-domain, frequency-domain, and spatial features"""
        features = {}
        
        # Time-domain features
        time_features = []
        for i in range(self.num_uavs):
            signal = rx_signals[i]
            # Energy
            energy = np.mean(np.abs(signal)**2)
            # Peak-to-average power ratio
            papr = np.max(np.abs(signal)**2) / np.mean(np.abs(signal)**2)
            # Variance
            variance = np.var(np.abs(signal))
            # Kurtosis
            kurtosis = np.mean(np.abs(signal)**4) / (np.mean(np.abs(signal)**2)**2)
            
            time_features.append([energy, papr, variance, kurtosis])
        
        features['time'] = np.array(time_features)  # Shape: (num_uavs, 4)
        
        # Frequency-domain features
        freq_features = []
        for i in range(self.num_uavs):
            signal = rx_signals[i]
            # FFT
            fft_signal = np.fft.fft(signal)
            # Power spectral density using Welch's method
            f, psd = scipy.signal.welch(signal, fs=self.fs, nperseg=min(256, len(signal)//4))
            
            # Divide into frequency bins
            freq_bins = np.linspace(0, len(psd)-1, self.num_freq_bins+1, dtype=int)
            freq_bin_features = []
            for j in range(self.num_freq_bins):
                start_idx, end_idx = freq_bins[j], freq_bins[j+1]
                bin_power = np.mean(psd[start_idx:end_idx])
                freq_bin_features.append(bin_power)
            
            freq_features.append(freq_bin_features)
        
        features['freq'] = np.array(freq_features)  # Shape: (num_uavs, num_freq_bins)
        
        # Spatial features
        spatial_features = []
        for i in range(self.num_uavs):
            # Distance to origin (PU)
            dist_to_pu = np.sqrt(np.sum(uav_pos[:, i]**2))
            # Elevation angle
            horiz_dist = np.sqrt(uav_pos[0, i]**2 + uav_pos[1, i]**2)
            elevation = np.arctan2(uav_pos[2, i], horiz_dist)
            # Azimuth angle
            azimuth = np.arctan2(uav_pos[1, i], uav_pos[0, i])
            
            spatial_features.append([dist_to_pu, elevation, azimuth])
        
        features['spatial'] = np.array(spatial_features)  # Shape: (num_uavs, 3)
        
        return features

    def __call__(self, idx):
        global reseed
        if not reseed:
            np.random.seed(os.getpid() + idx)
            logging.info('reseed at process: {}'.format(os.getpid()))
            reseed = True

        # Generate UAV positions
        uav_pos = self.generate_uav_positions()
        
        # Generate PU state sequence
        pu_states = self.generate_pu_state_sequence()
        
        # Generate features for each time step
        all_features = []
        all_labels = []
        
        for t in range(self.num_time_steps):
            if pu_states[t] == 1:
                # PU is ON - generate signal
                tx_signal = self.generate_baseband_signal()
            else:
                # PU is OFF - no signal
                tx_signal = np.zeros(self.N, dtype=complex)
            
            # Simulate reception
            rx_signals, snr_values = self.simulate_channel_and_reception(tx_signal, uav_pos)
            
            # Extract features
            features = self.extract_features(rx_signals, uav_pos)
            
            # Store features and labels
            all_features.append(features)
            all_labels.append(pu_states[t])
        
        return {
            'features': all_features,
            'labels': all_labels,
            'uav_positions': uav_pos,
            'pu_states': pu_states
        }


def generate_dataset():
    args = parse_args()
    
    logging.basicConfig(filename='gen_spectrum_data.log', level=logging.INFO)
    
    worker = SpectrumSensingWorker(
        args.num_uavs, 
        args.num_time_steps, 
        args.num_freq_bins, 
        args.pu_on_prob
    )
    
    # Generate dataset using multiprocessing
    with Pool(8) as pool:
        data = pool.map(worker, list(range(args.num)))
    
    # Process and save data
    all_time_features = []
    all_freq_features = []
    all_spatial_features = []
    all_labels = []
    all_uav_positions = []
    
    for sample in data:
        sample_time_features = []
        sample_freq_features = []
        sample_spatial_features = []
        sample_labels = []
        
        for t in range(args.num_time_steps):
            sample_time_features.append(sample['features'][t]['time'])
            sample_freq_features.append(sample['features'][t]['freq'])
            sample_spatial_features.append(sample['features'][t]['spatial'])
            sample_labels.append(sample['labels'][t])
        
        all_time_features.append(np.array(sample_time_features))
        all_freq_features.append(np.array(sample_freq_features))
        all_spatial_features.append(np.array(sample_spatial_features))
        all_labels.append(np.array(sample_labels))
        all_uav_positions.append(sample['uav_positions'])
    
    # Convert to tensors
    time_features = torch.FloatTensor(np.array(all_time_features))
    freq_features = torch.FloatTensor(np.array(all_freq_features))
    spatial_features = torch.FloatTensor(np.array(all_spatial_features))
    labels = torch.LongTensor(np.array(all_labels))
    uav_positions = torch.FloatTensor(np.array(all_uav_positions))
    
    print(f"Dataset shapes:")
    print(f"Time features: {time_features.shape}")  # (num_samples, num_time_steps, num_uavs, 4)
    print(f"Freq features: {freq_features.shape}")  # (num_samples, num_time_steps, num_uavs, num_freq_bins)
    print(f"Spatial features: {spatial_features.shape}")  # (num_samples, num_time_steps, num_uavs, 3)
    print(f"Labels: {labels.shape}")  # (num_samples, num_time_steps)
    print(f"UAV positions: {uav_positions.shape}")  # (num_samples, 3, num_uavs)
    
    # Save dataset
    torch.save({
        'time_features': time_features,
        'freq_features': freq_features,
        'spatial_features': spatial_features,
        'labels': labels,
        'uav_positions': uav_positions,
        'args': args
    }, args.save_path)
    
    logging.info(f'Dataset saved to {args.save_path}')
    print(f'Dataset generation completed. Saved to {args.save_path}')


if __name__ == '__main__':
    generate_dataset()