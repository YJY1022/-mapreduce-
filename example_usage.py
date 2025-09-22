#!/usr/bin/env python3
"""
Example usage script for Spectrum Sensing FGNN

This script demonstrates how to:
1. Generate a spectrum sensing dataset
2. Train the FGNN model
3. Test the trained model
"""

import os
import subprocess
import argparse


def run_command(cmd, description):
    """Run a command and handle errors"""
    print(f"\n{'='*60}")
    print(f"RUNNING: {description}")
    print(f"COMMAND: {cmd}")
    print(f"{'='*60}")
    
    try:
        result = subprocess.run(cmd, shell=True, check=True, capture_output=True, text=True)
        print("SUCCESS!")
        if result.stdout:
            print("STDOUT:", result.stdout[-500:])  # Show last 500 chars
        return True
    except subprocess.CalledProcessError as e:
        print(f"ERROR: Command failed with return code {e.returncode}")
        if e.stdout:
            print("STDOUT:", e.stdout[-500:])
        if e.stderr:
            print("STDERR:", e.stderr[-500:])
        return False


def main():
    parser = argparse.ArgumentParser(description='Spectrum Sensing FGNN Example')
    parser.add_argument('--skip_data_generation', action='store_true', 
                       help='Skip dataset generation step')
    parser.add_argument('--skip_training', action='store_true',
                       help='Skip training step') 
    parser.add_argument('--skip_testing', action='store_true',
                       help='Skip testing step')
    parser.add_argument('--dataset_size', type=int, default=100,
                       help='Number of samples to generate')
    parser.add_argument('--epochs', type=int, default=20,
                       help='Number of training epochs')
    
    args = parser.parse_args()
    
    # Create directories
    os.makedirs('data', exist_ok=True)
    os.makedirs('logs', exist_ok=True)
    os.makedirs('tf_logs', exist_ok=True)
    os.makedirs('models_spectrum_sensing', exist_ok=True)
    
    dataset_path = 'data/spectrum_sensing_dataset.pt'
    
    # Step 1: Generate dataset
    if not args.skip_data_generation:
        print("Step 1: Generating spectrum sensing dataset...")
        cmd = f"""python spectrum_sensing_data.py \
            --num {args.dataset_size} \
            --seed 42 \
            --save_path {dataset_path} \
            --num_uavs 10 \
            --num_time_steps 20 \
            --num_freq_bins 16 \
            --pu_on_prob 0.3"""
        
        if not run_command(cmd, "Dataset Generation"):
            print("Dataset generation failed. Exiting.")
            return
    else:
        print("Skipping dataset generation...")
        if not os.path.exists(dataset_path):
            print(f"ERROR: Dataset not found at {dataset_path}")
            print("Please run without --skip_data_generation first")
            return
    
    # Step 2: Train model
    model_path = None
    if not args.skip_training:
        print("\nStep 2: Training FGNN model...")
        cmd = f"""python spectrum_sensing_train.py \
            --train \
            --data_path {dataset_path} \
            --n_epochs {args.epochs} \
            --batch_size 16 \
            --lr 0.001 \
            --hidden_dim 64 \
            --num_message_passing_steps 5 \
            --save_every 10 \
            --log_every 5 \
            --num_workers 2"""
        
        if not run_command(cmd, "Model Training"):
            print("Training failed. Exiting.")
            return
        
        # Find the best model
        import glob
        model_files = glob.glob('models_spectrum_sensing/*/SpectrumSensingFGNN_best.pt')
        if model_files:
            model_path = model_files[-1]  # Use the most recent one
            print(f"Found trained model: {model_path}")
        else:
            print("No trained model found. Cannot proceed to testing.")
            return
    else:
        print("Skipping training...")
        # Try to find existing model
        import glob
        model_files = glob.glob('models_spectrum_sensing/*/SpectrumSensingFGNN_best.pt')
        if model_files:
            model_path = model_files[-1]
            print(f"Using existing model: {model_path}")
        else:
            print("No existing model found. Please run without --skip_training first")
            return
    
    # Step 3: Test model
    if not args.skip_testing and model_path:
        print("\nStep 3: Testing trained model...")
        cmd = f"""python spectrum_sensing_train.py \
            --test \
            --data_path {dataset_path} \
            --model_path {model_path} \
            --batch_size 32 \
            --num_workers 2"""
        
        if not run_command(cmd, "Model Testing"):
            print("Testing failed.")
        else:
            print("Testing completed successfully!")
    else:
        print("Skipping testing...")
    
    print("\n" + "="*60)
    print("EXAMPLE COMPLETED!")
    print("="*60)
    print("\nGenerated files:")
    print(f"- Dataset: {dataset_path}")
    if model_path:
        print(f"- Model: {model_path}")
        print(f"- Test results: {model_path.replace('.pt', '_test_results.json')}")
    print("- Logs: ./logs/")
    print("- TensorBoard logs: ./tf_logs/")
    
    print("\nTo view training progress:")
    print("tensorboard --logdir=./tf_logs")


if __name__ == '__main__':
    main()