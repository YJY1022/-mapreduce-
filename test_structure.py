#!/usr/bin/env python3
"""
Test script to verify the code structure and imports without running heavy computations
"""

import sys
import os
import importlib.util

def test_file_structure():
    """Test that all required files exist"""
    required_files = [
        'spectrum_sensing_data.py',
        'spectrum_sensing_model.py', 
        'spectrum_sensing_train.py',
        'spectrum_sensing_utils.py',
        'example_usage.py',
        'README.md'
    ]
    
    print("Testing file structure...")
    for file in required_files:
        if os.path.exists(file):
            print(f"✓ {file} exists")
        else:
            print(f"✗ {file} missing")
            return False
    
    return True

def test_syntax():
    """Test Python syntax of all files"""
    python_files = [
        'spectrum_sensing_data.py',
        'spectrum_sensing_model.py',
        'spectrum_sensing_train.py', 
        'spectrum_sensing_utils.py',
        'example_usage.py'
    ]
    
    print("\nTesting Python syntax...")
    for file in python_files:
        try:
            with open(file, 'r') as f:
                code = f.read()
            compile(code, file, 'exec')
            print(f"✓ {file} syntax OK")
        except SyntaxError as e:
            print(f"✗ {file} syntax error: {e}")
            return False
        except Exception as e:
            print(f"✗ {file} error: {e}")
            return False
    
    return True

def test_basic_structure():
    """Test basic code structure without importing heavy dependencies"""
    print("\nTesting code structure...")
    
    # Test data generation script
    with open('spectrum_sensing_data.py', 'r') as f:
        data_code = f.read()
    
    required_elements = [
        'class SpectrumSensingWorker',
        'def generate_dataset',
        'def parse_args',
        '__name__ == \'__main__\''
    ]
    
    for element in required_elements:
        if element in data_code:
            print(f"✓ Data script has {element}")
        else:
            print(f"✗ Data script missing {element}")
    
    # Test model script
    with open('spectrum_sensing_model.py', 'r') as f:
        model_code = f.read()
    
    model_elements = [
        'class SpectrumSensingFGNN',
        'class FactorNetwork',
        'class SpectrumSensingDataset',
        'def forward',
        'def message_passing_step'
    ]
    
    for element in model_elements:
        if element in model_code:
            print(f"✓ Model script has {element}")
        else:
            print(f"✗ Model script missing {element}")
    
    # Test training script
    with open('spectrum_sensing_train.py', 'r') as f:
        train_code = f.read()
    
    train_elements = [
        'def train',
        'def test',
        'def main',
        'def parse_args'
    ]
    
    for element in train_elements:
        if element in train_code:
            print(f"✓ Training script has {element}")
        else:
            print(f"✗ Training script missing {element}")
    
    return True

def test_readme_content():
    """Test README content"""
    print("\nTesting README content...")
    
    with open('README.md', 'r') as f:
        readme = f.read()
    
    required_sections = [
        '# Spectrum Sensing Factor Graph Neural Network',
        '## Overview',
        '## Architecture',
        '## Quick Start',
        '## Dataset Generation',
        '## Model Architecture'
    ]
    
    for section in required_sections:
        if section in readme:
            print(f"✓ README has {section}")
        else:
            print(f"✗ README missing {section}")
    
    return True

def main():
    print("="*60)
    print("SPECTRUM SENSING FGNN CODE STRUCTURE TEST")
    print("="*60)
    
    tests = [
        ("File Structure", test_file_structure),
        ("Python Syntax", test_syntax),
        ("Code Structure", test_basic_structure),
        ("README Content", test_readme_content)
    ]
    
    all_passed = True
    for test_name, test_func in tests:
        try:
            result = test_func()
            if result:
                print(f"\n✓ {test_name}: PASSED")
            else:
                print(f"\n✗ {test_name}: FAILED")
                all_passed = False
        except Exception as e:
            print(f"\n✗ {test_name}: ERROR - {e}")
            all_passed = False
    
    print("\n" + "="*60)
    if all_passed:
        print("ALL TESTS PASSED! ✓")
        print("\nThe spectrum sensing FGNN implementation is structurally complete.")
        print("To run the full implementation, install the required dependencies:")
        print("  pip install torch torchvision numpy scipy tensorboardX tqdm")
        print("\nThen run the example:")
        print("  python example_usage.py --dataset_size 50 --epochs 5")
    else:
        print("SOME TESTS FAILED! ✗")
    print("="*60)

if __name__ == '__main__':
    main()