import argparse
import datetime
import logging
import numpy as np
import os
import sys
import torch
import torch.nn as nn
from tensorboardX import SummaryWriter
import time
from tqdm import tqdm
import json

from spectrum_sensing_model import SpectrumSensingFGNN, SpectrumSensingDataset
from spectrum_sensing_utils import str2bool, to_cuda, calculate_metrics


def parse_args():
    parser = argparse.ArgumentParser(description='Spectrum Sensing FGNN Training')

    # Training parameters
    parser.add_argument('--n_epochs', type=int, default=100, help="Number of training epochs")
    parser.add_argument('--batch_size', type=int, default=32, help="Batch size")
    parser.add_argument('--lr', type=float, default=1e-3, help="Learning rate")
    parser.add_argument('--weight_decay', type=float, default=1e-5, help="Weight decay")
    parser.add_argument('--dropout', type=float, default=0.1, help="Dropout rate")
    
    # Model parameters
    parser.add_argument('--num_uavs', type=int, default=10, help="Number of UAVs")
    parser.add_argument('--num_time_steps', type=int, default=20, help="Number of time steps")
    parser.add_argument('--num_freq_bins', type=int, default=16, help="Number of frequency bins")
    parser.add_argument('--hidden_dim', type=int, default=64, help="Hidden dimension")
    parser.add_argument('--num_message_passing_steps', type=int, default=5, help="Number of message passing steps")
    
    # Data parameters
    parser.add_argument('--data_path', type=str, required=True, help="Path to dataset")
    parser.add_argument('--model_path', type=str, default='', help="Path to saved model (for resume/test)")
    parser.add_argument('--model_name', type=str, default='SpectrumSensingFGNN', help="Model name")
    
    # Training options
    parser.add_argument('--train', action='store_true', default=False, help="Train the model")
    parser.add_argument('--test', action='store_true', default=False, help="Test the model")
    parser.add_argument('--use_cuda', type=str2bool, default=True, help="Use CUDA if available")
    parser.add_argument('--num_workers', type=int, default=4, help="Number of data loader workers")
    
    # Logging and saving
    parser.add_argument('--save_every', type=int, default=10, help="Save model every N epochs")
    parser.add_argument('--log_every', type=int, default=10, help="Log every N batches")
    
    args = parser.parse_args()

    if not torch.cuda.is_available():
        args.use_cuda = False

    return args


def worker_init_fn(idx):
    """Initialize worker with different random seed"""
    t = int(time.time() * 1000.0) + idx
    seed = ((t & 0xff000000) >> 24) + ((t & 0x00ff0000) >> 8) + \
        ((t & 0x0000ff00) << 8) + ((t & 0x000000ff) << 24)
    np.random.seed(seed)
    torch.manual_seed(seed)


def train(args, model, writer, model_dir):
    """Training loop"""
    
    # Create datasets
    train_dataset = SpectrumSensingDataset(args.data_path, train=True)
    val_dataset = SpectrumSensingDataset(args.data_path, train=False)
    
    # Create data loaders
    train_loader = torch.utils.data.DataLoader(
        train_dataset,
        batch_size=args.batch_size,
        shuffle=True,
        num_workers=args.num_workers,
        worker_init_fn=worker_init_fn,
        pin_memory=args.use_cuda
    )
    
    val_loader = torch.utils.data.DataLoader(
        val_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        worker_init_fn=worker_init_fn,
        pin_memory=args.use_cuda
    )
    
    # Optimizer and scheduler
    optimizer = torch.optim.Adam(
        model.parameters(), 
        lr=args.lr, 
        weight_decay=args.weight_decay
    )
    
    def lr_scheduler(epoch):
        if epoch < 10:
            return 1.0
        elif epoch < 50:
            return 0.5
        else:
            return 0.1
    
    scheduler = torch.optim.lr_scheduler.LambdaLR(optimizer, lr_lambda=lr_scheduler)
    
    # Loss function
    criterion = nn.BCEWithLogitsLoss()
    
    # Resume training if model path provided
    start_epoch = 0
    best_val_acc = 0.0
    
    if args.model_path and os.path.exists(args.model_path):
        checkpoint = torch.load(args.model_path, map_location='cpu' if not args.use_cuda else None)
        model.load_state_dict(checkpoint['model_state_dict'])
        optimizer.load_state_dict(checkpoint['optimizer_state_dict'])
        scheduler.load_state_dict(checkpoint['scheduler_state_dict'])
        start_epoch = checkpoint['epoch'] + 1
        best_val_acc = checkpoint.get('best_val_acc', 0.0)
        logging.info(f'Resumed training from epoch {start_epoch}')
    
    def get_model_dict(epoch, val_acc):
        return {
            'model_state_dict': model.state_dict(),
            'optimizer_state_dict': optimizer.state_dict(),
            'scheduler_state_dict': scheduler.state_dict(),
            'epoch': epoch,
            'best_val_acc': val_acc,
            'args': args
        }
    
    def get_model_path(epoch):
        return os.path.join(model_dir, f'{args.model_name}_epoch_{epoch}.pt')
    
    logging.info('Training started!')
    global_step = 0
    
    for epoch in range(start_epoch, args.n_epochs):
        model.train()
        
        # Training phase
        train_losses = []
        train_metrics = {'tp': 0, 'tn': 0, 'fp': 0, 'fn': 0}
        
        pbar = tqdm(train_loader, desc=f'Epoch {epoch+1}/{args.n_epochs}')
        for batch_idx, batch in enumerate(pbar):
            optimizer.zero_grad()
            
            # Move to device
            if args.use_cuda:
                batch = {k: v.cuda() if isinstance(v, torch.Tensor) else v 
                        for k, v in batch.items()}
            
            # Forward pass
            predictions = model(
                batch['time_features'],
                batch['freq_features'], 
                batch['spatial_features'],
                batch['uav_positions']
            )
            
            # Compute loss
            labels = batch['labels'].float()
            loss = criterion(predictions, labels)
            
            # Backward pass
            loss.backward()
            torch.nn.utils.clip_grad_norm_(model.parameters(), 1.0)
            optimizer.step()
            
            # Metrics
            train_losses.append(loss.item())
            
            # Convert predictions to binary
            pred_binary = (torch.sigmoid(predictions) > 0.5).long()
            labels_binary = labels.long()
            
            # Update metrics
            batch_metrics = calculate_metrics(pred_binary, labels_binary)
            for key in train_metrics:
                train_metrics[key] += batch_metrics[key]
            
            # Logging
            if (batch_idx + 1) % args.log_every == 0:
                avg_loss = np.mean(train_losses[-args.log_every:])
                pbar.set_postfix({'loss': f'{avg_loss:.4f}'})
                
                writer.add_scalar('train/loss', avg_loss, global_step)
                writer.add_scalar('train/lr', scheduler.get_last_lr()[0], global_step)
            
            global_step += 1
        
        # Calculate training metrics
        train_acc, train_precision, train_recall, train_f1 = compute_metrics_from_counts(train_metrics)
        avg_train_loss = np.mean(train_losses)
        
        # Validation phase
        model.eval()
        val_losses = []
        val_metrics = {'tp': 0, 'tn': 0, 'fp': 0, 'fn': 0}
        
        with torch.no_grad():
            for batch in tqdm(val_loader, desc='Validation'):
                if args.use_cuda:
                    batch = {k: v.cuda() if isinstance(v, torch.Tensor) else v 
                            for k, v in batch.items()}
                
                predictions = model(
                    batch['time_features'],
                    batch['freq_features'],
                    batch['spatial_features'], 
                    batch['uav_positions']
                )
                
                labels = batch['labels'].float()
                loss = criterion(predictions, labels)
                val_losses.append(loss.item())
                
                # Metrics
                pred_binary = (torch.sigmoid(predictions) > 0.5).long()
                labels_binary = labels.long()
                
                batch_metrics = calculate_metrics(pred_binary, labels_binary)
                for key in val_metrics:
                    val_metrics[key] += batch_metrics[key]
        
        # Calculate validation metrics
        val_acc, val_precision, val_recall, val_f1 = compute_metrics_from_counts(val_metrics)
        avg_val_loss = np.mean(val_losses)
        
        # Logging
        logging.info(
            f'Epoch {epoch+1}: '
            f'Train Loss: {avg_train_loss:.4f}, Train Acc: {train_acc:.4f}, '
            f'Val Loss: {avg_val_loss:.4f}, Val Acc: {val_acc:.4f}, '
            f'Val F1: {val_f1:.4f}'
        )
        
        writer.add_scalar('train/epoch_loss', avg_train_loss, epoch)
        writer.add_scalar('train/epoch_acc', train_acc, epoch)
        writer.add_scalar('train/epoch_f1', train_f1, epoch)
        writer.add_scalar('val/epoch_loss', avg_val_loss, epoch)
        writer.add_scalar('val/epoch_acc', val_acc, epoch)
        writer.add_scalar('val/epoch_f1', val_f1, epoch)
        
        # Save model
        if (epoch + 1) % args.save_every == 0:
            torch.save(get_model_dict(epoch, val_acc), get_model_path(epoch))
            logging.info(f'Model saved to {get_model_path(epoch)}')
        
        # Save best model
        if val_acc > best_val_acc:
            best_val_acc = val_acc
            best_model_path = os.path.join(model_dir, f'{args.model_name}_best.pt')
            torch.save(get_model_dict(epoch, val_acc), best_model_path)
            logging.info(f'Best model saved to {best_model_path} with val_acc: {val_acc:.4f}')
        
        scheduler.step()
    
    # Save final model
    final_model_path = os.path.join(model_dir, f'{args.model_name}_final.pt')
    torch.save(get_model_dict(args.n_epochs - 1, best_val_acc), final_model_path)
    logging.info(f'Training completed! Final model saved to {final_model_path}')


def test(args, model):
    """Testing loop"""
    
    # Load test dataset
    test_dataset = SpectrumSensingDataset(args.data_path, train=False)
    test_loader = torch.utils.data.DataLoader(
        test_dataset,
        batch_size=args.batch_size,
        shuffle=False,
        num_workers=args.num_workers,
        worker_init_fn=worker_init_fn
    )
    
    # Load model
    assert args.model_path and os.path.exists(args.model_path), 'Model path required for testing'
    
    checkpoint = torch.load(args.model_path, map_location='cpu' if not args.use_cuda else None)
    model.load_state_dict(checkpoint['model_state_dict'])
    logging.info(f"Loaded model from {args.model_path}")
    
    model.eval()
    test_metrics = {'tp': 0, 'tn': 0, 'fp': 0, 'fn': 0}
    all_predictions = []
    all_labels = []
    
    with torch.no_grad():
        for batch in tqdm(test_loader, desc='Testing'):
            if args.use_cuda:
                batch = {k: v.cuda() if isinstance(v, torch.Tensor) else v 
                        for k, v in batch.items()}
            
            predictions = model(
                batch['time_features'],
                batch['freq_features'],
                batch['spatial_features'],
                batch['uav_positions']
            )
            
            # Convert to probabilities
            pred_probs = torch.sigmoid(predictions)
            pred_binary = (pred_probs > 0.5).long()
            labels_binary = batch['labels'].long()
            
            # Store for analysis
            all_predictions.append(pred_probs.cpu())
            all_labels.append(labels_binary.cpu())
            
            # Update metrics
            batch_metrics = calculate_metrics(pred_binary, labels_binary)
            for key in test_metrics:
                test_metrics[key] += batch_metrics[key]
    
    # Calculate final metrics
    test_acc, test_precision, test_recall, test_f1 = compute_metrics_from_counts(test_metrics)
    
    # Print results
    print("\n" + "="*50)
    print("TEST RESULTS")
    print("="*50)
    print(f"Accuracy: {test_acc:.4f}")
    print(f"Precision: {test_precision:.4f}")
    print(f"Recall: {test_recall:.4f}")
    print(f"F1-Score: {test_f1:.4f}")
    print(f"True Positives: {test_metrics['tp']}")
    print(f"True Negatives: {test_metrics['tn']}")
    print(f"False Positives: {test_metrics['fp']}")
    print(f"False Negatives: {test_metrics['fn']}")
    
    # Save test results
    results = {
        'accuracy': test_acc,
        'precision': test_precision,
        'recall': test_recall,
        'f1_score': test_f1,
        'confusion_matrix': test_metrics
    }
    
    results_path = args.model_path.replace('.pt', '_test_results.json')
    with open(results_path, 'w') as f:
        json.dump(results, f, indent=2)
    
    logging.info(f'Test results saved to {results_path}')


def compute_metrics_from_counts(metrics_dict):
    """Compute accuracy, precision, recall, F1 from TP/TN/FP/FN counts"""
    tp = metrics_dict['tp']
    tn = metrics_dict['tn']
    fp = metrics_dict['fp']
    fn = metrics_dict['fn']
    
    accuracy = (tp + tn) / (tp + tn + fp + fn + 1e-8)
    precision = tp / (tp + fp + 1e-8)
    recall = tp / (tp + fn + 1e-8)
    f1 = 2 * precision * recall / (precision + recall + 1e-8)
    
    return accuracy, precision, recall, f1


def main():
    args = parse_args()
    
    # Create model
    model = SpectrumSensingFGNN(
        num_uavs=args.num_uavs,
        num_time_steps=args.num_time_steps,
        num_freq_bins=args.num_freq_bins,
        hidden_dim=args.hidden_dim,
        num_message_passing_steps=args.num_message_passing_steps,
        dropout=args.dropout
    )
    
    logging.info(f'Model created: {model}')
    
    if args.use_cuda:
        model = model.cuda()
        logging.info('Model moved to CUDA')
    
    if args.train:
        # Setup logging and directories
        timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H:%M:%S")
        subdir = f'train_spectrum_sensing_{args.model_name}_at_{timestamp}'
        
        # Setup logging
        log_dir = './logs/'
        os.makedirs(log_dir, exist_ok=True)
        log_file = os.path.join(log_dir, f'{subdir}.log')
        
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[
                logging.FileHandler(log_file),
                logging.StreamHandler(sys.stdout)
            ]
        )
        
        logging.info(f'Arguments: {args}')
        
        # Setup tensorboard
        tb_dir = f'./tf_logs/{subdir}'
        writer = SummaryWriter(log_dir=tb_dir)
        logging.info(f'Tensorboard logs: {tb_dir}')
        
        # Setup model directory
        model_dir = f'./models_spectrum_sensing/{subdir}'
        os.makedirs(model_dir, exist_ok=True)
        logging.info(f'Model directory: {model_dir}')
        
        # Start training
        train(args, model, writer, model_dir)
        
        writer.close()
        
    elif args.test:
        # Setup logging for testing
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(levelname)s - %(message)s',
            handlers=[logging.StreamHandler(sys.stdout)]
        )
        
        test(args, model)
    
    else:
        print("Please specify --train or --test")


if __name__ == '__main__':
    main()