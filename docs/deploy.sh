#!/bin/bash

# robonix Documentation Deployment Script
# This script copies built HTML documentation to the remote server

set -e  # Exit on any error

# Configuration
BUILD_DIR="build/html"  # HTML documentation directory
REMOTE_USER="root"  # Server username
REMOTE_HOST="47.94.74.133"  # Your server IP address
REMOTE_PATH="/www/wwwroot/docs.oscommunity.cn"
SSH_CONTROL_PATH="/tmp/ssh_deploy_$$"  # SSH connection multiplexing control path

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if build directory exists
check_build_dir() {
    if [ ! -d "$BUILD_DIR" ]; then
        print_error "Build directory '$BUILD_DIR' not found!"
        print_warning "Please run the documentation build process first."
        exit 1
    fi
    
    if [ ! "$(ls -A $BUILD_DIR)" ]; then
        print_error "Build directory '$BUILD_DIR' is empty!"
        print_warning "Please run the documentation build process first."
        exit 1
    fi
    
    print_success "Build directory found and contains files."
}

# Setup SSH connection multiplexing
setup_ssh_connection() {
    print_status "Setting up SSH connection to $REMOTE_USER@$REMOTE_HOST..."
    print_warning "Please enter the root password (you'll only need to enter it once):"
    
    # Create SSH master connection with multiplexing
    ssh -M -S "$SSH_CONTROL_PATH" -f -N -o ConnectTimeout=10 "$REMOTE_USER@$REMOTE_HOST"
    
    if [ $? -eq 0 ]; then
        print_success "SSH connection established and ready for reuse."
    else
        print_error "Failed to establish SSH connection to $REMOTE_USER@$REMOTE_HOST"
        print_warning "Please check:"
        echo "  - Server IP/hostname: $REMOTE_HOST"
        echo "  - Username: $REMOTE_USER"
        echo "  - Password correctness"
        echo "  - Network connectivity"
        exit 1
    fi
}

# Function to run SSH commands using the multiplexed connection
ssh_run() {
    ssh -S "$SSH_CONTROL_PATH" "$REMOTE_USER@$REMOTE_HOST" "$@"
}

# Function to run SCP using the multiplexed connection
scp_run() {
    scp -o "ControlPath=$SSH_CONTROL_PATH" "$@"
}

# Function to run rsync using the multiplexed connection
rsync_run() {
    rsync -e "ssh -S $SSH_CONTROL_PATH" "$@"
}

# Cleanup SSH connection
cleanup_ssh_connection() {
    if [ -S "$SSH_CONTROL_PATH" ]; then
        print_status "Closing SSH connection..."
        ssh -S "$SSH_CONTROL_PATH" -O exit "$REMOTE_USER@$REMOTE_HOST" 2>/dev/null || true
        rm -f "$SSH_CONTROL_PATH"
    fi
}

# Create remote directory if it doesn't exist
create_remote_dir() {
    print_status "Ensuring remote directory exists..."
    ssh_run "mkdir -p $REMOTE_PATH"
    print_success "Remote directory ready."
}

# Backup existing documentation (optional)
backup_existing_docs() {
    print_status "Creating backup of existing documentation..."
    BACKUP_NAME="docs_backup_$(date +%Y%m%d_%H%M%S)"
    ssh_run "
        if [ -d '$REMOTE_PATH' ] && [ \"\$(ls -A $REMOTE_PATH 2>/dev/null)\" ]; then
            cp -r $REMOTE_PATH ${REMOTE_PATH}_$BACKUP_NAME
            echo 'Backup created: ${REMOTE_PATH}_$BACKUP_NAME'
        else
            echo 'No existing documentation to backup.'
        fi
    "
}

# Deploy documentation
deploy_docs() {
    print_status "Deploying documentation to $REMOTE_HOST:$REMOTE_PATH..."
    
    # Show what will be deployed
    print_status "Files to be deployed:"
    find "$BUILD_DIR" -type f | head -10
    if [ $(find "$BUILD_DIR" -type f | wc -l) -gt 10 ]; then
        print_status "... and $(( $(find "$BUILD_DIR" -type f | wc -l) - 10 )) more files"
    fi
    
    # Check if rsync is available on both local and remote systems
    if command -v rsync >/dev/null 2>&1; then
        print_status "Checking if rsync is available on remote server..."
        if ssh_run "command -v rsync >/dev/null 2>&1"; then
            print_status "Using rsync for efficient transfer..."
            print_status "Copying all files from $BUILD_DIR/ to $REMOTE_PATH/"
            rsync_run -avz --progress --delete \
                "$BUILD_DIR/" \
                "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/"
        else
            print_warning "rsync not found on remote server, falling back to scp..."
            print_status "Using scp for file transfer..."
            print_status "Copying all files from $BUILD_DIR/* to $REMOTE_PATH/"
            scp_run -r "$BUILD_DIR"/* "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/"
        fi
    else
        print_status "Using scp for file transfer..."
        print_status "Copying all files from $BUILD_DIR/* to $REMOTE_PATH/"
        scp_run -r "$BUILD_DIR"/* "$REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH/"
    fi
    
    # Verify deployment
    print_status "Verifying deployment..."
    REMOTE_FILE_COUNT=$(ssh_run "find $REMOTE_PATH -type f | wc -l")
    LOCAL_FILE_COUNT=$(find "$BUILD_DIR" -type f | wc -l)
    print_status "Local files: $LOCAL_FILE_COUNT, Remote files: $REMOTE_FILE_COUNT"
    
    if [ "$REMOTE_FILE_COUNT" -eq "$LOCAL_FILE_COUNT" ]; then
        print_success "All files deployed successfully!"
    else
        print_warning "File count mismatch. Local: $LOCAL_FILE_COUNT, Remote: $REMOTE_FILE_COUNT"
        print_status "This might be normal if there are hidden files or different file types."
    fi
    
    print_success "Documentation deployed successfully!"
}

# Set proper permissions on remote server
set_permissions() {
    print_status "Setting proper permissions on remote server..."
    ssh_run "
        # Set ownership (ignore errors for system files)
        chown -R www-data:www-data $REMOTE_PATH 2>/dev/null || chown -R nginx:nginx $REMOTE_PATH 2>/dev/null || true
        
        # Set permissions, but skip files with special attributes like .user.ini
        find $REMOTE_PATH -type f -name '*.user.ini' -exec echo 'Skipping protected file: {}' \; 2>/dev/null || true
        find $REMOTE_PATH -type f ! -name '*.user.ini' -exec chmod 644 {} \; 2>/dev/null || true
        find $REMOTE_PATH -type d -exec chmod 755 {} \; 2>/dev/null || true
        
        # Try to set general permissions, but don't fail if some files can't be changed
        chmod -R 755 $REMOTE_PATH 2>/dev/null || {
            echo 'Some files could not have permissions changed (this is normal for protected files like .user.ini)'
        }
    "
    print_success "Permissions set (protected files skipped)."
}

# Main deployment function
main() {
    print_status "Starting robonix documentation deployment..."
    echo "=================================="
    
    # Setup trap to cleanup SSH connection on exit
    trap cleanup_ssh_connection EXIT
    
    # Pre-deployment checks
    check_build_dir
    setup_ssh_connection
    
    # Create remote directory
    create_remote_dir
    
    # Optional: Create backup
    read -p "Do you want to backup existing documentation? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        backup_existing_docs
    fi
    
    # Deploy
    deploy_docs
    
    # Set permissions
    set_permissions
    
    echo "=================================="
    print_success "Deployment completed successfully!"
    print_status "Documentation is now available at: http://docs.oscommunity.cn"
    
    # Cleanup SSH connection
    cleanup_ssh_connection
}

# Help function
show_help() {
    echo "robonix Documentation Deployment Script"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "OPTIONS:"
    echo "  -h, --help     Show this help message"
    echo "  --no-backup    Skip backup creation"
    echo "  --dry-run      Show what would be deployed without actually doing it"
    echo ""
    echo "Configuration:"
    echo "  BUILD_DIR:     $BUILD_DIR"
    echo "  REMOTE_USER:   $REMOTE_USER"
    echo "  REMOTE_HOST:   $REMOTE_HOST"
    echo "  REMOTE_PATH:   $REMOTE_PATH"
    echo ""
    echo "Before running:"
    echo "  1. Build your documentation first"
    echo "  2. Ensure you have the root password for the server"
    echo "  3. Make sure SSH password authentication is enabled on the server"
}

# Parse command line arguments
case "${1:-}" in
    -h|--help)
        show_help
        exit 0
        ;;
    --dry-run)
        print_status "DRY RUN: Would deploy from $BUILD_DIR to $REMOTE_USER@$REMOTE_HOST:$REMOTE_PATH"
        check_build_dir
        print_status "Files to be deployed:"
        find "$BUILD_DIR" -type f | head -20
        if [ $(find "$BUILD_DIR" -type f | wc -l) -gt 20 ]; then
            print_status "... and $(( $(find "$BUILD_DIR" -type f | wc -l) - 20 )) more files"
        fi
        exit 0
        ;;
    --no-backup)
        print_status "Backup creation will be skipped."
        # Setup trap to cleanup SSH connection on exit
        trap cleanup_ssh_connection EXIT
        # Run main without backup prompt
        check_build_dir
        setup_ssh_connection
        create_remote_dir
        deploy_docs
        set_permissions
        print_success "Deployment completed successfully!"
        cleanup_ssh_connection
        exit 0
        ;;
    "")
        # No arguments, run normally
        main
        ;;
    *)
        print_error "Unknown option: $1"
        show_help
        exit 1
        ;;
esac
