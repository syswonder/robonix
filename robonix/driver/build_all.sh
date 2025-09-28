#!/bin/bash

# Build script for all driver and capability packages
# This script traverses all directories and builds ROS2 packages

set -e  # Exit on any error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
WHITE_BOLD='\033[1;37m'
NC='\033[0m' # No Color

# Function to print colored output
print_info() {
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

# Function to build a directory
build_directory() {
    local dir_path="$1"
    local dir_name=$(basename "$dir_path")
    
    print_info "Processing directory: ${WHITE_BOLD}$dir_name${NC}"
    
    # Check if build.sh exists
    if [ -f "$dir_path/build.sh" ]; then
        print_info "Found build.sh in ${WHITE_BOLD}$dir_name${NC}, executing it..."
        cd "$dir_path"
        chmod +x build.sh
        if ./build.sh; then
            print_success "build.sh completed successfully for ${WHITE_BOLD}$dir_name${NC}"
        else
            print_error "build.sh failed for ${WHITE_BOLD}$dir_name${NC}"
            return 1
        fi
        cd - > /dev/null
        return 0
    fi
    
    # Check if there are package.xml files (ROS2 packages)
    if find "$dir_path" -name "package.xml" -type f | grep -q .; then
        print_info "Found ROS2 packages in ${WHITE_BOLD}$dir_name${NC}, running colcon build..."
        cd "$dir_path"
        if colcon build --symlink-install; then
            print_success "colcon build completed successfully for ${WHITE_BOLD}$dir_name${NC}"
        else
            print_error "colcon build failed for ${WHITE_BOLD}$dir_name${NC}"
            return 1
        fi
        cd - > /dev/null
        return 0
    fi
    
    # No build requirements found
    print_warning "No build requirements found in ${WHITE_BOLD}$dir_name${NC}, skipping..."
    return 0
}

# Main execution
main() {
    print_info "Starting build process for all driver and capability packages..."
    
    # Get the script directory
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
    ROBONIX_DIR="$(dirname "$SCRIPT_DIR")"
    DRIVER_DIR="$ROBONIX_DIR/driver"
    CAPABILITY_DIR="$ROBONIX_DIR/capability"
    
    print_info "Driver directory: $DRIVER_DIR"
    print_info "Capability directory: $CAPABILITY_DIR"
    
    # Arrays to track results
    SUCCESS_DIRS=()
    FAILED_DIRS=()
    SKIPPED_DIRS=()
    
    # Process driver directories
    print_info "Processing driver directories..."
    for dir in "$DRIVER_DIR"/*; do
        if [ -d "$dir" ]; then
            if build_directory "$dir"; then
                SUCCESS_DIRS+=("$(basename "$dir")")
            else
                FAILED_DIRS+=("$(basename "$dir")")
            fi
        fi
    done
    
    # Process capability directories
    print_info "Processing capability directories..."
    for dir in "$CAPABILITY_DIR"/*; do
        if [ -d "$dir" ]; then
            if build_directory "$dir"; then
                SUCCESS_DIRS+=("$(basename "$dir")")
            else
                FAILED_DIRS+=("$(basename "$dir")")
            fi
        fi
    done
    
    # Print summary
    echo
    print_info "Build Summary:"
    print_success "Successfully built: ${SUCCESS_DIRS[*]}"
    
    if [ ${#FAILED_DIRS[@]} -gt 0 ]; then
        print_error "Failed to build: ${FAILED_DIRS[*]}"
    fi
    
    if [ ${#SKIPPED_DIRS[@]} -gt 0 ]; then
        print_warning "Skipped: ${SKIPPED_DIRS[*]}"
    fi
    
    # Exit with error code if any builds failed
    if [ ${#FAILED_DIRS[@]} -gt 0 ]; then
        print_error "Some builds failed. Please check the output above."
        exit 1
    else
        print_success "All builds completed successfully!"
        exit 0
    fi
}

# Run main function
main "$@"
