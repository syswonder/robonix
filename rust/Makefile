# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Makefile
#
# Convenient commands for building and running Robonix components

.PHONY: help build build-cli build-core build-sdk install install-cli install-core source-sdk env clean test check setup-dev fmt
.DEFAULT_GOAL := help

# Default ROS2 distribution (can be overridden)
ROS2_DISTRO ?= humble
ROS2_SETUP ?= /opt/ros/$(ROS2_DISTRO)/setup.bash

# Paths
RUST_DIR := $(shell pwd)
CLI_DIR := $(RUST_DIR)/robonix-cli
CORE_DIR := $(RUST_DIR)/robonix-core
SDK_DIR := $(RUST_DIR)/robonix-sdk
PROVIDER_DIR := $(RUST_DIR)/provider

# Default target
help:
	@echo "Robonix Makefile - Available commands:"
	@echo ""
	@echo "Build commands:"
	@echo "  make build          - Build robonix-cli and robonix-core"
	@echo "  make build-cli      - Build robonix-cli only"
	@echo "  make build-core     - Build robonix-core only"
	@echo "  make build-sdk      - Build robonix-sdk ROS2 interface package"
	@echo ""
	@echo "Install commands:"
	@echo "  make install        - Install robonix-cli and robonix-core binaries via cargo install"
	@echo "  make install-cli    - Install robonix-cli binaries (rbnx, rbnx-daemon) only"
	@echo "  make install-core   - Install robonix-core binary only"
	@echo "  Note: Binaries are installed to ~/.cargo/bin (make sure it's in your PATH)"
	@echo ""
	@echo "Environment commands:"
	@echo "  make source-sdk     - Source robonix-sdk environment (use: eval \$$(make source-sdk))"
	@echo "  make env            - Show environment information"
	@echo ""
	@echo "Development commands:"
	@echo "  make setup-dev      - Setup development environment (link provider directory)"
	@echo "  make fmt            - Format code using cargo fmt"
	@echo "  make check          - Run cargo check on all Rust projects"
	@echo "  make clean          - Clean build artifacts"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make install"
	@echo "  rbnx config --show"
	@echo "  rbnx package list"
	@echo "  eval \$$(make source-sdk)"

# Build commands
build: build-cli build-core
	@echo "✓ All components built successfully"

build-cli:
	@echo "Building robonix-cli..."
	cd $(CLI_DIR) && cargo build --release
	@echo "✓ robonix-cli built successfully"

build-core:
	@echo "Building robonix-core..."
	cd $(CORE_DIR) && cargo build --release
	@echo "✓ robonix-core built successfully"

build-sdk:
	@echo "Building robonix-sdk ROS2 interface package..."
	@if [ ! -f $(ROS2_SETUP) ]; then \
		echo "Error: ROS2 setup file not found at $(ROS2_SETUP)"; \
		echo "Please set ROS2_DISTRO environment variable or ensure ROS2 is installed"; \
		exit 1; \
	fi
	cd $(SDK_DIR) && bash build_ros2.sh
	@echo "✓ robonix-sdk built successfully"

# Alias for backward compatibility
buildsdk: build-sdk

# Install commands
install: install-cli install-core
	@echo "✓ All binaries installed successfully"
	@echo ""
	@echo "Binaries installed via cargo install to ~/.cargo/bin"
	@echo "Make sure ~/.cargo/bin is in your PATH"
	@echo ""
	@echo "You can now run:"
	@echo "  rbnx <command>"
	@echo "  rbnx-daemon <command>"
	@echo "  robonix-core"

install-cli:
	@echo "Installing robonix-cli binaries via cargo install..."
	cd $(CLI_DIR) && cargo install --path . --bin rbnx --bin rbnx-daemon
	@echo "✓ Installed rbnx and rbnx-daemon"

install-core:
	@echo "Installing robonix-core binary via cargo install..."
	cd $(CORE_DIR) && cargo install --path .
	@echo "✓ Installed robonix-core"

# Environment commands
source-sdk:
	@# Output commands to source SDK environment
	@# Usage: eval $$(make source-sdk)
	@SDK_PATH="$(SDK_DIR)"; \
	if [ -f "$$SDK_PATH/install/setup.bash" ]; then \
		echo "source $$SDK_PATH/install/setup.bash"; \
		echo "export ROBONIX_SDK_PATH=$$SDK_PATH"; \
		echo "echo '✓ robonix-sdk environment sourced'"; \
	else \
		echo "echo 'Error: robonix-sdk not built. Run: make build-sdk'"; \
		exit 1; \
	fi

env:
	@echo "Robonix Environment Information:"
	@echo "================================"
	@echo "RUST_DIR: $(RUST_DIR)"
	@echo "CLI_DIR: $(CLI_DIR)"
	@echo "CORE_DIR: $(CORE_DIR)"
	@echo "SDK_DIR: $(SDK_DIR)"
	@echo "PROVIDER_DIR: $(PROVIDER_DIR)"
	@echo ""
	@echo "ROS2 Configuration:"
	@echo "  ROS2_DISTRO: $(ROS2_DISTRO)"
	@echo "  ROS2_SETUP: $(ROS2_SETUP)"
	@if [ -f "$(ROS2_SETUP)" ]; then \
		echo "  ROS2 setup: ✓ Found"; \
	else \
		echo "  ROS2 setup: ✗ Not found"; \
	fi
	@echo ""
	@echo "robonix-sdk Status:"
	@if [ -f "$(SDK_DIR)/install/setup.bash" ]; then \
		echo "  Built: ✓ Yes"; \
		echo "  ROBONIX_SDK_PATH: $(SDK_DIR)"; \
	else \
		echo "  Built: ✗ No (run: make build-sdk)"; \
	fi
	@echo ""
	@echo "Rust Projects:"
	@if [ -f "$(CLI_DIR)/Cargo.toml" ]; then \
		echo "  robonix-cli: ✓ Found"; \
	else \
		echo "  robonix-cli: ✗ Not found"; \
	fi
	@if [ -f "$(CORE_DIR)/Cargo.toml" ]; then \
		echo "  robonix-core: ✓ Found"; \
	else \
		echo "  robonix-core: ✗ Not found"; \
	fi
	@echo ""
	@echo "Development Setup:"
	@if [ -L ~/.robonix/packages ] && [ "$(PROVIDER_DIR)" = "$$(readlink -f ~/.robonix/packages)" ]; then \
		echo "  Provider link: ✓ Configured"; \
	else \
		echo "  Provider link: ✗ Not configured (run: make setup-dev)"; \
	fi

# Development setup
setup-dev:
	@echo "Setting up development environment..."
	@mkdir -p ~/.robonix
	@if [ -L ~/.robonix/packages ]; then \
		echo "Removing existing provider link..."; \
		rm -f ~/.robonix/packages; \
	fi
	@ln -sf "$(PROVIDER_DIR)" ~/.robonix/packages
	@echo "✓ Linked $(PROVIDER_DIR) to ~/.robonix/packages"
	@echo "✓ Development environment setup complete"

# Format code
fmt:
	@echo "Formatting code with cargo fmt..."
	@echo "Formatting robonix-cli..."
	cd $(CLI_DIR) && cargo fmt
	@echo "Formatting robonix-core..."
	cd $(CORE_DIR) && cargo fmt
	@echo "✓ Code formatted"

# Check and test
check:
	@echo "Running cargo check on all Rust projects..."
	@echo "Checking robonix-cli..."
	cd $(CLI_DIR) && cargo check
	@echo "Checking robonix-core..."
	cd $(CORE_DIR) && cargo check
	@echo "✓ All checks passed"

test:
	@echo "Running tests..."
	cd $(CLI_DIR) && cargo test || true
	cd $(CORE_DIR) && cargo test || true
	@echo "✓ Tests completed"

# Clean commands
clean:
	@echo "Cleaning build artifacts..."
	cd $(CLI_DIR) && cargo clean
	cd $(CORE_DIR) && cargo clean
	@echo "✓ Clean complete"

clean-all: clean
	@echo "Cleaning SDK build artifacts..."
	@if [ -d "$(SDK_DIR)/build" ]; then rm -rf $(SDK_DIR)/build; fi
	@if [ -d "$(SDK_DIR)/install" ]; then rm -rf $(SDK_DIR)/install; fi
	@if [ -d "$(SDK_DIR)/log" ]; then rm -rf $(SDK_DIR)/log; fi
	@echo "✓ All artifacts cleaned"
