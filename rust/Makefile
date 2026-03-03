# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Makefile
#
# Convenient commands for building and running Robonix components
#
# Tab completion: bash-completion is installed in Docker container
# If tab completion doesn't work, ensure bash-completion is installed and enabled

.PHONY: help build build-cli build-core build-sdk debug release install install-cli install-core source-sdk env clean test test-all test-rust test-python test-cpp test-ros2 benchmark check setup-dev fmt fmt-check ci-rust ci-full test-framework
.DEFAULT_GOAL := help

# Default ROS2 distribution (can be overridden)
ROS2_DISTRO ?= humble
ROS2_SETUP ?= /opt/ros/$(ROS2_DISTRO)/setup.bash

# Build mode: debug (default) or release
BUILD_MODE ?= debug

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
	@echo "  make build          - Build robonix-cli and robonix-core (default: debug mode)"
	@echo "  make debug          - Build in debug mode (default)"
	@echo "  make release        - Build in release mode"
	@echo "  make build-cli      - Build robonix-cli only (uses BUILD_MODE, default: debug)"
	@echo "  make build-core     - Build robonix-core only (uses BUILD_MODE, default: debug)"
	@echo "  make build-sdk      - Build robonix-sdk ROS2 interface package"
	@echo "  Note: Set BUILD_MODE=release to build in release mode, or use 'make release'"
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
	@echo "  make fmt-check      - Check format only (CI; fails if not formatted)"
	@echo "  make check          - Run cargo check on all Rust projects"
	@echo "  make clean          - Clean build artifacts"
	@echo ""
	@echo "CI (single entry points):"
	@echo "  make ci-rust        - Format check + cargo check + build (no ROS2)"
	@echo "  make ci-full        - Build SDK + Rust (requires ROS2 sourced, colcon on PATH)"
	@echo ""
	@echo "Test commands:"
	@echo "  make test           - Run all tests (Rust, Python)"
	@echo "  make test-rust      - Run Rust CLI stress tests"
	@echo "  make test-python    - Run Python stress tests"
	@echo "  make benchmark      - Run benchmark suite with different concurrency levels"
	@echo ""
	@echo "Examples:"
	@echo "  make build"
	@echo "  make install"
	@echo "  rbnx config --show"
	@echo "  rbnx package list"
	@echo "  eval \$$(make source-sdk)"
	@echo ""
	@echo "Tab Completion:"
	@echo "  Tab completion for make targets is enabled automatically in Docker container"
	@echo "  If it doesn't work, ensure bash-completion package is installed"

# Build mode helper: determine cargo build flags
ifeq ($(BUILD_MODE),release)
CARGO_BUILD_FLAGS = --release
BUILD_MODE_DESC = release
else
CARGO_BUILD_FLAGS = 
BUILD_MODE_DESC = debug
endif

# Build commands
build: build-cli build-core
	@echo "✓ All components built successfully ($(BUILD_MODE_DESC) mode)"

debug:
	@$(MAKE) BUILD_MODE=debug build

release:
	@$(MAKE) BUILD_MODE=release build

build-cli:
	@echo "Building robonix-cli ($(BUILD_MODE_DESC) mode)..."
	cd $(CLI_DIR) && cargo build $(CARGO_BUILD_FLAGS)
	@echo "✓ robonix-cli built successfully ($(BUILD_MODE_DESC) mode)"

build-core:
	@echo "Building robonix-core ($(BUILD_MODE_DESC) mode)..."
	cd $(CORE_DIR) && cargo build $(CARGO_BUILD_FLAGS)
	@echo "✓ robonix-core built successfully ($(BUILD_MODE_DESC) mode)"

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

# CI targets (single entry points; extend these when adding packages)
ci-rust: fmt-check check build
	@echo "✓ ci-rust done"

ci-full: build-sdk build
	@echo "✓ ci-full done (requires: ROS2 sourced, colcon on PATH)"

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
	@echo "Build Configuration:"
	@echo "  BUILD_MODE: $(BUILD_MODE)"
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
	@if [ -e ~/.robonix/packages ]; then \
		echo "Removing existing packages path..."; \
		rm -rf ~/.robonix/packages; \
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

# Check formatting only (for CI; fails if not formatted)
fmt-check:
	@echo "Checking code format with cargo fmt -- --check..."
	cd $(CLI_DIR) && cargo fmt -- --check
	cd $(CORE_DIR) && cargo fmt -- --check
	@echo "✓ Format check passed"

# Check and test
check:
	@echo "Running cargo check on all Rust projects..."
	@echo "Checking robonix-cli..."
	cd $(CLI_DIR) && cargo check
	@echo "Checking robonix-core..."
	cd $(CORE_DIR) && cargo check
	@echo "✓ All checks passed"

# Test framework paths
TEST_FRAMEWORK_DIR := $(RUST_DIR)/tools/test_framework
TEST_RUST_DIR := $(TEST_FRAMEWORK_DIR)/rust_tests
TEST_PYTHON_DIR := $(TEST_FRAMEWORK_DIR)/python_tests
TEST_CPP_DIR := $(TEST_FRAMEWORK_DIR)/cpp_tests
TEST_ROS2_DIR := $(TEST_FRAMEWORK_DIR)/ros2_tests

# Test parameters (can be overridden)
CONCURRENCY ?= 3
REQUESTS ?= 1000
RATE ?= 100
DURATION ?= 0

# Test commands
test: test-all

test-all: test-rust test-python
	@echo "✓ All tests completed"

test-rust:
	@echo "Running Rust CLI stress tests (concurrency=$(CONCURRENCY), requests=$(REQUESTS))..."
	@if [ -f "$(TEST_FRAMEWORK_DIR)/test.sh" ]; then \
		bash $(TEST_FRAMEWORK_DIR)/test.sh rust rustdds $(CONCURRENCY) $(REQUESTS) $(RATE) $(DURATION); \
	else \
		echo "Test framework not found. Run 'make test-framework' first."; \
		exit 1; \
	fi

test-python:
	@echo "Running Python stress tests (concurrency=$(CONCURRENCY), requests=$(REQUESTS))..."
	@if [ -f "$(TEST_FRAMEWORK_DIR)/test.sh" ]; then \
		bash $(TEST_FRAMEWORK_DIR)/test.sh python rustdds $(CONCURRENCY) $(REQUESTS) $(RATE) $(DURATION); \
	else \
		echo "Test framework not found. Run 'make test-framework' first."; \
		exit 1; \
	fi


benchmark:
	@echo "Running benchmark suite with different concurrency levels..."
	@if [ -f "$(TEST_FRAMEWORK_DIR)/benchmark.sh" ]; then \
		bash $(TEST_FRAMEWORK_DIR)/benchmark.sh; \
	else \
		echo "Benchmark script not found."; \
		exit 1; \
	fi

test-framework:
	@echo "Setting up test framework..."
	@echo "✓ Test framework ready"
	@echo "Run 'make test' to execute all tests"

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
