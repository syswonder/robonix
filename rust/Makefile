# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Makefile — build and install robonix-server, rbnx, ridlc

.PHONY: help build install clean build-ridlc build-cli build-server install-ridlc install-cli install-server
.DEFAULT_GOAL := help

RUST_DIR := $(shell pwd)
RIDLC_DIR := $(RUST_DIR)/ridlc
CLI_DIR   := $(RUST_DIR)/robonix-cli
CORE_DIR  := $(RUST_DIR)/robonix-server

BUILD_MODE ?= debug
CARGO_FLAGS := $(if $(filter release,$(BUILD_MODE)),--release,)

help:
	@echo "Robonix Makefile"
	@echo ""
	@echo "  Build:"
	@echo "    make build         - Build ridlc, rbnx, robonix-server (default: debug)"
	@echo "    make release       - Build in release mode"
	@echo ""
	@echo "  Install:"
	@echo "    make install       - Install to ~/.cargo/bin (ridlc, rbnx, robonix-server)"
	@echo "    Note: robonix-server installs a wrapper that sources the colcon workspace;"
	@echo "    the rust/ directory must remain in place for robonix-server to run."
	@echo ""
	@echo "  Clean:"
	@echo "    make clean         - Remove target dirs and build artifacts"
	@echo ""

build: build-ridlc build-cli build-server
	@echo "Build complete."

release:
	@$(MAKE) BUILD_MODE=release build

build-ridlc:
	@echo "Building ridlc..."
	cd $(RIDLC_DIR) && cargo build $(CARGO_FLAGS)
	@echo "ridlc done."

build-cli:
	@echo "Building robonix-cli (rbnx)..."
	cd $(CLI_DIR) && cargo build $(CARGO_FLAGS)
	@echo "rbnx done."

build-server:
	@echo "Building robonix-server..."
	cd $(CORE_DIR) && cargo build $(CARGO_FLAGS)
	@echo "robonix-server done."

install: install-ridlc install-cli install-server
	@echo "Installed to ~/.cargo/bin: ridlc, rbnx, robonix-server"

install-ridlc:
	cd $(RIDLC_DIR) && cargo install --path . --bin ridlc $(CARGO_FLAGS)

install-cli:
	cd $(CLI_DIR) && cargo install --path . --bin rbnx $(CARGO_FLAGS)

RUST_ABS := $(shell cd $(RUST_DIR) && pwd)
BUILD_SUBDIR := $(if $(filter release,$(BUILD_MODE)),release,debug)

install-server: build-server
	@# robonix-server needs LD_LIBRARY_PATH from colcon workspace; install a wrapper
	@mkdir -p ~/.cargo/bin; \
	printf '%s\n' \
		'#!/usr/bin/env bash' \
		'set -euo pipefail' \
		'RUST_DIR="$(RUST_ABS)"' \
		'export ROBONIX_META_GRPC_ADDR="$${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"' \
		'export ROBONIX_META_GRPC_ENDPOINT="$${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"' \
		'export RUST_LOG="$${RUST_LOG:-robonix_server=info,rclrs=warn}"' \
		'export RMW_IMPLEMENTATION="$${RMW_IMPLEMENTATION:-rmw_zenoh_cpp}"' \
		'set +u; . "$$RUST_DIR/setup_zenoh_rclrs_env" "$$RUST_DIR/robonix-server/Cargo.toml" robonix-server; set -u' \
		'exec "$$RUST_DIR/robonix-server/target/$(BUILD_SUBDIR)/robonix-server" "$$@"' \
		> ~/.cargo/bin/robonix-server; \
	chmod +x ~/.cargo/bin/robonix-server; \
	echo "Installed robonix-server wrapper to ~/.cargo/bin (rust dir: $(RUST_ABS))"

clean:
	@echo "Cleaning ridlc..."
	cd $(RIDLC_DIR) && cargo clean
	@echo "Cleaning robonix-cli..."
	cd $(CLI_DIR) && cargo clean
	@echo "Cleaning robonix-server..."
	cd $(CORE_DIR) && cargo clean
	@echo "Clean complete."
