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

install-server:
	cd $(CORE_DIR) && cargo install --path . $(CARGO_FLAGS)

clean:
	@echo "Cleaning ridlc..."
	cd $(RIDLC_DIR) && cargo clean
	@echo "Cleaning robonix-cli..."
	cd $(CLI_DIR) && cargo clean
	@echo "Cleaning robonix-server..."
	cd $(CORE_DIR) && cargo clean
	@echo "Clean complete."
