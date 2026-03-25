# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Makefile — workspace build commands

.PHONY: help build release install clean pyrightconfig fmt check
.DEFAULT_GOAL := help

RUST_DIR := $(shell pwd)
BUILD_MODE ?= debug
CARGO_FLAGS := $(if $(filter release,$(BUILD_MODE)),--release,)

help:
	@echo "Robonix Makefile (Cargo workspace)"
	@echo ""
	@echo "  Build:"
	@echo "    make build         - Build all workspace crates (debug)"
	@echo "    make release       - Build all workspace crates (release)"
	@echo "    make build-server  - Build robonix-server only"
	@echo ""
	@echo "  Install:"
	@echo "    make install       - Install binaries to ~/.cargo/bin"
	@echo ""
	@echo "  Format & Lint:"
	@echo "    make fmt           - Format all Rust code"
	@echo "    make check         - Check formatting + clippy warnings"
	@echo ""
	@echo "  Clean:"
	@echo "    make clean         - Remove build artifacts"
	@echo ""
	@echo "  IDE:"
	@echo "    make pyrightconfig - Regenerate pyrightconfig.json"
	@echo ""

build:
	cargo build --workspace $(CARGO_FLAGS)

release:
	@$(MAKE) BUILD_MODE=release build

build-server:
	cargo build -p robonix-server $(CARGO_FLAGS)

fmt:
	cargo fmt --all

check:
	cargo fmt --all -- --check
	cargo clippy --workspace -- -D warnings

install: install-server
	cargo install --path crates/ridlc --bin ridlc $(CARGO_FLAGS)
	cargo install --path crates/robonix-cli --bin rbnx $(CARGO_FLAGS)
	cargo install --path crates/robonix-agent --bin robonix-agent $(CARGO_FLAGS)

RUST_ABS := $(shell cd $(RUST_DIR) && pwd)
BUILD_SUBDIR := $(if $(filter release,$(BUILD_MODE)),release,debug)

install-server: build-server
	@mkdir -p ~/.cargo/bin; \
	printf '%s\n' \
		'#!/usr/bin/env bash' \
		'set -euo pipefail' \
		'RUST_DIR="$(RUST_ABS)"' \
		'export ROBONIX_META_GRPC_ADDR="$${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"' \
		'export ROBONIX_META_GRPC_ENDPOINT="$${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"' \
		'export RUST_LOG="$${RUST_LOG:-robonix_server=info}"' \
		'exec "$$RUST_DIR/target/$(BUILD_SUBDIR)/robonix-server" "$$@"' \
		> ~/.cargo/bin/robonix-server; \
	chmod +x ~/.cargo/bin/robonix-server; \
	echo "Installed robonix-server wrapper to ~/.cargo/bin"

pyrightconfig:
	@cd $(RUST_DIR)/.. && python3 rust/tools/gen_pyrightconfig.py

clean:
	cargo clean
