# SPDX-License-Identifier: MulanPSL-2.0
# Robonix Makefile — workspace build commands

.PHONY: help build release install clean pyrightconfig fmt check \
        build-atlas build-pilot build-executor build-liaison
.DEFAULT_GOAL := help

RUST_DIR := $(shell pwd)
BUILD_MODE ?= debug
CARGO_FLAGS := $(if $(filter release,$(BUILD_MODE)),--release,)

help:
	@echo "Robonix Makefile (Cargo workspace)"
	@echo ""
	@echo "  Build:"
	@echo "    make build           - Build all workspace crates (debug)"
	@echo "    make release         - Build all workspace crates (release)"
	@echo "    make build-atlas     - Build robonix-atlas only"
	@echo "    make build-pilot     - Build robonix-pilot only"
	@echo "    make build-executor  - Build robonix-executor only"
	@echo "    make build-liaison   - Build robonix-liaison only"
	@echo ""
	@echo "  Install:"
	@echo "    make install         - Install binaries to ~/.cargo/bin (proto codegen runs in each crate's build.rs)"
	@echo ""
	@echo "  Format & Lint:"
	@echo "    make fmt             - Format all Rust code"
	@echo "    make check           - Check formatting + clippy warnings"
	@echo ""
	@echo "  Clean:"
	@echo "    make clean           - Remove build artifacts"
	@echo ""
	@echo "  IDE:"
	@echo "    make pyrightconfig   - Regenerate pyrightconfig.json"
	@echo ""

build:
	cargo build --workspace $(CARGO_FLAGS)

release:
	@$(MAKE) BUILD_MODE=release build

build-atlas:
	cargo build -p robonix-atlas $(CARGO_FLAGS)

build-pilot:
	cargo build -p robonix-pilot $(CARGO_FLAGS)

build-executor:
	cargo build -p robonix-executor $(CARGO_FLAGS)

build-liaison:
	cargo build -p robonix-liaison $(CARGO_FLAGS)

fmt:
	cargo fmt --all

check:
	cargo fmt --all -- --check
	cargo clippy --workspace -- -D warnings

install: install-atlas
	cargo install --path crates/robonix-codegen  --bin robonix-codegen  $(CARGO_FLAGS)
	cargo install --path crates/robonix-pilot    --bin robonix-pilot    $(CARGO_FLAGS)
	cargo install --path crates/robonix-executor --bin robonix-executor $(CARGO_FLAGS)
	@# rbnx-cli + robonix-liaison are still ported off the deleted
	@# robonix-sdk crate; uncomment once they migrate to robonix-atlas::client.
	@# cargo install --path crates/robonix-cli      --bin rbnx              $(CARGO_FLAGS)
	@# cargo install --path crates/robonix-liaison  --bin robonix-liaison   $(CARGO_FLAGS)
	@# Register this clone as the robonix source tree so packages anywhere
	@# on disk can resolve contracts/IDL via `rbnx path`. This updates
	@# ~/.robonix/config.yaml robonix_source_path = <repo-root> (the parent of rust/).
	@REPO_ROOT="$$(cd "$(RUST_ABS)/.." && pwd)"; \
	echo ""; \
	echo "[make install] registering robonix_source_path → $$REPO_ROOT"; \
	"$$HOME/.cargo/bin/rbnx" setup "$$REPO_ROOT" || { \
		echo "[make install] WARNING: automatic 'rbnx setup' failed — run it manually:"; \
		echo "                cd $$REPO_ROOT && rbnx setup"; \
	}

RUST_ABS := $(shell cd $(RUST_DIR) && pwd)
BUILD_SUBDIR := $(if $(filter release,$(BUILD_MODE)),release,debug)

install-atlas: build-atlas
	@mkdir -p ~/.cargo/bin; \
	printf '%s\n' \
		'#!/usr/bin/env bash' \
		'set -euo pipefail' \
		'RUST_DIR="$(RUST_ABS)"' \
		'export ROBONIX_META_GRPC_ADDR="$${ROBONIX_META_GRPC_ADDR:-0.0.0.0:50051}"' \
		'export ROBONIX_META_GRPC_ENDPOINT="$${ROBONIX_META_GRPC_ENDPOINT:-127.0.0.1:50051}"' \
		'export RUST_LOG="$${RUST_LOG:-robonix_atlas=info}"' \
		'exec "$$RUST_DIR/target/$(BUILD_SUBDIR)/robonix-atlas" "$$@"' \
		> ~/.cargo/bin/robonix-atlas; \
	chmod +x ~/.cargo/bin/robonix-atlas; \
	echo "Installed robonix-atlas wrapper to ~/.cargo/bin"

pyrightconfig:
	@cd $(RUST_DIR)/.. && python3 rust/tools/gen_pyrightconfig.py

clean:
	cargo clean
