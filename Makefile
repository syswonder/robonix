# SPDX-License-Identifier: MulanPSL-2.0
# Robonix top-level Makefile. Orchestrates Cargo workspace build + install.
#
# The Cargo workspace lives at the repo root and references the 4 Rust system
# components (system/{atlas,executor,liaison,pilot}) plus the 2 Rust dev tools
# (tools/{rbnx,codegen}). Python system components (system/scene) and Python
# reference services (services/*) are managed by uv from this same root.

.PHONY: help build release install clean fmt check pyrightconfig \
        build-atlas build-pilot build-executor build-liaison
.DEFAULT_GOAL := help

BUILD_MODE ?= debug
CARGO_FLAGS := $(if $(filter release,$(BUILD_MODE)),--release,)

help:
	@echo "Robonix Makefile"
	@echo ""
	@echo "  Build:"
	@echo "    make build           - Build all Cargo workspace crates (debug)"
	@echo "    make release         - Build all Cargo workspace crates (release)"
	@echo "    make build-atlas     - Install robonix-atlas to ~/.cargo/bin"
	@echo "    make build-pilot     - Install robonix-pilot to ~/.cargo/bin"
	@echo "    make build-executor  - Install robonix-executor to ~/.cargo/bin"
	@echo "    make build-liaison   - Install robonix-liaison to ~/.cargo/bin"
	@echo ""
	@echo "  Install:"
	@echo "    make install         - Install all binaries to ~/.cargo/bin and register this repo via rbnx setup"
	@echo ""
	@echo "  Format & Lint:"
	@echo "    make fmt             - cargo fmt --all"
	@echo "    make check           - cargo fmt --check + clippy -D warnings"
	@echo ""
	@echo "  Clean:"
	@echo "    make clean           - cargo clean"

build:
	cargo build --workspace $(CARGO_FLAGS)

release:
	@$(MAKE) BUILD_MODE=release build

build-atlas:
	cargo install --force --path system/atlas    --bin robonix-atlas    $(CARGO_FLAGS)

build-pilot:
	cargo install --force --path system/pilot    --bin robonix-pilot    $(CARGO_FLAGS)

build-executor:
	cargo install --force --path system/executor --bin robonix-executor $(CARGO_FLAGS)

build-liaison:
	cargo install --force --path system/liaison  --bin robonix-liaison  $(CARGO_FLAGS)

fmt:
	cargo fmt --all

check:
	cargo fmt --all -- --check
	cargo clippy --workspace -- -D warnings

install:
	cargo install --force --path tools/codegen   --bin robonix-codegen  $(CARGO_FLAGS)
	cargo install --force --path system/atlas    --bin robonix-atlas    $(CARGO_FLAGS)
	cargo install --force --path system/pilot    --bin robonix-pilot    $(CARGO_FLAGS)
	cargo install --force --path system/executor --bin robonix-executor $(CARGO_FLAGS)
	cargo install --force --path system/liaison  --bin robonix-liaison  $(CARGO_FLAGS)
	cargo install --force --path tools/rbnx      --bin rbnx             $(CARGO_FLAGS)
	@# Register this clone as the robonix source tree so packages anywhere
	@# on disk can resolve capabilities/IDL via `rbnx path`. Updates
	@# ~/.robonix/config.yaml robonix_source_path = <repo-root>.
	@REPO_ROOT="$$(pwd)"; \
	echo ""; \
	echo "[make install] registering robonix_source_path → $$REPO_ROOT"; \
	"$$HOME/.cargo/bin/rbnx" setup "$$REPO_ROOT" || { \
		echo "[make install] WARNING: 'rbnx setup' failed — run it manually:"; \
		echo "                cd $$REPO_ROOT && rbnx setup"; \
	}

clean:
	cargo clean
