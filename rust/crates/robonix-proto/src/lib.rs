// SPDX-License-Identifier: MulanPSL-2.0
//
// Robonix Atlas gRPC stubs (tonic-generated, no business logic).
// Replaces the retired `robonix-sdk` wrapper. Consumers build their own
// thin client wrappers as needed using the types re-exported below.

pub mod pb {
    tonic::include_proto!("robonix.atlas");
}

pub use pb::Transport;
pub use pb::atlas_client::AtlasClient;
