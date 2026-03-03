// SPDX-License-Identifier: MulanPSL-2.0
// Speech Module
//
// Text-to-Speech and Speech-to-Text services using Aliyun Intelligent Speech

pub mod stt;
pub mod tts;

pub use stt::SttService;
pub use tts::TtsService;
