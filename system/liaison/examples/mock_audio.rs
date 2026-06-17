// SPDX-License-Identifier: MulanPSL-2.0
// examples/mock_audio.rs — mock RobonixPrimitiveAudioMic + RobonixPrimitiveAudioSpeaker.
//
// Replaces real audio hardware with WAV file I/O:
//
//   RobonixPrimitiveAudioMic:
//     Reads PCM from a WAV file (env `MOCK_WAV_INPUT`) or generates 5s of
//     silence (16 kHz mono s16le). Streams AudioChunks in ~100 ms slices.
//
//   RobonixPrimitiveAudioSpeaker:
//     Receives AudioChunks and writes the accumulated audio to disk
//     (env `MOCK_WAV_OUTPUT`, default `/tmp/robonix_speaker_output.wav`).
//
// Both services register in Atlas under one capability so Liaison discovers
// them through `query_capabilities("", "robonix/primitive/audio/mic", …)` and
// `query_capabilities("", "robonix/primitive/audio/speaker", …)`.

use anyhow::Result;
use robonix_atlas::client::{self as atlas_client, AtlasClient};
use robonix_atlas::pb as atlas_pb;
use robonix_liaison::pb::audio::AudioChunk;
use robonix_liaison::pb::contracts::{
    robonix_primitive_audio_mic_server::{
        RobonixPrimitiveAudioMic, RobonixPrimitiveAudioMicServer,
    },
    robonix_primitive_audio_speaker_server::{
        RobonixPrimitiveAudioSpeaker, RobonixPrimitiveAudioSpeakerServer,
    },
};
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::mpsc;
use tokio_stream::wrappers::ReceiverStream;
use tonic::{Request, Response, Status, Streaming};

const CAPABILITY_ID: &str = "com.robonix.demo.mock_audio";
const NAMESPACE: &str = "robonix/primitive/audio";
const SAMPLE_RATE: u32 = 16_000;
const CHANNELS: u16 = 1;
const BITS_PER_SAMPLE: u16 = 16;
const BYTES_PER_SAMPLE: usize = 2;
const CHUNK_DURATION_MS: u64 = 100;
const CHUNK_SAMPLES: usize = (SAMPLE_RATE as usize) * (CHUNK_DURATION_MS as usize) / 1000;
const CHUNK_BYTES: usize = CHUNK_SAMPLES * BYTES_PER_SAMPLE;

fn read_wav_pcm(path: &str) -> Result<Vec<u8>> {
    let data = std::fs::read(path)?;
    if data.len() < 44 {
        anyhow::bail!("WAV file too small ({} bytes)", data.len());
    }
    if &data[0..4] != b"RIFF" || &data[8..12] != b"WAVE" {
        anyhow::bail!("not a valid RIFF/WAVE file");
    }
    let mut pos = 12;
    while pos + 8 <= data.len() {
        let chunk_id = &data[pos..pos + 4];
        let chunk_size = u32::from_le_bytes(data[pos + 4..pos + 8].try_into()?) as usize;
        if chunk_id == b"data" {
            let start = pos + 8;
            let end = (start + chunk_size).min(data.len());
            return Ok(data[start..end].to_vec());
        }
        pos += 8 + chunk_size;
        if pos % 2 != 0 {
            pos += 1;
        }
    }
    anyhow::bail!("no 'data' chunk found in WAV file")
}

fn write_wav(path: &str, pcm: &[u8]) -> Result<()> {
    let data_size = pcm.len() as u32;
    let file_size = 36 + data_size;
    let byte_rate = SAMPLE_RATE * (CHANNELS as u32) * (BITS_PER_SAMPLE as u32) / 8;
    let block_align = CHANNELS * BITS_PER_SAMPLE / 8;

    let mut buf = Vec::with_capacity(44 + pcm.len());
    buf.extend_from_slice(b"RIFF");
    buf.extend_from_slice(&file_size.to_le_bytes());
    buf.extend_from_slice(b"WAVE");
    buf.extend_from_slice(b"fmt ");
    buf.extend_from_slice(&16u32.to_le_bytes());
    buf.extend_from_slice(&1u16.to_le_bytes());
    buf.extend_from_slice(&CHANNELS.to_le_bytes());
    buf.extend_from_slice(&SAMPLE_RATE.to_le_bytes());
    buf.extend_from_slice(&byte_rate.to_le_bytes());
    buf.extend_from_slice(&block_align.to_le_bytes());
    buf.extend_from_slice(&BITS_PER_SAMPLE.to_le_bytes());
    buf.extend_from_slice(b"data");
    buf.extend_from_slice(&data_size.to_le_bytes());
    buf.extend_from_slice(pcm);
    std::fs::write(path, &buf)?;
    Ok(())
}

fn generate_silence(seconds: u32) -> Vec<u8> {
    vec![0u8; (SAMPLE_RATE as usize) * BYTES_PER_SAMPLE * (seconds as usize)]
}

struct MockMic {
    pcm_data: Arc<Vec<u8>>,
}

#[tonic::async_trait]
impl RobonixPrimitiveAudioMic for MockMic {
    type MicStream = ReceiverStream<Result<AudioChunk, Status>>;

    async fn mic(&self, _request: Request<()>) -> Result<Response<Self::MicStream>, Status> {
        let pcm = Arc::clone(&self.pcm_data);
        let (tx, rx) = mpsc::channel(64);

        tokio::spawn(async move {
            log::info!(
                "[mock-mic] streaming {} bytes ({:.1}s)",
                pcm.len(),
                pcm.len() as f32 / (SAMPLE_RATE as f32 * BYTES_PER_SAMPLE as f32),
            );
            for (i, chunk) in pcm.chunks(CHUNK_BYTES).enumerate() {
                let ac = AudioChunk {
                    timestamp_ns: now_ns(),
                    data: chunk.to_vec(),
                    sequence: i as u32,
                    duration_s: chunk.len() as f32 / (SAMPLE_RATE as f32 * BYTES_PER_SAMPLE as f32),
                };
                if tx.send(Ok(ac)).await.is_err() {
                    break;
                }
                tokio::time::sleep(Duration::from_millis(CHUNK_DURATION_MS)).await;
            }
            log::info!("[mock-mic] stream complete");
        });

        Ok(Response::new(ReceiverStream::new(rx)))
    }
}

struct MockSpeaker {
    output_path: String,
}

#[tonic::async_trait]
impl RobonixPrimitiveAudioSpeaker for MockSpeaker {
    async fn speaker(
        &self,
        request: Request<Streaming<AudioChunk>>,
    ) -> Result<Response<()>, Status> {
        let mut stream = request.into_inner();
        let mut pcm_buf: Vec<u8> = Vec::new();

        while let Some(chunk) = stream
            .message()
            .await
            .map_err(|e| Status::internal(e.to_string()))?
        {
            pcm_buf.extend_from_slice(&chunk.data);
        }

        let is_mp3 = pcm_buf.len() >= 3
            && (pcm_buf[0] == 0xFF && (pcm_buf[1] & 0xE0) == 0xE0 || &pcm_buf[0..3] == b"ID3");

        let out_path = if is_mp3 {
            let p = self.output_path.replace(".wav", ".mp3");
            log::info!(
                "[mock-speaker] received {} bytes MP3, writing to {p}",
                pcm_buf.len()
            );
            std::fs::write(&p, &pcm_buf)
                .map_err(|e| Status::internal(format!("write mp3: {e}")))?;
            p
        } else {
            log::info!(
                "[mock-speaker] received {} bytes PCM, writing WAV to {}",
                pcm_buf.len(),
                self.output_path
            );
            write_wav(&self.output_path, &pcm_buf)
                .map_err(|e| Status::internal(format!("write wav: {e}")))?;
            self.output_path.clone()
        };
        log::info!("[mock-speaker] saved audio to {out_path}");

        Ok(Response::new(()))
    }
}

#[tokio::main]
async fn main() -> Result<()> {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let atlas = std::env::var("ROBONIX_ATLAS").unwrap_or_else(|_| "127.0.0.1:50051".to_string());
    let atlas_http = if atlas.starts_with("http") {
        atlas
    } else {
        format!("http://{atlas}")
    };

    let port: u16 = std::env::var("MOCK_AUDIO_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(50091);
    let listen: std::net::SocketAddr = format!("0.0.0.0:{port}").parse()?;
    let advertised = format!("127.0.0.1:{port}");

    let pcm_data = if let Ok(wav_path) = std::env::var("MOCK_WAV_INPUT") {
        log::info!("[mock-audio] reading WAV from {wav_path}");
        Arc::new(read_wav_pcm(&wav_path)?)
    } else {
        log::info!("[mock-audio] no MOCK_WAV_INPUT, generating 5s silence");
        Arc::new(generate_silence(5))
    };
    let output_path = std::env::var("MOCK_WAV_OUTPUT")
        .unwrap_or_else(|_| "/tmp/robonix_speaker_output.wav".to_string());

    log::info!("[mock-audio] PCM buffer: {} bytes", pcm_data.len());
    log::info!("[mock-audio] speaker output: {output_path}");

    log::info!("[mock-audio] connecting to Atlas at {atlas_http}");
    let mut atlas =
        AtlasClient::connect_with_retry(&atlas_http, 10, Duration::from_secs(2)).await?;
    atlas.register_service(CAPABILITY_ID, NAMESPACE, "").await?;
    atlas
        .declare_capability(
            CAPABILITY_ID,
            "robonix/primitive/audio/mic",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/primitive/audio/mic.v1.toml",
                "robonix.contracts.RobonixPrimitiveAudioMic",
                "/robonix.contracts.RobonixPrimitiveAudioMic/Stream",
            ),
        )
        .await?;
    atlas
        .declare_capability(
            CAPABILITY_ID,
            "robonix/primitive/audio/speaker",
            atlas_pb::Transport::Grpc,
            &advertised,
            atlas_client::grpc_params(
                "capabilities/primitive/audio/speaker.v1.toml",
                "robonix.contracts.RobonixPrimitiveAudioSpeaker",
                "/robonix.contracts.RobonixPrimitiveAudioSpeaker/Stream",
            ),
        )
        .await?;
    log::info!("[mock-audio] registered '{CAPABILITY_ID}', mic+speaker on :{port}");
    eprintln!("mock-audio ready on :{port}");

    {
        let mut hb = atlas.clone();
        tokio::spawn(async move {
            let mut tick = tokio::time::interval(Duration::from_secs(20));
            tick.tick().await;
            loop {
                tick.tick().await;
                if let Err(e) = hb.heartbeat(CAPABILITY_ID).await {
                    log::warn!("heartbeat failed: {e:#}");
                }
            }
        });
    }

    tonic::transport::Server::builder()
        .add_service(RobonixPrimitiveAudioMicServer::new(MockMic { pcm_data }))
        .add_service(RobonixPrimitiveAudioSpeakerServer::new(MockSpeaker {
            output_path,
        }))
        .serve(listen)
        .await?;
    Ok(())
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}
