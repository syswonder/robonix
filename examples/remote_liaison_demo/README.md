# Remote Liaison Demo

This example is a small reusable remote voice path for Liaison:

```text
macOS mic/speaker
  -> SSH reverse tunnel
  -> audio_macos_bridge
  -> voiceprint/user access gate
  -> speech ASR
  -> Liaison + Pilot + Executor
  -> demo skills
  -> speech TTS
  -> macOS speaker
```

Linux runs Atlas, Executor, Pilot, Liaison, voiceprint, speech, the Mac audio
bridge primitive, and three lightweight skills. macOS only owns the physical
microphone and speaker.

The three skills are intentionally small but real Robonix skills:

- `status_skill`: reports demo status.
- `notes_skill`: records a note into its package-local `rbnx-build/data`.
- `summary_skill`: summarizes the demo and recorded notes.

Do not commit real cloud keys or enrolled voiceprint data. Put credentials in
your shell or a local `.env` file outside git.

## Mac Audio

Mac terminal 1:

```bash
cd ~/robonix-scripts/mac_server
source .venv/bin/activate
python3 server_web.py --host 127.0.0.1 --port 60000
```

Mac terminal 2:

```bash
ssh -N -R 60101:127.0.0.1:60000 <linux-user>@<linux-host>
```

Keep both Mac terminals open. If SSH prints `remote port forwarding failed`,
pick another Linux port and update `robonix_manifest.yaml` plus
`scripts/check_mac_audio.py`.

## Linux Run

```bash
cd examples/remote_liaison_demo

./scripts/clean_demo_state.sh
python3 scripts/check_mac_audio.py

export VLM_BASE_URL="https://api.deepseek.com"
export VLM_API_KEY="<your-vlm-api-key>"
export VLM_MODEL="deepseek-v4-flash"

export TENCENT_ASR_APPID="<your-tencent-asr-appid>"
export TENCENTCLOUD_SECRET_ID="<your-tencent-secret-id>"
export TENCENTCLOUD_SECRET_KEY="<your-tencent-secret-key>"
export TENCENT_ASR_ENGINE="16k_zh_en"
export TENCENT_TTS_VOICE_TYPE="1001"
export TENCENT_TTS_REGION="ap-guangzhou"

export ROBONIX_LIAISON_ALLOWED_USERS="voice:<your-user-id>"
# Optional GPU pin, for example:
# export VOICEPRINT_DEVICE="cuda:0"

rbnx build
rbnx boot
```

In another Linux terminal:

```bash
cd examples/remote_liaison_demo
rbnx chat
```

Press `Ctrl+V` in `rbnx chat`, speak to the Mac microphone, and listen from the
Mac speaker.

Try:

```text
Check the current demo status.
Remember that this demo uses remote Liaison with Pilot calling three skills.
Summarize the current demo.
```

## Voiceprint Enrolment

Run the demo first so Atlas, `audio_macos_bridge`, and `voiceprint` are active.
Then register one allowed speaker:

```bash
cd ../..
python3 examples/webots/scripts/enroll_voiceprint.py \
  --user-id <your-user-id> \
  --user-name <your-display-name> \
  --seconds 6
```

To replace an existing enrollment:

```bash
python3 examples/webots/scripts/delete_voiceprint.py --user-id <your-user-id>
python3 examples/webots/scripts/enroll_voiceprint.py \
  --user-id <your-user-id> \
  --user-name <your-display-name> \
  --seconds 6
```

`ROBONIX_LIAISON_ALLOWED_USERS` should include the matching voice identity,
for example `voice:alice`.
