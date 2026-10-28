<!-- SPDX-License-Identifier: MulanPSL-2.0 -->

# Robot render asset distribution

This document defines how URDF visual resources are prepared, stored,
distributed, fetched, streamed through Soma, and rendered by the Robonix
Client Vitals page. It applies to Mesh files, textures, generated visual-only
URDF files, and their license and provenance metadata.

## End-to-end model

Robot render assets pass through three separate stages:

1. A model maintainer prepares a visual-only URDF and the resources it
   references from a pinned, license-compatible upstream source.
2. Small resources are committed with the deployment; large resources are
   published as an immutable external artifact and installed by
   `scripts/fetch_model.py` before Soma starts.
3. Soma exposes the URDF resource manifest and streams requested files in
   bounded chunks. The Client verifies and caches each file, then serves it to
   the browser through a same-origin URL used by `urdf-loader`.

The deployment repository is responsible for acquiring the model. Soma and
the Client do not clone vendor repositories, expand Xacro, or repair invalid
URDF paths at runtime.

## Directory and path format

Use a stable directory layout beside the deployment's `soma.yaml`:

```text
robot-description/
├── soma.yaml
└── model/
    ├── robot.urdf
    ├── meshes/
    │   ├── dae/
    │   └── stl/
    ├── textures/
    └── licenses/
```

Every Mesh or texture reference must be a POSIX path relative to the directory
containing the URDF:

```xml
<mesh filename="meshes/dae/base_link.dae"/>
<mesh filename="meshes/stl/arm/link_1.stl"/>
<texture filename="textures/body.png"/>
```

References must:

- use `/` separators and match the filename's case exactly;
- stay below the URDF directory without `.` or `..` components;
- retain enough upstream directory structure to avoid name collisions;
- resolve to regular readable files, not symlinks escaping the model root.

Do not use absolute paths, backslashes, `package://`, `file://`, `http://`,
`https://`, or `data:` references for resources that must travel through Soma.
Rewrite those references while preparing the deployment model.

## Choose Git or external distribution

Use these review thresholds for the complete set of visual resources referenced
by one URDF:

| Condition | Storage policy |
|---|---|
| Every binary is at most 10 MiB and all visual resources total at most 50 MiB | The resources may be committed to ordinary Git. |
| Any binary exceeds 10 MiB or all visual resources total more than 50 MiB | Publish an immutable external artifact. |

These are project review thresholds, not statements about a hosting provider's
technical limits. A maintainer may require external distribution for a smaller
model because of licensing, update frequency, or repository growth. Do not
split a binary across Git objects or commit generated archives to bypass the
policy.

In both cases, keep these reviewable files in Git:

- the visual-only URDF with stable relative resource paths;
- source repository, exact tag or commit, license, and transformation notes;
- a deterministic manifest containing the prepared files' sizes and SHA-256
  hashes;
- any reproducible preparation or conversion script.

### Small checked-in models

Commit the referenced Meshes, textures, applicable license, URDF, provenance,
and manifest together. Regenerating one resource requires updating the URDF and
manifest in the same change. `examples/piper_vitals` is the reference example.

### Large externally distributed models

Git must contain the URDF and metadata, but ignore locally installed binary
roots such as `model/meshes/`, `model/textures/`, and `model/licenses/`. Also
keep:

- an executable `fetch_model.sh` wrapper around `scripts/fetch_model.py`;
- `model/model-assets.json`, which pins the external artifact and every
  extracted file;
- a source document describing how the artifact was reproduced.

Publish the binary archive through an institution object store, package
registry, or versioned release asset. Its URL must identify an immutable
version. Do not use `latest`, a mutable branch archive, or an unpinned upstream
URL. Keep the reproducible builder in the distribution repository while
excluding generated archives from its Git history.

Each distributed model must have its own reproducible build definition. A
complex model may use dedicated code for Xacro expansion, resource selection,
path rewriting, conversion, or decimation; a model that already follows the
required layout may use a small configuration over shared tooling. Common
download verification, deterministic archive creation, hashing, and manifest
generation should be implemented once and reused by all model definitions.
Lock every consumed upstream input by path, exact revision, byte size, and
SHA-256 so a mutable source server cannot silently change a release build.

## External artifact format

Use a `.tar`, `.tar.gz`, or `.tgz` archive. Archive paths must exactly match the
URDF-local paths and must belong to dedicated top-level install roots:

```text
meshes/dae/base_link.dae
meshes/stl/arm/link_1.stl
textures/body.png
licenses/LICENSE.txt
```

The archive may contain only regular files declared in `model-assets.json`.
Symlinks, duplicate members, undeclared files, absolute paths, backslashes, and
parent traversal are rejected.

A schema version 1 manifest has this shape:

```json
{
  "schemaVersion": 1,
  "modelId": "example-robot-visual",
  "version": "vendor-v1.2.3",
  "artifact": {
    "url": "https://artifacts.example.org/example-robot-v1.2.3.tar.gz",
    "bytes": 12345678,
    "sha256": "0000000000000000000000000000000000000000000000000000000000000000"
  },
  "installRoots": ["meshes"],
  "files": {
    "meshes/dae/base_link.dae": {
      "bytes": 123456,
      "sha256": "0000000000000000000000000000000000000000000000000000000000000000"
    }
  }
}
```

The archive byte count and SHA-256 cover the compressed artifact. Each `files`
entry records the exact extracted path, byte count, and SHA-256. The manifest
must declare every archive file, and each file must be below an `installRoots`
directory. Replace the zero digest placeholders above with measured SHA-256
values. Add `textures` or `licenses` to `installRoots` only when the archive and
`files` object contain at least one file below that root.

## Fetch and installation

Run the example wrapper before starting Soma:

```bash
examples/unitree_b2w_vitals/fetch_model.sh
```

The common fetcher:

- validates the manifest before network access;
- skips the network when the installed files already match;
- stores archives by SHA-256 under
  `${XDG_CACHE_HOME:-$HOME/.cache}/robonix/model-artifacts/`;
- resumes HTTP downloads when the server supports range requests;
- verifies the archive size and SHA-256 before extraction;
- extracts only declared regular files and verifies every file;
- atomically replaces only the declared top-level install roots and rolls back
  a failed replacement.

Useful options are:

```bash
# Require an already verified cache entry.
./fetch_model.sh --offline

# Reinstall even when the current destination passes verification.
./fetch_model.sh --force

# Use an institution mirror without changing the reviewed manifest.
./fetch_model.sh --artifact-url https://mirror.example.org/robot-v1.2.3.tar.gz
```

The mirror must serve the exact artifact pinned by the manifest because the
declared size and SHA-256 do not change.

## Soma-to-Client streaming

At runtime, Soma reads the URDF and validates all local resource references. It
returns URDF text plus path, size, SHA-256, and media type through the asset
manifest capability. Resource bytes are not placed in that unary response.

When `urdf-loader` requests a Mesh or texture, the browser calls the Client's
same-origin asset endpoint. A cache miss causes the Client to request that one
file from Soma's streaming capability. Soma reads it lazily and sends bounded
chunks; the Client writes a temporary file, verifies its size and SHA-256,
atomically publishes it into a content-addressed disk cache, and then serves it
to the browser. Concurrent requests for the same file share one download.

This separation means a 500 MiB robot bundle does not become one 500 MiB gRPC
message and does not have to be loaded into Soma memory. The browser still has
to parse and upload the requested geometry, so visual Meshes should be
decimated and textures compressed. Full-detail collision or CAD resources
should remain outside the Vitals rendering bundle.

## Client capacity

The Client defaults are intentionally bounded and can be increased before
startup:

| Environment variable | Default | Purpose |
|---|---:|---|
| `ROBONIX_CLIENT_MODEL_CACHE_DIR` | platform cache directory | Persistent model cache location. |
| `ROBONIX_CLIENT_MODEL_FILE_MAX_BYTES` | 1 GiB | Maximum size of one Mesh or texture. |
| `ROBONIX_CLIENT_MODEL_TOTAL_MAX_BYTES` | 4 GiB | Maximum total size of one robot model. |
| `ROBONIX_CLIENT_MODEL_CACHE_MAX_BYTES` | 8 GiB | Maximum disk cache size; must fit the active model. |
| `ROBONIX_CLIENT_MODEL_DOWNLOAD_CONCURRENCY` | 4 | Maximum simultaneous resource downloads. |

For example, permit a model totaling 12 GiB with individual resources up to
3 GiB:

```bash
export ROBONIX_CLIENT_MODEL_FILE_MAX_BYTES=$((3 * 1024 * 1024 * 1024))
export ROBONIX_CLIENT_MODEL_TOTAL_MAX_BYTES=$((12 * 1024 * 1024 * 1024))
export ROBONIX_CLIENT_MODEL_CACHE_MAX_BYTES=$((24 * 1024 * 1024 * 1024))
robonix-client --robot-host 127.0.0.1
```

Increasing limits changes admission and disk capacity only. It does not make a
high-polygon model inexpensive to parse or render.

## Release checklist

Before accepting a model version, verify all of the following:

- the upstream repository and exact revision are documented;
- redistribution of the prepared assets is permitted and the license is kept;
- only visual resources needed by the URDF are included;
- all URDF references use the required relative POSIX format;
- the Git-versus-external decision follows the size thresholds;
- manifests contain exact paths, byte counts, and SHA-256 values;
- external URLs are immutable and the fetch succeeds from an empty cache;
- Soma starts with the prepared model and the Client renders it without missing
  resource requests;
- large-model limits are configured for the deployment host when defaults are
  insufficient.

`examples/piper_vitals` demonstrates the small checked-in path.
`examples/unitree_b2w_vitals` demonstrates external distribution and runtime
streaming with a 71,748,110-byte individual DAE file.
