use anyhow::{Context, Result};
use clap::ValueEnum;
use resvg::tiny_skia::{Color, Pixmap, Transform};
use resvg::usvg;
use std::collections::HashMap;
use std::fmt::Write;
use std::path::{Path, PathBuf};

/// Font stack: JetBrains Mono preferred, then common monospace fallbacks.
const F: &str =
    "JetBrains Mono,Fira Mono,Fira Code,Consolas,DejaVu Sans Mono,Courier New,monospace";
/// PNG scale factor for sharper output.
const PNG_SCALE: f32 = 2.0;

/// `rbnx graph --format …` (also inferred from `-o` extension when flag omitted).
/// Both variants use the same layout: SVG via [`render_svg`], PNG by rasterizing that SVG with resvg.
#[derive(Clone, Copy, Debug, Default, ValueEnum)]
#[clap(rename_all = "lower")]
pub enum GraphOutputFormat {
    /// Raster PNG (internal SVG → resvg).
    #[default]
    Png,
    /// Vector SVG (internal renderer).
    Svg,
}

fn resolve_format(output: &Path, explicit: Option<GraphOutputFormat>) -> GraphOutputFormat {
    if let Some(f) = explicit {
        return f;
    }
    match output
        .extension()
        .and_then(|e| e.to_str())
        .map(str::to_lowercase)
    {
        Some(ext) if ext == "svg" => GraphOutputFormat::Svg,
        Some(ext) if ext == "png" => GraphOutputFormat::Png,
        _ => GraphOutputFormat::Png,
    }
}

// ─── entry ───────────────────────────────────────────────────────────────────

pub async fn execute(
    server: &str,
    output: PathBuf,
    format: Option<GraphOutputFormat>,
    test: bool,
) -> Result<()> {
    let fmt = resolve_format(&output, format);

    let (nodes, channels, title) = if test {
        let (n, c) = test_topology();
        (n, c, "Robonix topology (test)")
    } else {
        let ep = if server.starts_with("http") {
            server.to_string()
        } else {
            format!("http://{server}")
        };
        let mut sdk = robonix_sdk::RobonixClient::connect(&ep).await?;
        let nodes = sdk
            .query_nodes_opts(robonix_sdk::QueryNodesOpts::default())
            .await
            .context("query nodes")?;
        let rt = sdk.inspect_runtime().await.context("inspect runtime")?;
        (nodes, parse_channels(&rt), "Robonix topology")
    };

    let svg = render_svg(&nodes, &channels, title);
    match fmt {
        GraphOutputFormat::Svg => std::fs::write(&output, &svg).context("write svg")?,
        GraphOutputFormat::Png => to_png(&svg, &output)?,
    }
    println!("written to {}", output.display());
    Ok(())
}

// ─── channel data ────────────────────────────────────────────────────────────

struct Channel {
    consumer_id: String, // client side
    provider_id: String, // server side
    transport: String,
    interface_name: String,
    endpoint: String, // allocated / negotiated endpoint
}

fn parse_channels(json: &str) -> Vec<Channel> {
    let mut out = Vec::new();
    let Ok(v) = serde_json::from_str::<serde_json::Value>(json) else {
        return out;
    };
    // The runtime serializes `channels` as HashMap<String, ChannelRecord>,
    // which becomes a JSON *object* (keyed by channel_id), not an array.
    let Some(obj) = v.get("channels").and_then(|x| x.as_object()) else {
        return out;
    };
    for c in obj.values() {
        let g = |k: &str| c.get(k).and_then(|x| x.as_str()).unwrap_or("").to_string();
        let (con, pro) = (g("consumer_id"), g("provider_node_id"));
        if con.is_empty() || pro.is_empty() {
            continue;
        }
        out.push(Channel {
            consumer_id: con,
            provider_id: pro,
            transport: g("transport"),
            interface_name: g("interface_name"),
            endpoint: g("endpoint"),
        });
    }
    out
}

fn meta_ep(json: &str) -> String {
    serde_json::from_str::<serde_json::Value>(json)
        .ok()
        .and_then(|v| {
            v.get("endpoint")
                .and_then(|e| e.as_str())
                .map(str::to_string)
        })
        .unwrap_or_default()
}

// ─── tiny PRNG (no deps, seeded from wall clock) ─────────────────────────────

struct Rng(u64);
impl Rng {
    fn new() -> Self {
        use std::time::{SystemTime, UNIX_EPOCH};
        let seed = SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0xdeadbeef);
        Self(seed.wrapping_add(0xD6E8_FEB8_6637_597D))
    }
    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }
    fn usize(&mut self, n: usize) -> usize {
        if n == 0 {
            0
        } else {
            (self.next() % n as u64) as usize
        }
    }
    fn range(&mut self, lo: usize, hi: usize) -> usize {
        lo + self.usize(hi.saturating_sub(lo) + 1)
    }
}

const TRANSPORTS: &[&str] = &["grpc", "mcp", "ros2", "shared_memory"];
const PKGS: &[&str] = &["perception", "control", "planning", "io", "inference"];
const KINDS: &[&str] = &["primitive", "service", "skill"];

// ─── random test topology ─────────────────────────────────────────────────────

fn test_topology() -> (Vec<robonix_sdk::NodeInfo>, Vec<Channel>) {
    let mut r = Rng::new();

    // 3–5 nodes, each with 1–2 interfaces
    let n_nodes = r.range(3, 5);
    let mut nodes: Vec<robonix_sdk::NodeInfo> = (0..n_nodes)
        .map(|i| {
            let pkg = PKGS[r.usize(PKGS.len())];
            let kind = KINDS[r.usize(KINDS.len())];
            let n_if = r.range(1, 2);
            let interfaces = (0..n_if)
                .map(|j| {
                    // Pick 1–2 transports without duplicates
                    let mut ts = Vec::<String>::new();
                    for _ in 0..r.range(1, 2) {
                        let t = TRANSPORTS[r.usize(TRANSPORTS.len())].to_string();
                        if !ts.contains(&t) {
                            ts.push(t);
                        }
                    }
                    let port = 50100 + r.range(0, 199) as u16;
                    robonix_sdk::InterfaceInfo {
                        name: format!("/io/{pkg}_{i}_{j}"),
                        contract_id: format!("robonix/{pkg}/node{i}/port{j}"),
                        supported_transports: ts,
                        metadata_json: format!(r#"{{"endpoint":"127.0.0.1:{port}"}}"#),
                    }
                })
                .collect();
            robonix_sdk::NodeInfo {
                node_id: format!("{pkg}.node{i}"),
                namespace: format!("ns/{pkg}"),
                kind: kind.to_string(),
                interfaces,
                has_skill_md: false,
                skills: vec![],
                distro: String::new(),
                container_id: String::new(),
                last_heartbeat_ms: 0,
            }
        })
        .collect();

    // Generate n_nodes-1 .. n_nodes channels (a sparse but connected-ish graph)
    let n_ch = r.range(n_nodes - 1, n_nodes);
    let mut channels = Vec::<Channel>::new();
    for _ in 0..n_ch {
        let mut pi = r.usize(n_nodes);
        let mut ci = r.usize(n_nodes);
        // avoid self-loop; try once to find a different pair
        if pi == ci && n_nodes > 1 {
            ci = (ci + 1) % n_nodes;
        }
        if pi == ci {
            continue;
        }

        if nodes[pi].interfaces.is_empty() {
            continue;
        }
        let iface = &nodes[pi].interfaces[r.usize(nodes[pi].interfaces.len())];
        let tr = iface.supported_transports[r.usize(iface.supported_transports.len())].clone();
        let port = 50200 + r.range(0, 99) as u16;
        let ep = format!("127.0.0.1:{port}");

        // Clone data needed before borrowing nodes mutably
        let iface_name = iface.name.clone();
        let abs_id = iface.contract_id.clone();

        // Add a matching (client-side) interface on the consumer node if missing
        if !nodes[ci].interfaces.iter().any(|x| x.name == iface_name) {
            nodes[ci].interfaces.push(robonix_sdk::InterfaceInfo {
                name: iface_name.clone(),
                contract_id: abs_id,
                supported_transports: vec![tr.clone()],
                metadata_json: format!(r#"{{"endpoint":"{ep}"}}"#),
            });
        }

        channels.push(Channel {
            provider_id: nodes[pi].node_id.clone(),
            consumer_id: nodes[ci].node_id.clone(),
            transport: tr,
            interface_name: iface_name,
            endpoint: ep,
        });
        // prevent the same pair from being picked again (swap pi to avoid alias)
        pi = pi.wrapping_add(1) % n_nodes;
        let _ = pi; // unused after swap; suppress warning
    }

    (nodes, channels)
}

// ─── helpers ─────────────────────────────────────────────────────────────────

fn esc(s: &str) -> String {
    s.replace('&', "&amp;")
        .replace('<', "&lt;")
        .replace('>', "&gt;")
        .replace('"', "&quot;")
}

/// Transport-specific color (for edge lines and connection dots).
fn transport_color(t: &str) -> &'static str {
    match t {
        "grpc" => "#2563eb",
        "mcp" => "#16a34a",
        "ros2" => "#dc2626",
        "shared_memory" => "#7c3aed",
        _ => "#6b7280",
    }
}

/// Role badge background color.
fn role_bg(server: bool, client: bool) -> &'static str {
    match (server, client) {
        (true, false) => "#1d4ed8", // blue
        (false, true) => "#15803d", // green
        (true, true) => "#6d28d9",  // purple
        _ => "#94a3b8",             // gray / idle
    }
}

fn role_label(server: bool, client: bool) -> &'static str {
    match (server, client) {
        (true, false) => "SERVER",
        (false, true) => "CLIENT",
        (true, true) => "S+C",
        _ => "idle",
    }
}

// ─── geometry ────────────────────────────────────────────────────────────────

/// Rough text width heuristic. JetBrains Mono is monospace; ~0.62× font-size per char.
fn tw(s: &str, sz: f64) -> f64 {
    s.len() as f64 * sz * 0.63
}

const HDR_H: f64 = 58.0; // node header area height
const INSET: f64 = 8.0; // iface box inset from card edge
const BPAD: f64 = 7.0; // top/bottom padding inside iface box
const BGAP: f64 = 5.0; // gap between iface boxes
const LH: f64 = 13.5; // line height
const PIN_R: f64 = 5.0; // connection dot radius

#[derive(Clone)]
struct BoxGeom {
    y0: f64, // top of box, relative to node top
    h: f64,
    y_center: f64, // mid-height, relative to node top
}

#[derive(Clone)]
struct NodeGeom {
    w: f64,
    h: f64,
    boxes: HashMap<String, BoxGeom>, // interface_name → geometry
}

fn measure(n: &robonix_sdk::NodeInfo, channels: &[Channel]) -> NodeGeom {
    let short = n.node_id.rsplit('.').next().unwrap_or(&n.node_id);
    let mut max_w: f64 = 210.0_f64
        .max(tw(short, 13.0) + 2.0 * INSET + 12.0)
        .max(tw(&n.node_id, 9.5) + 2.0 * INSET + 12.0);

    let mut y = HDR_H + BGAP;
    let mut boxes = HashMap::new();

    for iface in &n.interfaces {
        let is_server = channels
            .iter()
            .any(|c| c.provider_id == n.node_id && c.interface_name == iface.name);
        let is_client = channels
            .iter()
            .any(|c| c.consumer_id == n.node_id && c.interface_name == iface.name);

        let active = channels.iter().find(|c| {
            (c.provider_id == n.node_id || c.consumer_id == n.node_id)
                && c.interface_name == iface.name
        });
        let tr = active.map(|c| c.transport.as_str()).unwrap_or_else(|| {
            iface
                .supported_transports
                .first()
                .map(|s| s.as_str())
                .unwrap_or("?")
        });
        let ep = active
            .map(|c| c.endpoint.clone())
            .filter(|e| !e.is_empty())
            .unwrap_or_else(|| meta_ep(&iface.metadata_json));
        let abs = if iface.contract_id.is_empty() {
            iface.name.as_str()
        } else {
            iface.contract_id.as_str()
        };
        let ep_line = format!("{tr}::{abs}:{ep}");

        // Rows: role+name / endpoint / peer (when active)
        let has_peer = is_server || is_client;
        let rows = 2 + usize::from(has_peer);
        let box_h = BPAD + rows as f64 * LH + BPAD;

        max_w = max_w
            .max(tw(&ep_line, 9.0) + 2.0 * INSET + 14.0)
            .max(tw(&iface.name, 9.0) + 2.0 * INSET + 65.0);

        boxes.insert(
            iface.name.clone(),
            BoxGeom {
                y0: y,
                h: box_h,
                y_center: y + box_h * 0.5,
            },
        );
        y += box_h + BGAP;
    }
    if n.interfaces.is_empty() {
        y += LH + BPAD;
    }
    y += 10.0;

    NodeGeom {
        w: max_w,
        h: y,
        boxes,
    }
}

// ─── layout ──────────────────────────────────────────────────────────────────

struct Placed {
    idx: usize,
    x: f64,
    y: f64,
    geom: NodeGeom,
}

/// consumer_rank = provider_rank + 1 → producers on left, consumers right.
fn rank_nodes(nodes: &[robonix_sdk::NodeInfo], channels: &[Channel]) -> Vec<usize> {
    let id_to_i: HashMap<&str, usize> = nodes
        .iter()
        .enumerate()
        .map(|(i, n)| (n.node_id.as_str(), i))
        .collect();
    let mut rank = vec![0usize; nodes.len()];
    for _ in 0..nodes.len() + channels.len() + 4 {
        let mut changed = false;
        for ch in channels {
            let (Some(&pi), Some(&ci)) = (
                id_to_i.get(ch.provider_id.as_str()),
                id_to_i.get(ch.consumer_id.as_str()),
            ) else {
                continue;
            };
            let nr = rank[pi] + 1;
            if nr > rank[ci] {
                rank[ci] = nr;
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    rank
}

fn layout(nodes: &[robonix_sdk::NodeInfo], channels: &[Channel]) -> (Vec<Placed>, f64, f64) {
    let geoms: Vec<NodeGeom> = nodes.iter().map(|n| measure(n, channels)).collect();
    let ranks = rank_nodes(nodes, channels);

    let mut layers: HashMap<usize, Vec<usize>> = HashMap::new();
    for (i, &r) in ranks.iter().enumerate() {
        layers.entry(r).or_default().push(i);
    }
    let mut lkeys: Vec<usize> = layers.keys().copied().collect();
    lkeys.sort_unstable();
    for k in &lkeys {
        layers
            .get_mut(k)
            .unwrap()
            .sort_by_key(|&i| &nodes[i].node_id);
    }

    const MARGIN: f64 = 28.0;
    const COL_GAP: f64 = 76.0;
    const ROW_GAP: f64 = 22.0;
    const TITLE_H: f64 = 26.0;

    let col_ws: Vec<f64> = lkeys
        .iter()
        .map(|k| {
            layers[k]
                .iter()
                .map(|&i| geoms[i].w)
                .fold(0.0_f64, f64::max)
        })
        .collect();
    let col_hs: Vec<f64> = lkeys
        .iter()
        .map(|k| {
            let idxs = &layers[k];
            idxs.iter()
                .enumerate()
                .map(|(j, &i)| geoms[i].h + if j + 1 < idxs.len() { ROW_GAP } else { 0.0 })
                .sum::<f64>()
        })
        .collect();

    let max_h = col_hs.iter().copied().fold(0.0_f64, f64::max);
    let canvas_h = MARGIN + TITLE_H + max_h + MARGIN;

    let mut cx = MARGIN;
    let mut placed = Vec::new();
    for (li, k) in lkeys.iter().enumerate() {
        let idxs = &layers[k];
        let stack_h = col_hs[li];
        let mut cy = MARGIN + TITLE_H + (max_h - stack_h) * 0.5;
        for &ni in idxs {
            placed.push(Placed {
                idx: ni,
                x: cx,
                y: cy,
                geom: geoms[ni].clone(),
            });
            cy += geoms[ni].h + ROW_GAP;
        }
        cx += col_ws[li] + COL_GAP;
    }

    let canvas_w = (cx - COL_GAP + MARGIN).max(MARGIN * 2.0);
    (placed, canvas_w, canvas_h)
}

// ─── SVG renderer ────────────────────────────────────────────────────────────
//
// All SVG attribute values use single quotes to avoid `"#rrggbb"` conflicting
// with Rust's `r#"..."#` raw-string delimiters.

fn render_svg(nodes: &[robonix_sdk::NodeInfo], channels: &[Channel], title: &str) -> String {
    let (placed, cw, ch) = layout(nodes, channels);
    let mut s = String::with_capacity(32_768);

    // Pure white background, minimalist.
    writeln!(
        s,
        "<svg xmlns='http://www.w3.org/2000/svg' width='{cw:.0}' height='{ch:.0}'>"
    )
    .unwrap();
    // Arrow marker — small, sharp, dark gray.
    writeln!(s, "<defs><marker id='arr' markerWidth='7' markerHeight='6' refX='6' refY='3' orient='auto'><path d='M0,0 L6,3 L0,6 z' fill='#333333'/></marker></defs>").unwrap();
    writeln!(s, "<rect width='{cw:.0}' height='{ch:.0}' fill='#ffffff'/>").unwrap();
    writeln!(s, "<text x='28' y='19' font-family='{F}' font-size='12' font-weight='bold' fill='#111111'>{}</text>", esc(title)).unwrap();

    // ── Channel edges — drawn FIRST (under nodes) ─────────────────────────────
    // Cubic bezier curves (S-bend): smooth, and clearly distinguishable even
    // when same-colored lines cross.  Parallel edges between the same node pair
    // are vertically offset at the control points so they fan out rather than
    // overlap.
    {
        let by_id: HashMap<&str, &Placed> = placed
            .iter()
            .map(|p| (nodes[p.idx].node_id.as_str(), p))
            .collect();

        for (chi, ch) in channels.iter().enumerate() {
            let (Some(pp), Some(pc)) = (
                by_id.get(ch.provider_id.as_str()),
                by_id.get(ch.consumer_id.as_str()),
            ) else {
                continue;
            };

            let y_srv = pp.y
                + pp.geom
                    .boxes
                    .get(&ch.interface_name)
                    .map(|b| b.y_center)
                    .unwrap_or(pp.geom.h * 0.5);
            let y_cli = pc.y
                + pc.geom
                    .boxes
                    .get(&ch.interface_name)
                    .map(|b| b.y_center)
                    .unwrap_or(pc.geom.h * 0.5);
            let x0 = pp.x + pp.geom.w; // server right edge
            let x1 = pc.x; // client left edge
            let color = transport_color(&ch.transport);

            // For parallel edges between the same (provider, consumer) pair,
            // fan the control-point y-offsets so each curve takes its own arc.
            let same: Vec<usize> = channels
                .iter()
                .enumerate()
                .filter(|(_, c)| c.provider_id == ch.provider_id && c.consumer_id == ch.consumer_id)
                .map(|(i, _)| i)
                .collect();
            let k = same.iter().position(|&i| i == chi).unwrap_or(0) as f64;
            let n = same.len() as f64;
            // Vertical fan: spread control points ±20 px per slot.
            let fan = (k - (n - 1.0) * 0.5) * 20.0;

            // Horizontal pull: bezier "handles" extend 45% of the gap outward.
            let dx = (x1 - x0).abs();
            let ctrl = dx * 0.45 + 30.0; // minimum 30 px even for close nodes

            // Control points:  cp1 pulls right from start, cp2 pulls left from end.
            // The fan shifts both control points vertically to separate parallel lines.
            let (cx1, cy1) = (x0 + ctrl, y_srv + fan);
            let (cx2, cy2) = (x1 - ctrl, y_cli + fan);

            let d = format!(
                "M{x0:.1},{y_srv:.1} C{cx1:.1},{cy1:.1} {cx2:.1},{cy2:.1} {x1:.1},{y_cli:.1}"
            );
            writeln!(s, "<path d='{d}' fill='none' stroke='{color}' stroke-width='1.5' marker-end='url(#arr)'/>").unwrap();
        }
    }

    // ── Node cards ────────────────────────────────────────────────────────────
    // White fill covers any edge that passes through the card interior.
    // No shadows, no gradient header. Clean 1px border.
    for p in &placed {
        let n = &nodes[p.idx];
        let (gx, gy, gw, gh) = (p.x, p.y, p.geom.w, p.geom.h);
        let tx = gx + INSET;

        // Card
        writeln!(s, "<rect x='{gx:.1}' y='{gy:.1}' width='{gw:.1}' height='{gh:.1}' fill='#ffffff' stroke='#222222' stroke-width='1'/>").unwrap();

        // Node name (bold) and metadata
        writeln!(s, "<text x='{tx:.1}' y='{:.1}' font-family='{F}' font-size='12' font-weight='bold' fill='#111111'>{}</text>",
            gy + 15.0, esc(n.node_id.rsplit('.').next().unwrap_or(&n.node_id))).unwrap();
        writeln!(s, "<text x='{tx:.1}' y='{:.1}' font-family='{F}' font-size='9' fill='#555555'>{} / {}</text>",
            gy + 27.0, esc(&n.kind), esc(&n.namespace)).unwrap();
        writeln!(s, "<text x='{tx:.1}' y='{:.1}' font-family='{F}' font-size='8.5' fill='#999999'>{}</text>",
            gy + 38.0, esc(&n.node_id)).unwrap();

        // Thin divider below header
        writeln!(s, "<line x1='{gx:.1}' y1='{:.1}' x2='{:.1}' y2='{:.1}' stroke='#dddddd' stroke-width='0.75'/>",
            gy + HDR_H, gx + gw, gy + HDR_H).unwrap();

        if n.interfaces.is_empty() {
            writeln!(s, "<text x='{tx:.1}' y='{:.1}' font-family='{F}' font-size='9' fill='#aaaaaa' font-style='italic'>no interfaces</text>",
                gy + HDR_H + LH + 2.0).unwrap();
        }

        // ── Interface sub-boxes ──────────────────────────────────────────────
        // Each box: thin border, 3px left accent in transport color when active.
        for iface in &n.interfaces {
            let Some(bg) = p.geom.boxes.get(&iface.name) else {
                continue;
            };
            let (bx, by, bw, bh) = (gx + INSET, gy + bg.y0, gw - 2.0 * INSET, bg.h);

            let is_server = channels
                .iter()
                .any(|c| c.provider_id == n.node_id && c.interface_name == iface.name);
            let is_client = channels
                .iter()
                .any(|c| c.consumer_id == n.node_id && c.interface_name == iface.name);

            let active = channels.iter().find(|c| {
                (c.provider_id == n.node_id || c.consumer_id == n.node_id)
                    && c.interface_name == iface.name
            });
            let tr = active.map(|c| c.transport.as_str()).unwrap_or_else(|| {
                iface
                    .supported_transports
                    .first()
                    .map(|s| s.as_str())
                    .unwrap_or("?")
            });
            let ep = active
                .map(|c| c.endpoint.clone())
                .filter(|e| !e.is_empty())
                .unwrap_or_else(|| meta_ep(&iface.metadata_json));
            let abs = if iface.contract_id.is_empty() {
                iface.name.as_str()
            } else {
                iface.contract_id.as_str()
            };
            let ep_label = format!("{tr}::{abs}:{ep}");

            let tc = transport_color(tr);
            let rbg = role_bg(is_server, is_client);
            let rl = role_label(is_server, is_client);

            // Box outline: thin light gray
            writeln!(s, "<rect x='{bx:.1}' y='{by:.1}' width='{bw:.1}' height='{bh:.1}' fill='#ffffff' stroke='#cccccc' stroke-width='0.75'/>").unwrap();

            // Left accent bar: transport color when active, invisible when idle
            if is_server || is_client {
                writeln!(
                    s,
                    "<rect x='{bx:.1}' y='{by:.1}' width='3' height='{bh:.1}' fill='{tc}'/>"
                )
                .unwrap();
            }

            // Text starts after the accent bar
            let tx2 = bx + 8.0;

            // Row 1: role badge + interface name
            let badge_w = rl.len() as f64 * 5.2 + 8.0;
            let row1_y = by + BPAD;
            writeln!(
                s,
                "<rect x='{tx2:.1}' y='{row1_y:.1}' width='{badge_w:.1}' height='11' fill='{rbg}'/>"
            )
            .unwrap();
            writeln!(s, "<text x='{:.1}' y='{:.1}' font-family='{F}' font-size='7.5' font-weight='bold' fill='#ffffff' text-anchor='middle'>{}</text>",
                tx2 + badge_w * 0.5, row1_y + 8.5, esc(rl)).unwrap();
            writeln!(s, "<text x='{:.1}' y='{:.1}' font-family='{F}' font-size='9' fill='#333333'>{}</text>",
                tx2 + badge_w + 5.0, row1_y + 9.0, esc(&iface.name)).unwrap();

            // Row 2: transport::contract_id:allocated_endpoint
            writeln!(s, "<text x='{tx2:.1}' y='{:.1}' font-family='{F}' font-size='9' fill='#111111'>{}</text>",
                by + BPAD + LH + 8.5, esc(&ep_label)).unwrap();

            // Row 3: peer info
            if is_server {
                let consumers: Vec<String> = channels
                    .iter()
                    .filter(|c| c.provider_id == n.node_id && c.interface_name == iface.name)
                    .map(|c| {
                        c.consumer_id
                            .rsplit('.')
                            .next()
                            .unwrap_or(&c.consumer_id)
                            .to_string()
                    })
                    .collect();
                if !consumers.is_empty() {
                    writeln!(s, "<text x='{tx2:.1}' y='{:.1}' font-family='{F}' font-size='8.5' fill='#666666'>consumed by: {}</text>",
                        by + BPAD + 2.0 * LH + 8.5, esc(&consumers.join(", "))).unwrap();
                }
            } else if is_client
                && let Some(prov) = channels
                    .iter()
                    .find(|c| c.consumer_id == n.node_id && c.interface_name == iface.name)
                    .map(|c| {
                        c.provider_id
                            .rsplit('.')
                            .next()
                            .unwrap_or(&c.provider_id)
                            .to_string()
                    })
            {
                writeln!(s, "<text x='{tx2:.1}' y='{:.1}' font-family='{F}' font-size='8.5' fill='#666666'>provider: {}</text>",
                        by + BPAD + 2.0 * LH + 8.5, esc(&prov)).unwrap();
            }
        }

        // ── Connection dots on card edges (server = right, client = left) ────
        for iface in &n.interfaces {
            let Some(bg) = p.geom.boxes.get(&iface.name) else {
                continue;
            };
            let cy = gy + bg.y_center;

            let is_server = channels
                .iter()
                .any(|c| c.provider_id == n.node_id && c.interface_name == iface.name);
            let is_client = channels
                .iter()
                .any(|c| c.consumer_id == n.node_id && c.interface_name == iface.name);
            if !is_server && !is_client {
                continue;
            }

            let active = channels.iter().find(|c| {
                (c.provider_id == n.node_id || c.consumer_id == n.node_id)
                    && c.interface_name == iface.name
            });
            let tr = active.map(|c| c.transport.as_str()).unwrap_or_else(|| {
                iface
                    .supported_transports
                    .first()
                    .map(|s| s.as_str())
                    .unwrap_or("?")
            });
            let tc = transport_color(tr);

            // Server: dot on right card edge
            if is_server {
                writeln!(s, "<circle cx='{:.1}' cy='{cy:.1}' r='{PIN_R}' fill='{tc}' stroke='#ffffff' stroke-width='1'/>", gx + gw).unwrap();
            }
            // Client: dot on left card edge
            if is_client {
                writeln!(s, "<circle cx='{gx:.1}' cy='{cy:.1}' r='{PIN_R}' fill='{tc}' stroke='#ffffff' stroke-width='1'/>").unwrap();
            }
        }
    }

    s.push_str("</svg>");
    s
}

// ─── PNG rasterizer (2× scale for sharp output) ──────────────────────────────

fn to_png(svg: &str, path: &Path) -> Result<()> {
    let mut opt = usvg::Options::default();
    opt.fontdb_mut().load_system_fonts();
    opt.font_family = "JetBrains Mono".to_string();
    let tree = usvg::Tree::from_str(svg, &opt).context("parse svg")?;
    let sz = tree.size();
    let w = (sz.width() * PNG_SCALE).ceil().max(1.0) as u32;
    let h = (sz.height() * PNG_SCALE).ceil().max(1.0) as u32;
    let mut pix = Pixmap::new(w, h).context("alloc pixmap")?;
    pix.fill(Color::WHITE);
    resvg::render(
        &tree,
        Transform::from_scale(PNG_SCALE, PNG_SCALE),
        &mut pix.as_mut(),
    );
    pix.save_png(path)
        .map_err(|e| anyhow::anyhow!("png encode: {e}"))?;
    Ok(())
}
