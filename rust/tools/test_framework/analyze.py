#!/usr/bin/env python3
import json, sys, re, argparse, subprocess
from pathlib import Path
from report import BenchmarkReport
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np

# Update font cache and try to use Latin Modern Sans
try:
    subprocess.run(['fc-cache', '-f'], check=False)
except:
    pass

def setup_font():
    """Matplotlib 3.x+ robust font detection for Latin Modern Sans"""
    # Set default sizes for better readability
    plt.rcParams.update({
        'font.size': 16,
        'axes.labelsize': 18,
        'axes.titlesize': 22,
        'xtick.labelsize': 14,
        'ytick.labelsize': 14,
        'legend.fontsize': 16,
        'figure.titlesize': 26,
        'font.weight': 'normal',
        'axes.titleweight': 'normal'
    })
    try:
        import matplotlib.font_manager as fm
        
        # Check for Latin Modern Sans in different possible names
        target_fonts = ["Latin Modern Sans", "LM Sans 10", "LMSans10"]
        available_fonts = [f.name for f in fm.fontManager.ttflist]
        
        # Try finding by name first
        for font_name in target_fonts:
            if font_name in available_fonts:
                plt.rcParams['font.family'] = font_name
                print(f"Using font: {font_name}")
                return True
        
        # If not found by name, try to find by file patterns in system paths
        font_paths = fm.findSystemFonts()
        for path in font_paths:
            path_lower = path.lower()
            if 'lmsans' in path_lower and (path_lower.endswith('.otf') or path_lower.endswith('.ttf')):
                try:
                    fm.fontManager.addfont(path)
                    new_font = fm.FontProperties(fname=path).get_name()
                    plt.rcParams['font.family'] = new_font
                    print(f"Added and using font from: {path} (Name: {new_font})")
                    return True
                except:
                    continue
    except Exception as e:
        print(f"Warning: Font setup failed: {e}")
    
    plt.rcParams['font.family'] = 'sans-serif'
    print("Warning: Latin Modern Sans font not found, using default sans-serif")
    return False

setup_font()

def main():
    if len(sys.argv) > 1:
        log_base_dir = Path(sys.argv[1])
    else:
        # If no dir provided, find the latest benchmark_* folder in logs
        logs_root = Path("logs")
        if not logs_root.exists(): return
        bench_dirs = sorted([d for d in logs_root.iterdir() if d.is_dir() and d.name.startswith("benchmark_")], reverse=True)
        if not bench_dirs: return
        log_base_dir = bench_dirs[0]
    
    print(f"Processing benchmark directory: {log_base_dir}")
    test_dirs = sorted([d for d in log_base_dir.iterdir() if d.is_dir() and "_c" in d.name])
    if not test_dirs: 
        print("No test directories found.")
        return

    all_results = []
    metadata = {}
    for d in test_dirs:
        c_match = re.search(r'_c(\d+)(?:_|$)', d.name)
        concurrency = int(c_match.group(1)) if c_match else 0
        report = BenchmarkReport(d, concurrency=concurrency)
        res = report.collect_results()
        
        # Try to load saved metadata from the report file if it exists
        report_file = d / "benchmark_report.json"
        if report_file.exists():
            try:
                with open(report_file, 'r') as f:
                    report_data = json.load(f)
                    if 'metadata' in report_data:
                        # Update global metadata (assuming it's the same across all tests in the benchmark)
                        metadata.update(report_data['metadata'])
            except:
                pass

        for t_type, stats in res["stats"].items():
            all_results.append({"type": t_type, "concurrency": concurrency, "stats": stats, "name": d.name})

    if not all_results:
        print("No results to report.")
        return

    # Create a parameter string for titles
    param_str = ""
    if metadata:
        parts = []
        if metadata.get('requests'): parts.append(f"Reqs: {metadata['requests']}")
        if metadata.get('rate'): parts.append(f"Rate: {metadata['rate']}Hz")
        if metadata.get('duration'): parts.append(f"Dur: {metadata['duration']}s")
        if parts:
            param_str = "\n(" + ", ".join(parts) + ")"

    # Generate Text Report
    generate_text_report(all_results, log_base_dir)

    # Prepare data for plotting
    concurrencies = sorted(list(set(r["concurrency"] for r in all_results)))
    
    # Each entry will store: { 'p50': [mean1, mean2...], 'p50_min': [...], 'p50_max': [...], 'p99': [...], 'fail_rate': [...], 'fail_rate_min': [...], 'fail_rate_max': [...] }
    plot_data = {
        'rust': {'p50': [], 'p50_min': [], 'p50_max': [], 'p99': [], 'p99_min': [], 'p99_max': [], 'fail_rate': [], 'fail_rate_min': [], 'fail_rate_max': []},
        'cpp': {'p50': [], 'p50_min': [], 'p50_max': [], 'p99': [], 'p99_min': [], 'p99_max': [], 'fail_rate': [], 'fail_rate_min': [], 'fail_rate_max': []},
        'python': {'p50': [], 'p50_min': [], 'p50_max': [], 'p99': [], 'p99_min': [], 'p99_max': [], 'fail_rate': [], 'fail_rate_min': [], 'fail_rate_max': []}
    }

    for c in concurrencies:
        for t in ['rust', 'cpp', 'python']:
            matches = [r for r in all_results if r['type'] == t and r['concurrency'] == c]
            if matches:
                p50s = [r['stats'].get('p50', 0) for r in matches]
                p99s = [r['stats'].get('p99', 0) for r in matches]
                fails = [(r['stats']['failed'] / r['stats']['total'] * 100) if r['stats']['total'] > 0 else 0 for r in matches]
                
                plot_data[t]['p50'].append(np.mean(p50s))
                plot_data[t]['p50_min'].append(np.min(p50s))
                plot_data[t]['p50_max'].append(np.max(p50s))
                
                plot_data[t]['p99'].append(np.mean(p99s))
                plot_data[t]['p99_min'].append(np.min(p99s))
                plot_data[t]['p99_max'].append(np.max(p99s))
                
                plot_data[t]['fail_rate'].append(np.mean(fails))
                plot_data[t]['fail_rate_min'].append(np.min(fails))
                plot_data[t]['fail_rate_max'].append(np.max(fails))
            else:
                for k in plot_data[t]: plot_data[t][k].append(0)

    # Plot Latency Comparison - Bar Charts in 1x2 Grid
    fig, axes = plt.subplots(1, 2, figsize=(22, 11))
    
    metrics = ['p50', 'p99']
    types = ['rust', 'cpp', 'python']
    colors = {'rust': '#3498db', 'cpp': '#2ecc71', 'python': '#e74c3c'}
    
    x = np.arange(len(concurrencies))
    width = 0.25  
    
    for j, m in enumerate(metrics):
        ax = axes[j]
        
        for i, t in enumerate(types):
            means = plot_data[t][m]
            mins = plot_data[t][f'{m}_min']
            maxs = plot_data[t][f'{m}_max']
            
            offset = (i - 1) * width
            
            # Plot the mean bar
            bars = ax.bar(x + offset, means, width, label=t.upper(), 
                          color=colors[t], alpha=0.7, edgecolor='black', linewidth=1.5)
            
            # Plot min-max whiskers (OS paper style)
            yerr = [np.array(means) - np.array(mins), np.array(maxs) - np.array(means)]
            ax.errorbar(x + offset, means, yerr=yerr, fmt='none', ecolor='black', 
                        capsize=5, capthick=1.5, elinewidth=1.5)
            
            # Bar label (integer mean)
            ax.bar_label(bars, fmt='%d', padding=5, fontsize=11, fontweight='bold')
        
        ax.set_xticks(x)
        ax.set_xticklabels(concurrencies)
        ax.set_xlabel('Concurrency (Concurrent Clients)')
        ax.set_ylabel('Latency (us)')
        ax.set_title(f'{m.upper()} Latency (Mean w/ Min-Max)', pad=25)
        ax.grid(True, axis='y', linestyle='--', alpha=0.3)
        
        all_maxs = [v for t in types for v in plot_data[t][f'{m}_max']]
        if all_maxs:
            ax.set_ylim(0, max(all_maxs) * 1.35)

    # Place a single legend for both subplots at the top
    handles, labels = axes[0].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper center', bbox_to_anchor=(0.5, 0.92), ncol=3, frameon=False, fontsize=16)

    plt.suptitle(f'Latency Comparison for Requesting /rbnx/ping{param_str}', fontsize=26, y=0.98)
    plt.subplots_adjust(wspace=0.3, top=0.82, bottom=0.15, left=0.08, right=0.96)
    plt.savefig(log_base_dir / "latency_comparison.png")
    plt.close()
    print(f"Latency comparison saved to {log_base_dir / 'latency_comparison.png'}")

    # Plot Failure Rate Comparison - Bar Charts
    plt.figure(figsize=(12, 8))
    
    types = ['rust', 'cpp', 'python']
    colors = {'rust': '#3498db', 'cpp': '#2ecc71', 'python': '#e74c3c'}
    
    x = np.arange(len(concurrencies))
    width = 0.25
    
    for i, t in enumerate(types):
        means = plot_data[t]['fail_rate']
        mins = plot_data[t]['fail_rate_min']
        maxs = plot_data[t]['fail_rate_max']
        
        offset = (i - 1) * width
        
        # Plot the mean bar
        bars = plt.bar(x + offset, means, width, label=t.upper(), 
                      color=colors[t], alpha=0.7, edgecolor='black', linewidth=1.5)
        
        # Plot min-max whiskers (OS paper style)
        yerr = [np.array(means) - np.array(mins), np.array(maxs) - np.array(means)]
        plt.errorbar(x + offset, means, yerr=yerr, fmt='none', ecolor='black', 
                    capsize=5, capthick=1.5, elinewidth=1.5)
        
        # Bar label (precision reduced if very small)
        for bar in bars:
            height = bar.get_height()
            if height > 0:
                plt.text(bar.get_x() + bar.get_width()/2., height + 0.1,
                        f'{height:.1f}%', ha='center', va='bottom', fontsize=10, fontweight='bold')

    plt.xticks(x, concurrencies)
    plt.xlabel('Concurrency (Concurrent Clients)')
    plt.ylabel('Failure Rate (%)')
    plt.title(f'Failure Rate Comparison for Requesting /rbnx/ping{param_str}', pad=30)
    plt.grid(True, axis='y', linestyle='--', alpha=0.3)
    plt.legend(loc='upper left', frameon=False, fontsize=14)
    
    # Ensure y-axis starts at 0 and has some headroom
    all_maxs = [v for t in types for v in plot_data[t]['fail_rate_max']]
    if all_maxs and max(all_maxs) > 0:
        plt.ylim(0, max(all_maxs) * 1.4)
    else:
        plt.ylim(0, 10)  # Default if all zero
        
    plt.tight_layout()
    plt.savefig(log_base_dir / "failure_rate_comparison.png")
    plt.close()
    print(f"Failure rate comparison saved to {log_base_dir / 'failure_rate_comparison.png'}")

def generate_text_report(all_results, output_dir):
    # Group results by (concurrency, type)
    groups = {}
    for r in all_results:
        key = (r["concurrency"], r["type"])
        if key not in groups: groups[key] = []
        groups[key].append(r)

    report_lines = []
    report_lines.append("=" * 110)
    report_lines.append(f"{'Robonix Benchmark Summary Report (Averages across Repeats)':^110}")
    report_lines.append("=" * 110)
    report_lines.append(f"{'Conc':<6} {'Type':<10} {'Reps':>5} {'Total':>8} {'Succ':>8} {'Fail':>8} {'Rate':>8} {'Avg':>10} {'P50':>10} {'P99':>10}")
    report_lines.append("-" * 110)

    for c in sorted(list(set(k[0] for k in groups.keys()))):
        for t in ['rust', 'cpp', 'python']:
            key = (c, t)
            if key not in groups: continue
            
            matches = groups[key]
            reps = len(matches)
            
            # Aggregate stats
            total = np.mean([r['stats']['total'] for r in matches])
            success = np.mean([r['stats']['success'] for r in matches])
            failed = np.mean([r['stats']['failed'] for r in matches])
            rate = np.mean([(r['stats']['success'] / r['stats']['total'] * 100) if r['stats']['total'] > 0 else 0 for r in matches])
            avg = np.mean([r['stats'].get('avg', 0) for r in matches])
            p50 = np.mean([r['stats'].get('p50', 0) for r in matches])
            p99 = np.mean([r['stats'].get('p99', 0) for r in matches])
            
            report_lines.append(f"{c:<6} {t:<10} {reps:>5} {total:>8.0f} {success:>8.0f} {failed:>8.0f} {rate:>7.1f}% {avg:>10.2f} {p50:>10.2f} {p99:>10.2f}")
        report_lines.append("-" * 110)

    report_text = "\n".join(report_lines)
    print(report_text)
    (output_dir / "benchmark_summary.txt").write_text(report_text)
    print(f"Summary report saved to {output_dir / 'benchmark_summary.txt'}")

if __name__ == '__main__': main()
