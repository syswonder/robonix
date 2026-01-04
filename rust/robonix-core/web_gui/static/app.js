let autoRefreshInterval = null;

async function loadStatus() {
    try {
        const response = await fetch('/api/status');
        const status = await response.json();
        document.getElementById('core-status').textContent = status.core_initialized ? 'Online' : 'Offline';
        document.getElementById('active-tasks').textContent = status.active_tasks;
        document.getElementById('skills-count').textContent = status.registered_skills;
        document.getElementById('services-count').textContent = status.registered_services;
    } catch (error) {
        console.error('Failed to load status:', error);
    }
}

let currentTfTreeData = null;

function renderTfTree(data) {
    const container = document.getElementById('tf-tree-container');
    
    if (!data || !data.frames || data.frames.length === 0) {
        if (!currentTfTreeData) {
            container.innerHTML = 'No TF frames available';
        }
        return;
    }

    // Only update if data changed
    const dataStr = JSON.stringify(data);
    if (currentTfTreeData === dataStr) {
        return; // No change, skip update
    }
    currentTfTreeData = dataStr;

    const frameMap = new Map();
    data.frames.forEach(frame => {
        frameMap.set(frame.frame_id, frame);
    });

    function renderFrame(frameId, prefix = '', isLast = true) {
        const frame = frameMap.get(frameId);
        if (!frame) return '';

        let output = prefix + '<span style="color: #999;">' + (isLast ? '└── ' : '├── ') + '</span>' + '<strong>' + frameId + '</strong>';
        
        if (frame.transform) {
            const t = frame.transform.translation;
            output += ` <span style="color: #999;">[${t[0].toFixed(2)}, ${t[1].toFixed(2)}, ${t[2].toFixed(2)}]</span>`;
        }
        output += '\n';

        if (frame.child_frames && frame.child_frames.length > 0) {
            const childPrefix = prefix + '<span style="color: #999;">' + (isLast ? '    ' : '│   ') + '</span>';
            // Sort child frames alphabetically
            const sortedChildren = [...frame.child_frames].sort();
            sortedChildren.forEach((childId, index) => {
                const isLastChild = index === sortedChildren.length - 1;
                output += renderFrame(childId, childPrefix, isLastChild);
            });
        }

        return output;
    }

    let treeText = '';
    // Sort root frames alphabetically
    const sortedRootFrames = [...data.root_frames].sort();
    sortedRootFrames.forEach((rootId, index) => {
        const isLast = index === sortedRootFrames.length - 1;
        treeText += renderFrame(rootId, '', isLast);
    });

    container.innerHTML = treeText || 'No frames found';
}

async function loadTfTree() {
    const container = document.getElementById('tf-tree-container');
    
    try {
        const response = await fetch('/api/tf-tree');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        renderTfTree(data);
    } catch (error) {
        console.error('Failed to load TF tree:', error);
        if (!currentTfTreeData) {
            container.innerHTML = `Error: ${error.message}`;
        }
    }
}

let currentTopicsData = null;

function renderTopics(data) {
    const container = document.getElementById('topics-container');
    
    if (!data || !data.topics || data.topics.length === 0) {
        if (!currentTopicsData) {
            container.innerHTML = 'No topics available';
        }
        return;
    }

    // Only update if data changed
    const dataStr = JSON.stringify(data);
    if (currentTopicsData === dataStr) {
        return; // No change, skip update
    }
    currentTopicsData = dataStr;

    let topicsHtml = '';
    data.topics.forEach(topic => {
        const freqText = topic.frequency !== null && topic.frequency !== undefined
            ? `${topic.frequency.toFixed(2)} Hz`
            : '-';
        
        topicsHtml += `<div class="topic-entry">`;
        topicsHtml += `<span class="topic-name">${topic.name}</span>`;
        topicsHtml += `<span class="topic-type">${topic.message_type}</span>`;
        topicsHtml += `<span class="topic-frequency">${freqText}</span>`;
        topicsHtml += `</div>`;
    });

    container.innerHTML = topicsHtml || 'No topics found';
}

async function loadTopics() {
    const container = document.getElementById('topics-container');
    
    try {
        const response = await fetch('/api/topics');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const data = await response.json();
        renderTopics(data);
    } catch (error) {
        console.error('Failed to load topics:', error);
        if (!currentTopicsData) {
            container.innerHTML = `Error: ${error.message}`;
        }
    }
}

function setupAutoRefresh() {
    const checkbox = document.getElementById('auto-refresh');
    checkbox.addEventListener('change', (e) => {
        if (e.target.checked) {
            startAutoRefresh();
        } else {
            stopAutoRefresh();
        }
    });
    if (checkbox.checked) {
        startAutoRefresh();
    }
}

function startAutoRefresh() {
    if (autoRefreshInterval) {
        clearInterval(autoRefreshInterval);
    }
    autoRefreshInterval = setInterval(() => {
        loadTfTree();
        loadTopics();
        loadStatus();
    }, 2000);
}

function stopAutoRefresh() {
    if (autoRefreshInterval) {
        clearInterval(autoRefreshInterval);
        autoRefreshInterval = null;
    }
}

let autoRefreshLogsInterval = null;

async function loadLogs() {
    const container = document.getElementById('logs-container');
    
    try {
        const response = await fetch('/api/logs?limit=200');
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        const logs = await response.json();
        renderLogs(logs);
    } catch (error) {
        console.error('Failed to load logs:', error);
        if (logEntriesMap.size === 0) {
            container.innerHTML = `Error: ${error.message}`;
        }
    }
}

let lastLogTimestamp = null;
let logEntriesMap = new Map();

function renderLogs(logs) {
    const container = document.getElementById('logs-container');
    
    if (!logs || logs.length === 0) {
        if (logEntriesMap.size === 0) {
            container.innerHTML = 'No logs available';
        }
        return;
    }

    // Parse timestamp to number for proper comparison (format: "seconds.microseconds")
    // Must use numeric comparison, not string comparison!
    function parseTimestamp(ts) {
        if (!ts) return 0;
        const num = parseFloat(ts);
        return isNaN(num) ? 0 : num;
    }

    // Find the latest timestamp
    let latestTimestamp = lastLogTimestamp;
    let hasNewLogs = false;

    logs.forEach(log => {
        const logKey = `${log.timestamp}-${log.level}-${log.message}`;
        if (!logEntriesMap.has(logKey)) {
            hasNewLogs = true;
            logEntriesMap.set(logKey, log);
            const logTs = parseTimestamp(log.timestamp);
            const latestTs = latestTimestamp ? parseTimestamp(latestTimestamp) : 0;
            if (logTs > latestTs) {
                latestTimestamp = log.timestamp;
            }
        }
    });

    // Only update if there are new logs
    if (hasNewLogs) {
        // Sort logs by timestamp (numeric comparison)
        const sortedLogs = Array.from(logEntriesMap.values()).sort((a, b) => {
            const tsA = parseTimestamp(a.timestamp);
            const tsB = parseTimestamp(b.timestamp);
            return tsA - tsB;
        });

        // Keep only last 200 logs
        if (sortedLogs.length > 200) {
            const toRemove = sortedLogs.slice(0, sortedLogs.length - 200);
            toRemove.forEach(log => {
                const logKey = `${log.timestamp}-${log.level}-${log.message}`;
                logEntriesMap.delete(logKey);
            });
        }

        // Render all logs in chronological order
        let logsHtml = '';
        Array.from(logEntriesMap.values())
            .sort((a, b) => {
                const tsA = parseTimestamp(a.timestamp);
                const tsB = parseTimestamp(b.timestamp);
                return tsA - tsB;
            })
            .forEach(log => {
                const levelClass = `log-${log.level.toLowerCase()}`;
                logsHtml += `<div class="log-entry ${levelClass}">[${log.timestamp}] [${log.level}] ${log.message}</div>`;
            });

        const wasAtBottom = container.scrollHeight - container.scrollTop <= container.clientHeight + 10;
        container.innerHTML = logsHtml;
        
        // Auto-scroll to bottom only if user was already at bottom
        if (wasAtBottom) {
            container.scrollTop = container.scrollHeight;
        }
        
        lastLogTimestamp = latestTimestamp;
    }
}

function setupAutoRefreshLogs() {
    const checkbox = document.getElementById('auto-refresh-logs');
    checkbox.addEventListener('change', (e) => {
        if (e.target.checked) {
            startAutoRefreshLogs();
        } else {
            stopAutoRefreshLogs();
        }
    });
    if (checkbox.checked) {
        startAutoRefreshLogs();
    }
}

function startAutoRefreshLogs() {
    if (autoRefreshLogsInterval) {
        clearInterval(autoRefreshLogsInterval);
    }
    autoRefreshLogsInterval = setInterval(() => {
        loadLogs();
    }, 2000);
}

function stopAutoRefreshLogs() {
    if (autoRefreshLogsInterval) {
        clearInterval(autoRefreshLogsInterval);
        autoRefreshLogsInterval = null;
    }
}

// Initialize on page load
loadStatus();
loadTfTree();
loadTopics();
loadLogs();
setupAutoRefresh();
setupAutoRefreshLogs();
