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
        topicsHtml += `<div class="topic-entry">`;
        topicsHtml += `<span class="topic-name">${topic.name}</span>`;
        topicsHtml += `<span class="topic-type">${topic.message_type}</span>`;
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

// Components loading
let autoRefreshComponentsInterval = null;

async function loadComponents() {
    await Promise.all([
        loadPrimitives(),
        loadServices(),
        loadSkills(),
        loadTasks(),
    ]);
}

async function loadPrimitives() {
    const container = document.getElementById('primitives-container');
    try {
        const response = await fetch('/api/primitives');
        const data = await response.json();
        renderPrimitives(data.primitives || []);
    } catch (error) {
        console.error('Failed to load primitives:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

// Store component data for modal display
let primitivesData = [];
let servicesData = [];
let skillsData = [];
let tasksData = [];

function renderPrimitives(primitives) {
    const container = document.getElementById('primitives-container');
    primitivesData = primitives; // Store for modal

    if (primitives.length === 0) {
        container.innerHTML = '<div class="component-item">No primitives registered</div>';
        return;
    }

    let html = '';
    primitives.forEach((prim, index) => {
        // Key format is "name$provider$version", so we need to extract name properly
        let name = prim.name;
        if (!name && prim.key) {
            // Extract name from key
            // Key format: "prm::camera.rgb$provider$version"
            // We want: "prm::camera.rgb"
            const keyParts = prim.key.split('$');
            if (keyParts.length >= 3) {
                // Key has at least 3 parts: name (may contain ::), provider, version
                // Join all parts except the last two (provider and version)
                name = keyParts.slice(0, keyParts.length - 2).join('$');
            } else {
                name = keyParts[0] || prim.key;
            }
        }
        name = name || prim.key || 'Unknown';
        html += `<div class="component-item" onclick="showComponentModal('primitive', ${index})">`;
        html += `<div class="component-item-name">${escapeHtml(name)}</div>`;
        html += `<div class="component-item-meta">Provider: ${escapeHtml(prim.provider)} | Version: ${escapeHtml(prim.version)}</div>`;
        html += `</div>`;
    });
    container.innerHTML = html;
}

// Component List Modal (for entire card)
let currentListModalType = null;
let currentListModalView = 'list'; // 'list' or 'detail'

function showComponentListModal(type) {
    const modal = document.getElementById('component-list-modal');
    const title = document.getElementById('component-list-modal-title');
    const content = document.getElementById('component-list-modal-content');

    if (!modal || !title || !content) {
        console.error('Modal elements not found');
        return;
    }

    currentListModalType = type;
    currentListModalView = 'list';

    let data = [];
    let titleText = '';

    switch (type) {
        case 'primitive':
            data = primitivesData;
            titleText = 'Primitives';
            break;
        case 'service':
            data = servicesData;
            titleText = 'Services';
            break;
        case 'skill':
            data = skillsData;
            titleText = 'Skills';
            break;
        case 'task':
            data = tasksData;
            titleText = 'Tasks';
            break;
    }

    title.textContent = titleText;

    // Render list view
    let html = '';

    if (data.length === 0) {
        html = '<div class="component-list-empty">No items registered</div>';
    } else {
        html = '<div class="component-list-grid">';
        data.forEach((item, index) => {
            html += renderComponentListItem(type, item, index);
        });
        html += '</div>';
    }

    content.innerHTML = html;
    modal.style.display = 'block';
    // Trigger animation
    setTimeout(() => {
        modal.classList.add('show');
    }, 10);
}

function renderComponentListItem(type, item, index) {
    let itemClass = 'component-list-item';

    // Add state class for tasks
    if (type === 'task') {
        const stateClass = `task-state-${item.state.toLowerCase()}`;
        itemClass += ` ${stateClass}`;
    }

    let html = `<div class="${itemClass}" onclick="showComponentDetailInListModal('${type}', ${index})">`;

    if (type === 'primitive') {
        let name = item.name;
        if (!name && item.key) {
            const keyParts = item.key.split('$');
            name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || item.key);
        }
        name = name || item.key || 'Unknown';
        html += `<div class="component-list-item-name">${escapeHtml(name)}</div>`;
        html += `<div class="component-list-item-meta">Provider: ${escapeHtml(item.provider)} | Version: ${escapeHtml(item.version)}</div>`;
    } else if (type === 'service') {
        let name = item.name;
        if (!name && item.key) {
            const keyParts = item.key.split('$');
            name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || item.key);
        }
        name = name || item.key || 'Unknown';
        let metadata = {};
        try {
            metadata = JSON.parse(item.metadata || '{}');
        } catch (e) { }
        const status = metadata.status || 'unknown';
        html += `<div class="component-list-item-name">${escapeHtml(name)}</div>`;
        html += `<div class="component-list-item-meta">Status: ${escapeHtml(status)} | Provider: ${escapeHtml(item.provider)}</div>`;
    } else if (type === 'skill') {
        html += `<div class="component-list-item-name">${escapeHtml(item.name)}</div>`;
        html += `<div class="component-list-item-meta">Type: ${escapeHtml(item.type)} | Provider: ${escapeHtml(item.provider)}</div>`;
    } else if (type === 'task') {
        html += `<div class="component-list-item-name">${escapeHtml(item.task_id)}</div>`;
        html += `<div class="component-list-item-meta">${escapeHtml(item.description)} | State: ${item.state}</div>`;
    }

    html += '<div class="component-list-item-arrow">→</div>';
    html += '</div>';
    return html;
}

function showComponentDetailInListModal(type, index) {
    const content = document.getElementById('component-list-modal-content');
    const title = document.getElementById('component-list-modal-title');

    currentListModalView = 'detail';

    let data = null;
    let titleText = '';

    switch (type) {
        case 'primitive':
            if (index >= 0 && index < primitivesData.length) {
                data = primitivesData[index];
                let name = data.name;
                if (!name && data.key) {
                    const keyParts = data.key.split('$');
                    name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || data.key);
                }
                titleText = `Primitive: ${name || data.key}`;
            }
            break;
        case 'service':
            if (index >= 0 && index < servicesData.length) {
                data = servicesData[index];
                let name = data.name;
                if (!name && data.key) {
                    const keyParts = data.key.split('$');
                    name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || data.key);
                }
                titleText = `Service: ${name || data.key}`;
            }
            break;
        case 'skill':
            if (index >= 0 && index < skillsData.length) {
                data = skillsData[index];
                titleText = `Skill: ${data.name}`;
            }
            break;
        case 'task':
            if (index >= 0 && index < tasksData.length) {
                data = tasksData[index];
                titleText = `Task: ${data.task_id}`;
            }
            break;
    }

    if (!data) {
        console.error('Data not found for type:', type, 'index:', index);
        return;
    }

    title.textContent = titleText;

    let html = '<div class="component-detail-back" onclick="backToComponentList()">← Back to List</div>';
    html += renderComponentItemDetail(type, data, index);

    content.innerHTML = html;
}

function backToComponentList() {
    if (!currentListModalType) return;
    showComponentListModal(currentListModalType);
}

function renderComponentItemDetail(type, item, index) {
    let html = '<div class="component-detail-section">';

    if (type === 'primitive') {
        let name = item.name;
        if (!name && item.key) {
            const keyParts = item.key.split('$');
            name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || item.key);
        }
        name = name || item.key || 'Unknown';

        html += `<h3>${escapeHtml(name)}</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(item.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(item.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Key:</span><span class="component-detail-value">${escapeHtml(item.key)}</span></div>`;

        html += `<div class="component-detail-field"><span class="component-detail-label">Input Schema:</span></div>`;
        try {
            const inputSchema = JSON.parse(item.input_schema || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(inputSchema, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(item.input_schema || 'N/A')}</div>`;
        }

        html += `<div class="component-detail-field"><span class="component-detail-label">Output Schema:</span></div>`;
        try {
            const outputSchema = JSON.parse(item.output_schema || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(outputSchema, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(item.output_schema || 'N/A')}</div>`;
        }

        html += `<div class="component-detail-field"><span class="component-detail-label">Metadata:</span></div>`;
        try {
            const metadata = JSON.parse(item.metadata || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(item.metadata || 'N/A')}</div>`;
        }
    } else if (type === 'service') {
        let name = item.name;
        if (!name && item.key) {
            const keyParts = item.key.split('$');
            name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || item.key);
        }
        name = name || item.key || 'Unknown';

        let metadata = {};
        try {
            metadata = JSON.parse(item.metadata || '{}');
        } catch (e) { }
        const status = metadata.status || 'unknown';

        html += `<h3>${escapeHtml(name)}</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Status:</span><span class="component-detail-value">${escapeHtml(status)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(item.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(item.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Entry:</span><span class="component-detail-value">${escapeHtml(item.entry)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Key:</span><span class="component-detail-value">${escapeHtml(item.key)}</span></div>`;

        html += `<div class="component-detail-field"><span class="component-detail-label">Metadata:</span></div>`;
        html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
    } else if (type === 'skill') {
        html += `<h3>${escapeHtml(item.name)}</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Skill ID:</span><span class="component-detail-value">${escapeHtml(item.skill_id || 'N/A')}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Type:</span><span class="component-detail-value">${escapeHtml(item.type)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(item.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(item.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Start Topic:</span><span class="component-detail-value">${escapeHtml(item.start_topic || 'N/A')}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Status Topic:</span><span class="component-detail-value">${escapeHtml(item.status_topic || 'N/A')}</span></div>`;
        if (item.skill_dir) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Skill Dir:</span><span class="component-detail-value">${escapeHtml(item.skill_dir)}</span></div>`;
        }
        if (item.main_rtdl) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Main RTDL:</span><span class="component-detail-value">${escapeHtml(item.main_rtdl)}</span></div>`;
        }
        if (item.start_args) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Start Args:</span></div>`;
            try {
                const startArgs = JSON.parse(item.start_args || '{}');
                html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(startArgs, null, 2))}</div>`;
            } catch (e) {
                html += `<div class="component-detail-value">${escapeHtml(item.start_args)}</div>`;
            }
        }
        if (item.metadata) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Metadata:</span></div>`;
            try {
                const metadata = JSON.parse(item.metadata || '{}');
                html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
            } catch (e) {
                html += `<div class="component-detail-value">${escapeHtml(item.metadata)}</div>`;
            }
        }
    } else if (type === 'task') {
        const stateClass = `task-state-${item.state.toLowerCase()}`;
        html += `<h3 class="${stateClass}">${escapeHtml(item.task_id)}</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Description:</span><span class="component-detail-value">${escapeHtml(item.description)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">State:</span><span class="component-detail-value">${escapeHtml(item.state)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Priority:</span><span class="component-detail-value">${item.priority}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Instruction Pointer:</span><span class="component-detail-value">${item.rtdl_instruction_pointer || 0}</span></div>`;
        if (item.rtdl) {
            html += `<div class="component-detail-field"><span class="component-detail-label">RTDL Type:</span><span class="component-detail-value">${escapeHtml(item.rtdl_type || 'N/A')}</span></div>`;
            html += `<div class="component-detail-field"><span class="component-detail-label">RTDL Program:</span></div>`;
            html += `<div class="component-detail-json">${escapeHtml(item.rtdl)}</div>`;
        }
        if (item.params) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Parameters:</span></div>`;
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(item.params, null, 2))}</div>`;
        }
        if (item.result) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Result:</span></div>`;
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(item.result, null, 2))}</div>`;
        }
        if (item.error_message) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Error:</span></div>`;
            html += `<div class="component-detail-value" style="color: #d32f2f;">${escapeHtml(item.error_message)}</div>`;
        }
    }

    html += '</div>';
    return html;
}

function closeComponentListModal() {
    const modal = document.getElementById('component-list-modal');
    if (modal) {
        modal.classList.remove('show');
        // Wait for animation to complete before hiding
        setTimeout(() => {
            modal.style.display = 'none';
        }, 300);
    }
}

async function loadServices() {
    const container = document.getElementById('services-container');
    try {
        const response = await fetch('/api/services');
        const data = await response.json();
        renderServices(data.services || []);
    } catch (error) {
        console.error('Failed to load services:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

function renderServices(services) {
    const container = document.getElementById('services-container');
    servicesData = services; // Store for modal

    // Update status bar
    const servicesCountEl = document.getElementById('services-count');
    if (servicesCountEl) {
        servicesCountEl.textContent = services.length;
    }

    if (services.length === 0) {
        container.innerHTML = '<div class="component-item">No services registered</div>';
        return;
    }

    let html = '';
    services.forEach((srv, index) => {
        // Key format is "name$provider$version"
        let name = srv.name;
        if (!name && srv.key) {
            const keyParts = srv.key.split('$');
            if (keyParts.length >= 3) {
                name = keyParts.slice(0, keyParts.length - 2).join('$');
            } else {
                name = keyParts[0] || srv.key;
            }
        }
        name = name || srv.key || 'Unknown';
        let metadata = {};
        try {
            metadata = JSON.parse(srv.metadata || '{}');
        } catch (e) {
            // Ignore parse errors
        }
        const status = metadata.status || 'unknown';
        html += `<div class="component-item" onclick="showComponentModal('service', ${index})">`;
        html += `<div class="component-item-name">${escapeHtml(name)}</div>`;
        html += `<div class="component-item-meta">Status: ${escapeHtml(status)} | Provider: ${escapeHtml(srv.provider)}</div>`;
        html += `</div>`;
    });
    container.innerHTML = html;
}

async function loadSkills() {
    const container = document.getElementById('skills-container');
    try {
        const response = await fetch('/api/skills');
        const data = await response.json();
        renderSkills(data.skills || []);
    } catch (error) {
        console.error('Failed to load skills:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

function renderSkills(skills) {
    const container = document.getElementById('skills-container');
    skillsData = skills; // Store for modal

    // Update status bar
    const skillsCountEl = document.getElementById('skills-count');
    if (skillsCountEl) {
        skillsCountEl.textContent = skills.length;
    }

    if (skills.length === 0) {
        container.innerHTML = '<div class="component-item">No skills registered</div>';
        return;
    }

    let html = '';
    skills.forEach((skill, index) => {
        html += `<div class="component-item" onclick="showComponentModal('skill', ${index})">`;
        html += `<div class="component-item-name">${escapeHtml(skill.name)}</div>`;
        html += `<div class="component-item-meta">Type: ${escapeHtml(skill.type)} | Provider: ${escapeHtml(skill.provider)}</div>`;
        html += `</div>`;
    });
    container.innerHTML = html;
}

async function loadTasks() {
    const container = document.getElementById('tasks-container');
    try {
        const response = await fetch('/api/tasks');
        const data = await response.json();
        renderTasks(data.tasks || []);
    } catch (error) {
        console.error('Failed to load tasks:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

function renderTasks(tasks) {
    const container = document.getElementById('tasks-container');
    tasksData = tasks; // Store for modal

    // Update status bar - count active tasks (not finished, failed, or cancelled)
    const activeTasks = tasks.filter(task => {
        const state = task.state.toLowerCase();
        return state !== 'finished' && state !== 'failed' && state !== 'cancelled';
    });
    const activeTasksEl = document.getElementById('active-tasks');
    if (activeTasksEl) {
        activeTasksEl.textContent = activeTasks.length;
    }

    if (tasks.length === 0) {
        container.innerHTML = '<div class="component-item">No tasks</div>';
        return;
    }

    let html = '';
    tasks.forEach((task, index) => {
        const stateClass = `task-state-${task.state.toLowerCase()}`;
        html += `<div class="component-item ${stateClass}" onclick="showComponentModal('task', ${index})">`;
        html += `<div class="task-description">${escapeHtml(task.description)}</div>`;
        html += `<div class="component-item-meta">State: ${task.state} | Priority: ${task.priority}</div>`;
        if (task.rtdl) {
            html += `<div class="task-rtdl-link" onclick="event.stopPropagation(); showRtdlModal('${escapeHtml(task.rtdl)}')">View RTDL</div>`;
        }
        html += `</div>`;
    });
    container.innerHTML = html;
}

function escapeHtml(text) {
    const div = document.createElement('div');
    div.textContent = text;
    return div.innerHTML;
}

// Remove markdown formatting from text
function stripMarkdown(text) {
    if (!text) return '';
    return text
        // Remove code blocks
        .replace(/```[\s\S]*?```/g, '')
        // Remove inline code
        .replace(/`([^`]+)`/g, '$1')
        // Remove bold/italic
        .replace(/\*\*([^*]+)\*\*/g, '$1')
        .replace(/\*([^*]+)\*/g, '$1')
        .replace(/__([^_]+)__/g, '$1')
        .replace(/_([^_]+)_/g, '$1')
        // Remove headers
        .replace(/^#{1,6}\s+(.+)$/gm, '$1')
        // Remove links
        .replace(/\[([^\]]+)\]\([^\)]+\)/g, '$1')
        // Remove images
        .replace(/!\[([^\]]*)\]\([^\)]+\)/g, '$1')
        // Remove horizontal rules
        .replace(/^---$/gm, '')
        // Remove list markers
        .replace(/^[\*\-\+]\s+/gm, '')
        .replace(/^\d+\.\s+/gm, '')
        // Clean up extra whitespace
        .replace(/\n{3,}/g, '\n\n')
        .trim();
}

// Visualization loading
let autoRefreshVizInterval = null;

async function loadVisualization() {
    await Promise.all([
        loadSemanticMap(),
        loadImageMonitor(),
        loadSemanticMap2D(),
    ]);
}

async function loadSemanticMap() {
    const container = document.getElementById('semantic-map-container');
    try {
        const response = await fetch('/api/semantic-map');
        const data = await response.json();
        renderSemanticMap(data.objects || []);
    } catch (error) {
        console.error('Failed to load semantic map:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

function renderSemanticMap(objects) {
    const container = document.getElementById('semantic-map-container');
    if (!Array.isArray(objects) || objects.length === 0) {
        container.innerHTML = '<div class="semantic-object">No objects detected</div>';
        return;
    }

    let html = '';
    objects.forEach((obj, idx) => {
        html += `<div class="semantic-object">`;
        html += `<div class="object-label">${obj.label || `Object ${idx + 1}`}</div>`;
        html += `<div class="object-meta">ID: ${obj.id || 'N/A'}</div>`;
        if (obj.frame_mapping && obj.frame_mapping.length > 0) {
            const fm = obj.frame_mapping[0];
            if (fm.center) {
                html += `<div class="object-meta">Position: [${fm.center.x.toFixed(2)}, ${fm.center.y.toFixed(2)}, ${fm.center.z.toFixed(2)}]</div>`;
            }
        }
        html += `</div>`;
    });
    container.innerHTML = html;
}

// Semantic Map 2D View
let autoRefreshMap2DInterval = null;
let map2DState = {
    scale: 1.0,
    offsetX: 0,
    offsetY: 0,
    minX: 0,
    maxX: 0,
    minY: 0,
    maxY: 0,
    isDragging: false,
    dragStartX: 0,
    dragStartY: 0,
    dragStartOffsetX: 0,
    dragStartOffsetY: 0,
};

async function loadSemanticMap2D() {
    try {
        const response = await fetch('/api/semantic-map');
        const data = await response.json();
        renderSemanticMap2D(data.objects || []);
    } catch (error) {
        console.error('Failed to load semantic map 2D:', error);
    }
}

function renderSemanticMap2D(objects) {
    const canvas = document.getElementById('semantic-map-2d-canvas');
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Set canvas size
    const container = canvas.parentElement;
    const containerWidth = container.clientWidth || container.offsetWidth || 800;
    const containerHeight = container.clientHeight || container.offsetHeight || 400;

    // Support high DPI displays (Retina)
    const dpr = window.devicePixelRatio || 1;
    const displayWidth = containerWidth - 20;
    const displayHeight = Math.max(380, containerHeight - 20);

    // Set canvas CSS size
    canvas.style.width = displayWidth + 'px';
    canvas.style.height = displayHeight + 'px';

    // Set canvas actual size (scaled for high DPI)
    canvas.width = displayWidth * dpr;
    canvas.height = displayHeight * dpr;

    // Scale context to match device pixel ratio
    ctx.scale(dpr, dpr);

    // Clear canvas
    ctx.clearRect(0, 0, displayWidth, displayHeight);

    if (!Array.isArray(objects) || objects.length === 0) {
        ctx.fillStyle = '#999';
        ctx.font = '16px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('No objects detected', displayWidth / 2, displayHeight / 2);
        return;
    }

    // Extract objects with map frame coordinates
    const mapObjects = [];
    objects.forEach(obj => {
        if (!obj.frame_mapping) return;

        // Find map frame mapping
        let mapFrame = null;
        for (const fm of obj.frame_mapping) {
            if (fm.frame_id === 'map' && fm.center) {
                mapFrame = fm;
                break;
            }
        }

        if (mapFrame) {
            // Extract yaw from bbox if available
            let yaw = 0.0;
            if (mapFrame.bbox && mapFrame.bbox.length > 0 && mapFrame.bbox[0].yaw !== undefined) {
                yaw = mapFrame.bbox[0].yaw;
            }

            mapObjects.push({
                id: obj.id,
                label: obj.label,
                x: mapFrame.center.x,
                y: mapFrame.center.y,
                z: mapFrame.center.z,
                yaw: yaw,
                isRobot: obj.id === 'robot_self' || obj.label === 'robot',
            });
        }
    });

    if (mapObjects.length === 0) {
        ctx.fillStyle = '#999';
        ctx.font = '16px Arial';
        ctx.textAlign = 'center';
        ctx.fillText('No objects with map coordinates', displayWidth / 2, displayHeight / 2);
        return;
    }

    // Calculate bounds
    let minX = Infinity, maxX = -Infinity;
    let minY = Infinity, maxY = -Infinity;

    mapObjects.forEach(obj => {
        minX = Math.min(minX, obj.x);
        maxX = Math.max(maxX, obj.x);
        minY = Math.min(minY, obj.y);
        maxY = Math.max(maxY, obj.y);
    });

    // Add padding
    const padding = 2.0; // meters
    minX -= padding;
    maxX += padding;
    minY -= padding;
    maxY += padding;

    // Update map2DState bounds
    map2DState.minX = minX;
    map2DState.maxX = maxX;
    map2DState.minY = minY;
    map2DState.maxY = maxY;

    // Calculate initial scale to fit all objects (only if scale is 1.0, i.e., first render)
    const rangeX = maxX - minX;
    const rangeY = maxY - minY;
    if (map2DState.scale === 1.0 && rangeX > 0 && rangeY > 0) {
        const scaleX = (displayWidth - 40) / rangeX;
        const scaleY = (displayHeight - 40) / rangeY;
        map2DState.scale = Math.min(scaleX, scaleY);
    }

    const scale = map2DState.scale;
    const offsetX = map2DState.offsetX;
    const offsetY = map2DState.offsetY;

    // Helper function to convert world coordinates to canvas coordinates
    const worldToCanvas = (wx, wy) => {
        const cx = 20 + (wx - minX) * scale + offsetX;
        const cy = displayHeight - 20 - (wy - minY) * scale - offsetY; // Flip Y axis
        return { x: cx, y: cy };
    };

    // Draw grid - cover entire canvas
    ctx.strokeStyle = '#e0e0e0';
    ctx.lineWidth = 0.5;

    const gridStep = Math.max(1.0, Math.ceil(Math.max(rangeX, rangeY) / 10));

    // Calculate world coordinates for canvas edges (considering offset)
    const canvasToWorld = (cx, cy) => {
        const wx = (cx - 20 - offsetX) / scale + minX;
        const wy = (displayHeight - 20 - cy - offsetY) / scale + minY; // Flip Y axis
        return { x: wx, y: wy };
    };

    // Get world bounds for visible area (considering offset and scale)
    const topLeft = canvasToWorld(0, 0);
    const bottomRight = canvasToWorld(displayWidth, displayHeight);
    const visibleMinX = Math.min(topLeft.x, bottomRight.x);
    const visibleMaxX = Math.max(topLeft.x, bottomRight.x);
    const visibleMinY = Math.min(topLeft.y, bottomRight.y);
    const visibleMaxY = Math.max(topLeft.y, bottomRight.y);

    // Draw vertical grid lines - extend beyond visible area to ensure full coverage
    const gridStartX = Math.floor(visibleMinX / gridStep) * gridStep - gridStep * 2;
    const gridEndX = Math.ceil(visibleMaxX / gridStep) * gridStep + gridStep * 2;
    for (let x = gridStartX; x <= gridEndX; x += gridStep) {
        const pos = worldToCanvas(x, 0);
        // Draw line from top to bottom of canvas
        ctx.beginPath();
        ctx.moveTo(pos.x, 0);
        ctx.lineTo(pos.x, displayHeight);
        ctx.stroke();
    }

    // Draw horizontal grid lines - extend beyond visible area to ensure full coverage
    const gridStartY = Math.floor(visibleMinY / gridStep) * gridStep - gridStep * 2;
    const gridEndY = Math.ceil(visibleMaxY / gridStep) * gridStep + gridStep * 2;
    for (let y = gridStartY; y <= gridEndY; y += gridStep) {
        const pos = worldToCanvas(0, y);
        // Draw line from left to right of canvas
        ctx.beginPath();
        ctx.moveTo(0, pos.y);
        ctx.lineTo(displayWidth, pos.y);
        ctx.stroke();
    }

    // Draw coordinate axes
    ctx.strokeStyle = '#999';
    ctx.lineWidth = 1;
    const origin = worldToCanvas(0, 0);
    ctx.beginPath();
    ctx.moveTo(origin.x, 0);
    ctx.lineTo(origin.x, displayHeight);
    ctx.stroke();
    ctx.beginPath();
    ctx.moveTo(0, origin.y);
    ctx.lineTo(displayWidth, origin.y);
    ctx.stroke();

    // Draw origin marker
    ctx.fillStyle = '#666';
    ctx.fillText('(0,0)', origin.x + 5, origin.y - 5);

    // Simple drawing - no collision avoidance
    mapObjects.forEach((obj) => {
        const pos = worldToCanvas(obj.x, obj.y);
        const labelText = obj.label || (obj.isRobot ? 'robot' : 'object');
        const coordText = `[${obj.x.toFixed(1)}, ${obj.y.toFixed(1)}]`;

        // Draw object as a circle (including robot)
        if (obj.isRobot) {
            ctx.fillStyle = '#1976d2';
            ctx.strokeStyle = '#0d47a1';
        } else {
            ctx.fillStyle = '#4caf50';
            ctx.strokeStyle = '#2e7d32';
        }
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(pos.x, pos.y, 6, 0, 2 * Math.PI);
        ctx.fill();
        ctx.stroke();

        // Draw label
        ctx.fillStyle = obj.isRobot ? '#1976d2' : '#333';
        ctx.font = obj.isRobot ? 'bold 12px Arial' : '11px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(labelText, pos.x, pos.y - 10);

        // Draw coordinates
        ctx.fillStyle = '#666';
        ctx.font = '9px Arial';
        ctx.textAlign = 'center';
        ctx.fillText(coordText, pos.x, pos.y + 20);
    });

    // Draw legend
    ctx.fillStyle = '#333';
    ctx.font = '11px Arial';
    ctx.textAlign = 'left';
    ctx.fillText('Scale: ' + scale.toFixed(2) + ' px/m', 10, 20);
    ctx.fillText('Objects: ' + mapObjects.length, 10, 35);
}

async function loadImageMonitor() {
    const container = document.getElementById('image-monitor-container');
    try {
        const response = await fetch('/api/image-topics');
        const data = await response.json();
        renderImageMonitor(data.image_topics || []);
        // Hide loading text if we have data
        if (container.textContent === 'Loading...' && (data.image_topics || []).length > 0) {
            container.textContent = '';
        }
    } catch (error) {
        console.error('Failed to load image monitor:', error);
        container.innerHTML = `Error: ${error.message}`;
    }
}

let lastImageMonitorData = new Map(); // Map<topicName, Set<timestamp>>

// Image monitor state management
let imageMonitorState = {
    topics: new Map(), // Map<topicName, {url, timestamp}>
    updateTopic(topicName, imagePath, timestamp) {
        if (imagePath) {
            const pathParts = imagePath.split('/');
            const filename = pathParts[pathParts.length - 1];
            const url = `/api/images/${filename}?t=${timestamp}`;
            this.topics.set(topicName, { url, timestamp });
        } else {
            this.topics.delete(topicName);
        }
    },
    getTopicUrl(topicName) {
        return this.topics.get(topicName)?.url || '';
    },
    getTopicTimestamp(topicName) {
        const topic = this.topics.get(topicName);
        if (!topic) return null;
        const date = new Date(topic.timestamp * 1000);
        const hours = String(date.getHours()).padStart(2, '0');
        const minutes = String(date.getMinutes()).padStart(2, '0');
        const seconds = String(date.getSeconds()).padStart(2, '0');
        return `${hours}:${minutes}:${seconds}`;
    }
};

function renderImageMonitor(imageTopics) {
    const container = document.getElementById('image-monitor-container');

    // Clear loading text if present
    if (container.textContent.trim() === 'Loading...') {
        container.textContent = '';
    }

    if (imageTopics.length === 0) {
        return;
    }

    // Update state
    imageTopics.forEach(topic => {
        const topicName = topic.topic_name;
        const imagePaths = topic.image_paths || [];
        const latestImage = imagePaths.length > 0 ? imagePaths[0] : null;

        if (latestImage) {
            imageMonitorState.updateTopic(topicName, latestImage.path, latestImage.timestamp);
        } else {
            imageMonitorState.updateTopic(topicName, null, null);
        }
    });

    // Render using simple DOM updates
    imageTopics.forEach(topic => {
        const topicName = topic.topic_name;
        const imagePaths = topic.image_paths || [];
        const latestImage = imagePaths.length > 0 ? imagePaths[0] : null;

        // Find or create topic entry
        let topicEntry = container.querySelector(`[data-topic="${CSS.escape(topicName)}"]`);
        if (!topicEntry) {
            topicEntry = document.createElement('div');
            topicEntry.className = 'image-topic-entry';
            topicEntry.setAttribute('data-topic', topicName);
            container.appendChild(topicEntry);
        }

        // Update topic name (shorten if too long)
        let nameEl = topicEntry.querySelector('.image-topic-name');
        if (!nameEl) {
            nameEl = document.createElement('div');
            nameEl.className = 'image-topic-name';
            topicEntry.insertBefore(nameEl, topicEntry.firstChild);
        }
        // Display full topic name
        nameEl.textContent = topicName;
        nameEl.title = topicName;

        // Update gallery
        let gallery = topicEntry.querySelector('.image-gallery');
        if (!gallery) {
            gallery = document.createElement('div');
            gallery.className = 'image-gallery';
            topicEntry.appendChild(gallery);
        }

        let item = gallery.querySelector('.image-gallery-item-single');
        if (!item) {
            item = document.createElement('div');
            item.className = 'image-gallery-item image-gallery-item-single';
            gallery.appendChild(item);
        }

        let img = item.querySelector('img');
        if (!img) {
            img = document.createElement('img');
            img.className = 'image-preview-single';
            img.alt = topicName;
            item.insertBefore(img, item.firstChild);
        }

        let timestampEl = item.querySelector('.image-timestamp');
        if (!timestampEl) {
            timestampEl = document.createElement('div');
            timestampEl.className = 'image-timestamp';
            item.appendChild(timestampEl);
        }

        if (latestImage) {
            const url = imageMonitorState.getTopicUrl(topicName);
            const timeStr = imageMonitorState.getTopicTimestamp(topicName);

            // Only update src if changed
            if (img.src !== url && url) {
                img.src = url;
                img.style.display = 'block';
            }

            if (timestampEl && timeStr) {
                timestampEl.textContent = timeStr;
                timestampEl.title = timeStr;
            }
        } else {
            img.style.display = 'none';
            if (timestampEl) {
                timestampEl.textContent = '--:--:--';
            }
        }
    });

    // Remove topics that no longer exist
    const currentTopicNames = new Set(imageTopics.map(t => t.topic_name));
    const existingEntries = container.querySelectorAll('.image-topic-entry');
    existingEntries.forEach(entry => {
        const topicName = entry.getAttribute('data-topic');
        if (topicName && !currentTopicNames.has(topicName)) {
            entry.remove();
            imageMonitorState.topics.delete(topicName);
        }
    });
}

// RTDL Modal
function showRtdlModal(rtdl) {
    const modal = document.getElementById('rtdl-modal');
    const content = document.getElementById('rtdl-content');
    content.textContent = rtdl;
    modal.style.display = 'block';
    // Trigger animation
    setTimeout(() => {
        modal.classList.add('show');
    }, 10);
}

function closeRtdlModal() {
    const modal = document.getElementById('rtdl-modal');
    modal.classList.remove('show');
    // Wait for animation to complete before hiding
    setTimeout(() => {
        modal.style.display = 'none';
    }, 300);
}

// Component Detail Modal
function showComponentModal(type, index) {
    const modal = document.getElementById('component-modal');
    const title = document.getElementById('component-modal-title');
    const content = document.getElementById('component-modal-content');

    if (!modal || !title || !content) {
        console.error('Modal elements not found');
        return;
    }

    let data = null;
    let titleText = '';

    switch (type) {
        case 'primitive':
            if (index >= 0 && index < primitivesData.length) {
                data = primitivesData[index];
                let name = data.name;
                if (!name && data.key) {
                    const keyParts = data.key.split('$');
                    name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || data.key);
                }
                titleText = `Primitive: ${name || data.key}`;
            }
            break;
        case 'service':
            if (index >= 0 && index < servicesData.length) {
                data = servicesData[index];
                let name = data.name;
                if (!name && data.key) {
                    const keyParts = data.key.split('$');
                    name = keyParts.length >= 3 ? keyParts.slice(0, keyParts.length - 2).join('$') : (keyParts[0] || data.key);
                }
                titleText = `Service: ${name || data.key}`;
            }
            break;
        case 'skill':
            if (index >= 0 && index < skillsData.length) {
                data = skillsData[index];
                titleText = `Skill: ${data.name}`;
            }
            break;
        case 'task':
            if (index >= 0 && index < tasksData.length) {
                data = tasksData[index];
                titleText = `Task: ${data.task_id}`;
            }
            break;
    }

    if (!data) {
        console.error('Data not found for type:', type, 'index:', index);
        return;
    }

    title.textContent = titleText;

    let html = '';

    if (type === 'primitive') {
        html += `<div class="component-detail-section">`;
        html += `<h3>Basic Information</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Name:</span><span class="component-detail-value">${escapeHtml(data.name || data.key)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(data.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(data.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Key:</span><span class="component-detail-value">${escapeHtml(data.key)}</span></div>`;
        html += `</div>`;

        html += `<div class="component-detail-section">`;
        html += `<h3>Input Schema</h3>`;
        try {
            const inputSchema = JSON.parse(data.input_schema || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(inputSchema, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(data.input_schema || 'N/A')}</div>`;
        }
        html += `</div>`;

        html += `<div class="component-detail-section">`;
        html += `<h3>Output Schema</h3>`;
        try {
            const outputSchema = JSON.parse(data.output_schema || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(outputSchema, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(data.output_schema || 'N/A')}</div>`;
        }
        html += `</div>`;

        html += `<div class="component-detail-section">`;
        html += `<h3>Metadata</h3>`;
        try {
            const metadata = JSON.parse(data.metadata || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(data.metadata || 'N/A')}</div>`;
        }
        html += `</div>`;
    } else if (type === 'service') {
        html += `<div class="component-detail-section">`;
        html += `<h3>Basic Information</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Name:</span><span class="component-detail-value">${escapeHtml(data.name || data.key)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(data.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(data.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Entry:</span><span class="component-detail-value">${escapeHtml(data.entry)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Key:</span><span class="component-detail-value">${escapeHtml(data.key)}</span></div>`;
        html += `</div>`;

        html += `<div class="component-detail-section">`;
        html += `<h3>Metadata</h3>`;
        try {
            const metadata = JSON.parse(data.metadata || '{}');
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
        } catch (e) {
            html += `<div class="component-detail-value">${escapeHtml(data.metadata || 'N/A')}</div>`;
        }
        html += `</div>`;
    } else if (type === 'skill') {
        html += `<div class="component-detail-section">`;
        html += `<h3>Basic Information</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Name:</span><span class="component-detail-value">${escapeHtml(data.name)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Skill ID:</span><span class="component-detail-value">${escapeHtml(data.skill_id || 'N/A')}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Type:</span><span class="component-detail-value">${escapeHtml(data.type)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Provider:</span><span class="component-detail-value">${escapeHtml(data.provider)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Version:</span><span class="component-detail-value">${escapeHtml(data.version)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Start Topic:</span><span class="component-detail-value">${escapeHtml(data.start_topic || 'N/A')}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Status Topic:</span><span class="component-detail-value">${escapeHtml(data.status_topic || 'N/A')}</span></div>`;
        if (data.skill_dir) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Skill Dir:</span><span class="component-detail-value">${escapeHtml(data.skill_dir)}</span></div>`;
        }
        if (data.main_rtdl) {
            html += `<div class="component-detail-field"><span class="component-detail-label">Main RTDL:</span><span class="component-detail-value">${escapeHtml(data.main_rtdl)}</span></div>`;
        }
        html += `</div>`;

        if (data.start_args) {
            html += `<div class="component-detail-section">`;
            html += `<h3>Start Args</h3>`;
            try {
                const startArgs = JSON.parse(data.start_args || '{}');
                html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(startArgs, null, 2))}</div>`;
            } catch (e) {
                html += `<div class="component-detail-value">${escapeHtml(data.start_args)}</div>`;
            }
            html += `</div>`;
        }

        if (data.metadata) {
            html += `<div class="component-detail-section">`;
            html += `<h3>Metadata</h3>`;
            try {
                const metadata = JSON.parse(data.metadata || '{}');
                html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(metadata, null, 2))}</div>`;
            } catch (e) {
                html += `<div class="component-detail-value">${escapeHtml(data.metadata)}</div>`;
            }
            html += `</div>`;
        }
    } else if (type === 'task') {
        html += `<div class="component-detail-section">`;
        html += `<h3>Basic Information</h3>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Task ID:</span><span class="component-detail-value">${escapeHtml(data.task_id)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Description:</span><span class="component-detail-value">${escapeHtml(data.description)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">State:</span><span class="component-detail-value">${escapeHtml(data.state)}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Priority:</span><span class="component-detail-value">${data.priority}</span></div>`;
        html += `<div class="component-detail-field"><span class="component-detail-label">Instruction Pointer:</span><span class="component-detail-value">${data.rtdl_instruction_pointer || 0}</span></div>`;
        html += `</div>`;

        if (data.rtdl) {
            html += `<div class="component-detail-section">`;
            html += `<h3>RTDL Program</h3>`;
            html += `<div class="component-detail-field"><span class="component-detail-label">Type:</span><span class="component-detail-value">${escapeHtml(data.rtdl_type || 'N/A')}</span></div>`;
            html += `<div class="component-detail-json">${escapeHtml(data.rtdl)}</div>`;
            html += `</div>`;
        }

        if (data.params) {
            html += `<div class="component-detail-section">`;
            html += `<h3>Parameters</h3>`;
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(data.params, null, 2))}</div>`;
            html += `</div>`;
        }

        if (data.result) {
            html += `<div class="component-detail-section">`;
            html += `<h3>Result</h3>`;
            html += `<div class="component-detail-json">${escapeHtml(JSON.stringify(data.result, null, 2))}</div>`;
            html += `</div>`;
        }

        if (data.error_message) {
            html += `<div class="component-detail-section">`;
            html += `<h3>Error Message</h3>`;
            html += `<div class="component-detail-value" style="color: #d32f2f;">${escapeHtml(data.error_message)}</div>`;
            html += `</div>`;
        }

        const stateLower = (data.state || '').toLowerCase();
        const canCancel = stateLower !== 'finished' && stateLower !== 'failed' && stateLower !== 'cancelled';
        if (canCancel) {
            html += `<div class="component-detail-section modal-actions">`;
            html += `<button type="button" class="btn btn-danger" onclick="event.stopPropagation(); showCancelConfirmModal(${index})">Cancel task</button>`;
            html += `</div>`;
        }
    }

    content.innerHTML = html;
    modal.style.display = 'block';
    // Trigger animation
    setTimeout(() => {
        modal.classList.add('show');
    }, 10);
}

function closeComponentModal() {
    const modal = document.getElementById('component-modal');
    modal.classList.remove('show');
    // Wait for animation to complete before hiding
    setTimeout(() => {
        modal.style.display = 'none';
    }, 300);
}

let pendingCancelTaskId = null;

function showCancelConfirmModal(taskIndex) {
    if (taskIndex < 0 || taskIndex >= tasksData.length) return;
    pendingCancelTaskId = tasksData[taskIndex].task_id;
    const msg = document.getElementById('cancel-task-confirm-message');
    if (msg) msg.textContent = 'Are you sure you want to cancel this task?';
    const modal = document.getElementById('cancel-task-confirm-modal');
    if (modal) {
        modal.style.display = 'block';
        setTimeout(() => modal.classList.add('show'), 10);
    }
    const btn = document.getElementById('cancel-task-confirm-btn');
    if (btn) btn.onclick = confirmCancelTask;
}

function closeCancelConfirmModal() {
    pendingCancelTaskId = null;
    const modal = document.getElementById('cancel-task-confirm-modal');
    if (modal) {
        modal.classList.remove('show');
        setTimeout(() => { modal.style.display = 'none'; }, 300);
    }
}

async function confirmCancelTask() {
    if (!pendingCancelTaskId) return;
    const taskId = pendingCancelTaskId;
    closeCancelConfirmModal();
    pendingCancelTaskId = null;
    try {
        const response = await fetch(`/api/tasks/${encodeURIComponent(taskId)}/cancel`, { method: 'POST' });
        const result = await response.json();
        if (result.success) {
            closeComponentModal();
            await loadTasks();
        } else {
            console.error('Cancel task failed:', result);
            await loadTasks();
        }
    } catch (err) {
        console.error('Cancel task request failed:', err);
        await loadTasks();
    }
}

// Close modal when clicking outside
window.onclick = function (event) {
    const rtdlModal = document.getElementById('rtdl-modal');
    const componentModal = document.getElementById('component-modal');
    const componentListModal = document.getElementById('component-list-modal');
    const cancelConfirmModal = document.getElementById('cancel-task-confirm-modal');
    if (event.target == rtdlModal) {
        rtdlModal.classList.remove('show');
        setTimeout(() => {
            rtdlModal.style.display = 'none';
        }, 300);
    }
    if (event.target == componentModal) {
        componentModal.classList.remove('show');
        setTimeout(() => {
            componentModal.style.display = 'none';
        }, 300);
    }
    if (event.target == componentListModal) {
        componentListModal.classList.remove('show');
        setTimeout(() => {
            componentListModal.style.display = 'none';
        }, 300);
    }
    if (event.target == cancelConfirmModal) {
        closeCancelConfirmModal();
    }
}

// Auto refresh setup
function setupAutoRefreshComponents() {
    const checkbox = document.getElementById('auto-refresh-components');
    checkbox.addEventListener('change', (e) => {
        if (e.target.checked) {
            startAutoRefreshComponents();
        } else {
            stopAutoRefreshComponents();
        }
    });
    if (checkbox.checked) {
        startAutoRefreshComponents();
    }
}

function startAutoRefreshComponents() {
    if (autoRefreshComponentsInterval) {
        clearInterval(autoRefreshComponentsInterval);
    }
    autoRefreshComponentsInterval = setInterval(() => {
        loadComponents();
    }, 2000);
}

function stopAutoRefreshComponents() {
    if (autoRefreshComponentsInterval) {
        clearInterval(autoRefreshComponentsInterval);
        autoRefreshComponentsInterval = null;
    }
}

function setupAutoRefreshViz() {
    const checkbox = document.getElementById('auto-refresh-viz');
    checkbox.addEventListener('change', (e) => {
        if (e.target.checked) {
            startAutoRefreshViz();
        } else {
            stopAutoRefreshViz();
        }
    });
    if (checkbox.checked) {
        startAutoRefreshViz();
    }
}

function setupAutoRefreshMap2D() {
    const checkbox = document.getElementById('auto-refresh-map2d');
    if (!checkbox) return;

    checkbox.addEventListener('change', (e) => {
        if (e.target.checked) {
            startAutoRefreshMap2D();
        } else {
            stopAutoRefreshMap2D();
        }
    });
    if (checkbox.checked) {
        startAutoRefreshMap2D();
    }
}

function startAutoRefreshMap2D() {
    if (autoRefreshMap2DInterval) {
        clearInterval(autoRefreshMap2DInterval);
    }
    autoRefreshMap2DInterval = setInterval(() => {
        loadSemanticMap2D();
    }, 2000);
}

function stopAutoRefreshMap2D() {
    if (autoRefreshMap2DInterval) {
        clearInterval(autoRefreshMap2DInterval);
        autoRefreshMap2DInterval = null;
    }
}

function startAutoRefreshViz() {
    if (autoRefreshVizInterval) {
        clearInterval(autoRefreshVizInterval);
    }
    // Refresh visualization more frequently (500ms for real-time updates)
    autoRefreshVizInterval = setInterval(() => {
        loadVisualization();
    }, 500);
}

function stopAutoRefreshViz() {
    if (autoRefreshVizInterval) {
        clearInterval(autoRefreshVizInterval);
        autoRefreshVizInterval = null;
    }
}

// Scroll position persistence using reliable method
// Disable browser's default scroll restoration
if ('scrollRestoration' in history) {
    history.scrollRestoration = 'manual';
}

(function () {
    'use strict';
    const SCROLL_KEY = 'robonix_scroll_pos';
    let scrollTimeout = null;
    let isRestoring = false;

    function saveScrollPosition() {
        if (isRestoring) return;
        try {
            sessionStorage.setItem(SCROLL_KEY, window.pageYOffset || window.scrollY || 0);
        } catch (e) {
            // Ignore
        }
    }

    function restoreScrollPosition() {
        try {
            const saved = sessionStorage.getItem(SCROLL_KEY);
            if (saved !== null) {
                const y = parseInt(saved, 10);
                if (!isNaN(y) && y > 0) {
                    isRestoring = true;

                    // Multiple attempts to ensure restoration
                    const attemptRestore = () => {
                        window.scrollTo(0, y);
                        document.documentElement.scrollTop = y;
                        document.body.scrollTop = y;
                    };

                    // Try immediately
                    attemptRestore();

                    // Try after a short delay
                    setTimeout(attemptRestore, 10);
                    setTimeout(attemptRestore, 50);
                    setTimeout(attemptRestore, 100);
                    setTimeout(attemptRestore, 200);

                    setTimeout(() => {
                        isRestoring = false;
                    }, 300);
                }
            }
        } catch (e) {
            isRestoring = false;
        }
    }

    // Throttled scroll save
    window.addEventListener('scroll', function () {
        if (isRestoring) return;
        if (scrollTimeout) {
            clearTimeout(scrollTimeout);
        }
        scrollTimeout = setTimeout(saveScrollPosition, 250);
    }, { passive: true });

    // Save on page unload
    window.addEventListener('beforeunload', saveScrollPosition);
    window.addEventListener('pagehide', saveScrollPosition);

    // Restore on load
    if (document.readyState === 'complete') {
        restoreScrollPosition();
    } else {
        window.addEventListener('load', restoreScrollPosition, { once: true });
        document.addEventListener('DOMContentLoaded', restoreScrollPosition, { once: true });
    }
})();

// Initialize 2D map canvas interaction
function setupMap2DInteraction() {
    const canvas = document.getElementById('semantic-map-2d-canvas');
    if (!canvas) return;

    // Mouse drag
    canvas.addEventListener('mousedown', (e) => {
        map2DState.isDragging = true;
        map2DState.dragStartX = e.clientX;
        map2DState.dragStartY = e.clientY;
        map2DState.dragStartOffsetX = map2DState.offsetX;
        map2DState.dragStartOffsetY = map2DState.offsetY;
        canvas.style.cursor = 'grabbing';
    });

    canvas.addEventListener('mousemove', (e) => {
        if (map2DState.isDragging) {
            const dx = e.clientX - map2DState.dragStartX;
            const dy = e.clientY - map2DState.dragStartY;
            map2DState.offsetX = map2DState.dragStartOffsetX + dx;
            map2DState.offsetY = map2DState.dragStartOffsetY - dy; // Flip Y
            loadSemanticMap2D(); // Re-render
        }
    });

    canvas.addEventListener('mouseup', () => {
        map2DState.isDragging = false;
        canvas.style.cursor = 'crosshair';
    });

    canvas.addEventListener('mouseleave', () => {
        map2DState.isDragging = false;
        canvas.style.cursor = 'crosshair';
    });
}

// Agent card toggle
function toggleAgentChat() {
    const card = document.querySelector('.agent-card');
    if (card) {
        card.classList.toggle('expanded');
    }
}

// Voice input state
let isRecording = false;
let mediaRecorder = null;
let audioStream = null;
let audioChunks = [];
let currentUtterance = null;

// Initialize MediaRecorder for audio recording
async function initAudioRecording() {
    try {
        // Stop existing stream if any
        if (audioStream) {
            audioStream.getTracks().forEach(track => track.stop());
            audioStream = null;
        }
        
        // Get new stream
        audioStream = await navigator.mediaDevices.getUserMedia({ 
            audio: {
                sampleRate: 16000,
                channelCount: 1,
                echoCancellation: true,
                noiseSuppression: true
            }
        });
        
        // Use webm format (will convert to WAV later)
        const options = {
            mimeType: 'audio/webm;codecs=opus',
            audioBitsPerSecond: 16000
        };
        
        // Check if mimeType is supported
        if (!MediaRecorder.isTypeSupported(options.mimeType)) {
            console.warn('WebM with Opus not supported, using default');
            options.mimeType = '';
        }
        
        mediaRecorder = new MediaRecorder(audioStream, options);
        
        mediaRecorder.ondataavailable = function(event) {
            if (event.data.size > 0) {
                audioChunks.push(event.data);
            }
        };
        
        mediaRecorder.onstop = async function() {
            // Don't stop stream here, we'll reuse it
            const audioBlob = new Blob(audioChunks, { type: 'audio/webm' });
            audioChunks = [];
            
            // Convert to WAV format for Aliyun STT
            try {
                const wavBlob = await convertToWav(audioBlob);
                console.log('Audio converted to WAV:', {
                    originalSize: audioBlob.size,
                    wavSize: wavBlob.size,
                    originalType: audioBlob.type,
                    wavType: wavBlob.type
                });
                
                // Send to backend (backend will save both original and converted files)
                await sendAudioToSTT(wavBlob, audioBlob);
            } catch (error) {
                console.error('Failed to process audio:', error);
                alert('Failed to process audio: ' + error.message);
            }
        };
        
        mediaRecorder.onerror = function(event) {
            console.error('MediaRecorder error:', event.error);
            stopVoiceInput();
            alert('Audio recording error: ' + event.error);
        };
        
        return true;
    } catch (error) {
        console.error('Failed to initialize audio recording:', error);
        alert('Failed to access microphone. Please check permissions.');
        return false;
    }
}

// Convert audio blob to WAV format using Web Audio API
async function convertToWav(audioBlob) {
    try {
        // Create AudioContext with 16000 Hz sample rate
        const audioContext = new (window.AudioContext || window.webkitAudioContext)({
            sampleRate: 16000
        });
        const arrayBuffer = await audioBlob.arrayBuffer();
        const audioBuffer = await audioContext.decodeAudioData(arrayBuffer);
        
        console.log('AudioBuffer info:', {
            sampleRate: audioBuffer.sampleRate,
            length: audioBuffer.length,
            duration: audioBuffer.duration,
            numberOfChannels: audioBuffer.numberOfChannels
        });
        
        // Resample to 16000 Hz if needed
        let processedBuffer = audioBuffer;
        if (audioBuffer.sampleRate !== 16000) {
            console.log(`Resampling from ${audioBuffer.sampleRate} Hz to 16000 Hz`);
            processedBuffer = await resampleAudioBuffer(audioBuffer, 16000);
        }
        
        // Convert AudioBuffer to WAV
        const wav = audioBufferToWav(processedBuffer);
        return new Blob([wav], { type: 'audio/wav' });
    } catch (error) {
        console.warn('Failed to convert to WAV, sending original:', error);
        // Fallback: send original blob, let backend try opus format
        return audioBlob;
    }
}

// Resample AudioBuffer to target sample rate
async function resampleAudioBuffer(sourceBuffer, targetSampleRate) {
    const offlineContext = new OfflineAudioContext(
        sourceBuffer.numberOfChannels,
        Math.floor(sourceBuffer.length * targetSampleRate / sourceBuffer.sampleRate),
        targetSampleRate
    );
    
    const source = offlineContext.createBufferSource();
    source.buffer = sourceBuffer;
    source.connect(offlineContext.destination);
    source.start();
    
    return await offlineContext.startRendering();
}

// Convert AudioBuffer to WAV format
function audioBufferToWav(buffer) {
    const length = buffer.length;
    const numberOfChannels = buffer.numberOfChannels;
    const sampleRate = buffer.sampleRate;
    const bytesPerSample = 2; // 16-bit
    const blockAlign = numberOfChannels * bytesPerSample;
    const byteRate = sampleRate * blockAlign;
    const dataSize = length * blockAlign;
    const bufferSize = 44 + dataSize;
    
    const arrayBuffer = new ArrayBuffer(bufferSize);
    const view = new DataView(arrayBuffer);
    
    // WAV header
    const writeString = (offset, string) => {
        for (let i = 0; i < string.length; i++) {
            view.setUint8(offset + i, string.charCodeAt(i));
        }
    };
    
    writeString(0, 'RIFF');
    view.setUint32(4, bufferSize - 8, true);
    writeString(8, 'WAVE');
    writeString(12, 'fmt ');
    view.setUint32(16, 16, true); // fmt chunk size
    view.setUint16(20, 1, true); // audio format (PCM)
    view.setUint16(22, numberOfChannels, true);
    view.setUint32(24, sampleRate, true);
    view.setUint32(28, byteRate, true);
    view.setUint16(32, blockAlign, true);
    view.setUint16(34, 16, true); // bits per sample
    writeString(36, 'data');
    view.setUint32(40, dataSize, true);
    
    // Convert float samples to 16-bit PCM
    let offset = 44;
    for (let i = 0; i < length; i++) {
        for (let channel = 0; channel < numberOfChannels; channel++) {
            const sample = Math.max(-1, Math.min(1, buffer.getChannelData(channel)[i]));
            view.setInt16(offset, sample < 0 ? sample * 0x8000 : sample * 0x7FFF, true);
            offset += 2;
        }
    }
    
    return arrayBuffer;
}

// Send audio to backend STT service
async function sendAudioToSTT(audioBlob, originalBlob = null) {
    try {
        console.log('Sending audio to STT:', {
            size: audioBlob.size,
            type: audioBlob.type
        });
        
        const response = await fetch('/api/stt', {
            method: 'POST',
            body: audioBlob,
            headers: {
                'Content-Type': 'application/octet-stream'
            }
        });
        
        console.log('STT response status:', response.status, response.statusText);
        
        if (!response.ok) {
            const errorText = await response.text().catch(() => 'Unknown error');
            console.error('STT HTTP error:', {
                status: response.status,
                statusText: response.statusText,
                body: errorText
            });
            throw new Error(`STT API error (${response.status}): ${errorText}`);
        }
        
        const result = await response.json();
        console.log('STT response:', result);
        
        if (result.status === 'success' && result.result) {
            const input = document.getElementById('agent-input');
            if (input) {
                input.value = result.result;
                // Auto-send after a short delay
                setTimeout(() => {
                    if (input.value.trim()) {
                        sendAgentMessage();
                    }
                }, 300);
            }
        } else {
            console.error('STT recognition failed:', result);
            throw new Error(`STT recognition failed: ${result.error || 'Unknown error'}`);
        }
    } catch (error) {
        console.error('Failed to send audio to STT:', error);
        alert('Speech recognition error: ' + error.message);
    }
}

// Toggle voice input
async function toggleVoiceInput() {
    if (isRecording) {
        stopVoiceInput();
    } else {
        // Always reinitialize to ensure fresh stream
        const initialized = await initAudioRecording();
        if (!initialized) {
            return;
        }
        
        if (mediaRecorder && mediaRecorder.state === 'inactive') {
            audioChunks = [];
            try {
                mediaRecorder.start();
                isRecording = true;
                
                const btn = document.getElementById('agent-voice-input-btn');
                const icon = document.getElementById('voice-input-icon');
                if (btn) {
                    btn.classList.add('recording');
                    icon.textContent = 'Stop';
                }
            } catch (error) {
                console.error('Failed to start MediaRecorder:', error);
                alert('Failed to start recording: ' + error.message);
                // Clean up on error
                if (audioStream) {
                    audioStream.getTracks().forEach(track => track.stop());
                    audioStream = null;
                }
                mediaRecorder = null;
            }
        }
    }
}

function stopVoiceInput() {
    isRecording = false;
    const btn = document.getElementById('agent-voice-input-btn');
    const icon = document.getElementById('voice-input-icon');
    if (btn) {
        btn.classList.remove('recording');
        icon.textContent = 'Mic';
    }
    if (mediaRecorder && mediaRecorder.state === 'recording') {
        mediaRecorder.stop();
    }
    // Don't stop stream here - we'll reuse it for next recording
}

// Text-to-Speech function using backend Aliyun TTS
async function speakText(text) {
    const ttsToggle = document.getElementById('agent-tts-toggle');
    if (!ttsToggle || !ttsToggle.checked) {
        return; // TTS disabled
    }
    
    if (!text || text.trim().length === 0) {
        return;
    }
    
    try {
        // Stop any current speech
        stopSpeaking();
        
        // Call backend TTS API
        const response = await fetch('/api/tts', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({
                text: text,
                format: 'wav',
                voice: 'zhishuo'
            }),
        });
        
        if (!response.ok) {
            throw new Error(`TTS API error: ${response.status}`);
        }
        
        // Get audio data
        const audioBlob = await response.blob();
        const audioUrl = URL.createObjectURL(audioBlob);
        
        // Create audio element and play
        const audio = new Audio(audioUrl);
        currentUtterance = audio;
        
        audio.onended = function() {
            URL.revokeObjectURL(audioUrl);
            currentUtterance = null;
        };
        
        audio.onerror = function(event) {
            console.error('Audio playback error:', event);
            URL.revokeObjectURL(audioUrl);
            currentUtterance = null;
        };
        
        await audio.play();
    } catch (error) {
        console.error('Failed to synthesize speech:', error);
        // Fallback to browser TTS if available
        if ('speechSynthesis' in window) {
            const utterance = new SpeechSynthesisUtterance(text);
            utterance.lang = 'zh-CN';
            utterance.rate = 1.0;
            utterance.pitch = 1.0;
            utterance.volume = 1.0;
            currentUtterance = utterance;
            window.speechSynthesis.speak(utterance);
        }
    }
}

// Stop TTS
function stopSpeaking() {
    // Stop backend audio
    if (currentUtterance && currentUtterance instanceof HTMLAudioElement) {
        currentUtterance.pause();
        currentUtterance.currentTime = 0;
        currentUtterance = null;
    }
    
    // Stop browser TTS
    if ('speechSynthesis' in window) {
        window.speechSynthesis.cancel();
    }
}

// Stop speaking when TTS toggle is turned off
document.addEventListener('DOMContentLoaded', function() {
    const ttsToggle = document.getElementById('agent-tts-toggle');
    if (ttsToggle) {
        ttsToggle.addEventListener('change', function() {
            if (!this.checked) {
                stopSpeaking();
            }
        });
    }
});

// Clear agent chat (local only)
function clearAgentChat() {
    if (!confirm('Clear all chat messages? This will only clear the display, not the agent\'s memory.')) {
        return;
    }
    
    const chatContainer = document.getElementById('agent-chat-messages');
    if (!chatContainer) return;
    
    // Clear all messages
    chatContainer.innerHTML = '';
    
    // Add welcome message back
    const welcomeMessage = document.createElement('div');
    welcomeMessage.className = 'agent-message agent-message-bot';
    const welcomeContent = document.createElement('div');
    welcomeContent.className = 'agent-message-content';
    welcomeContent.textContent = 'Hello! I\'m the Robonix agent. I can help you query the semantic map, submit tasks, and query system capabilities. How can I assist you?';
    welcomeMessage.appendChild(welcomeContent);
    chatContainer.appendChild(welcomeMessage);
    
    // Clear localStorage
    localStorage.removeItem(CHAT_HISTORY_KEY);
}

// Reset agent (clear backend history)
async function resetAgent() {
    if (!confirm('Reset agent? This will clear the agent\'s conversation history on the server.')) {
        return;
    }
    
    try {
        const response = await fetch('/api/agent/reset', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        // Clear local chat and localStorage
        const chatContainer = document.getElementById('agent-chat-messages');
        if (chatContainer) {
            chatContainer.innerHTML = '';
            
            // Add welcome message back
            const welcomeMessage = document.createElement('div');
            welcomeMessage.className = 'agent-message agent-message-bot';
            const welcomeContent = document.createElement('div');
            welcomeContent.className = 'agent-message-content';
            welcomeContent.textContent = 'Hello! I\'m the Robonix agent. I can help you query the semantic map, submit tasks, and query system capabilities. How can I assist you?';
            welcomeMessage.appendChild(welcomeContent);
            chatContainer.appendChild(welcomeMessage);
            
            // Add success message
            const successMessage = document.createElement('div');
            successMessage.className = 'agent-message agent-message-bot';
            const successContent = document.createElement('div');
            successContent.className = 'agent-message-content';
            successContent.style.color = '#4caf50';
            successContent.textContent = 'Agent has been reset. Conversation history cleared.';
            successMessage.appendChild(successContent);
            chatContainer.appendChild(successMessage);
            
            chatContainer.scrollTop = chatContainer.scrollHeight;
        }
        
        // Clear localStorage
        localStorage.removeItem(CHAT_HISTORY_KEY);
    } catch (error) {
        console.error('Failed to reset agent:', error);
        alert(`Failed to reset agent: ${error.message}`);
    }
}

// Agent chat history persistence
const CHAT_HISTORY_KEY = 'robonix_agent_chat_history';
const MAX_HISTORY_SIZE = 100; // Maximum number of messages to keep

function saveChatHistory() {
    try {
        const chatContainer = document.getElementById('agent-chat-messages');
        if (!chatContainer) return;
        
        const messages = [];
        const messageElements = chatContainer.querySelectorAll('.agent-message:not(#agent-loading)');
        
        messageElements.forEach((msgEl) => {
            const isUser = msgEl.classList.contains('agent-message-user');
            const contentEl = msgEl.querySelector('.agent-message-content');
            
            if (contentEl) {
                const messageData = {
                    type: isUser ? 'user' : 'bot',
                    content: contentEl.textContent,
                    functionCalls: []
                };
                
                // Extract function calls if any
                const functionCalls = msgEl.querySelectorAll('.agent-function-call');
                functionCalls.forEach((funcCallEl) => {
                    const funcName = funcCallEl.querySelector('.function-name')?.textContent || '';
                    const funcArgsEl = funcCallEl.querySelector('.agent-function-args');
                    const funcResultEl = funcCallEl.querySelector('.agent-function-result');
                    
                    let args = {};
                    let result = {};
                    
                    try {
                        if (funcArgsEl) {
                            args = JSON.parse(funcArgsEl.textContent.replace(/^Arguments:\s*/, ''));
                        }
                    } catch (e) {
                        args = { raw: funcArgsEl?.textContent || '' };
                    }
                    
                    try {
                        if (funcResultEl) {
                            result = JSON.parse(funcResultEl.textContent.replace(/^Result:\s*/, ''));
                        }
                    } catch (e) {
                        result = { raw: funcResultEl?.textContent || '' };
                    }
                    
                    messageData.functionCalls.push({
                        name: funcName,
                        arguments: args,
                        result: result
                    });
                });
                
                messages.push(messageData);
            }
        });
        
        // Keep only last MAX_HISTORY_SIZE messages
        const messagesToSave = messages.slice(-MAX_HISTORY_SIZE);
        localStorage.setItem(CHAT_HISTORY_KEY, JSON.stringify(messagesToSave));
    } catch (e) {
        console.error('Failed to save chat history:', e);
    }
}

function loadChatHistory() {
    try {
        const saved = localStorage.getItem(CHAT_HISTORY_KEY);
        if (!saved) return;
        
        const messages = JSON.parse(saved);
        if (!Array.isArray(messages) || messages.length === 0) return;
        
        const chatContainer = document.getElementById('agent-chat-messages');
        if (!chatContainer) return;
        
        // Clear existing messages (except initial welcome message)
        const existingMessages = chatContainer.querySelectorAll('.agent-message');
        let hasWelcomeMessage = false;
        existingMessages.forEach((msg, index) => {
            // Keep the first welcome message if it exists
            if (index === 0 && msg.querySelector('.agent-message-content')?.textContent.includes('Hello')) {
                hasWelcomeMessage = true;
                return;
            }
            msg.remove();
        });
        
        // If we have saved history, remove welcome message to avoid duplication
        if (hasWelcomeMessage && messages.length > 0) {
            const welcomeMsg = chatContainer.querySelector('.agent-message');
            if (welcomeMsg) {
                welcomeMsg.remove();
            }
        }
        
        // Restore messages
        messages.forEach((msgData, index) => {
            const messageEl = document.createElement('div');
            messageEl.className = `agent-message agent-message-${msgData.type}`;
            // Disable animation for restored messages to avoid overwhelming animation
            messageEl.style.animation = 'none';
            messageEl.style.opacity = '1';
            
            if (msgData.content) {
                const contentEl = document.createElement('div');
                contentEl.className = 'agent-message-content';
                contentEl.textContent = msgData.content;
                messageEl.appendChild(contentEl);
            }
            
            // Restore function calls
            if (msgData.functionCalls && msgData.functionCalls.length > 0) {
                msgData.functionCalls.forEach((funcCall) => {
                    const funcCallDiv = document.createElement('div');
                    funcCallDiv.className = 'agent-function-call';
                    
                    const funcHeader = document.createElement('div');
                    funcHeader.className = 'agent-function-header';
                    funcHeader.style.cursor = 'pointer';
                    funcHeader.onclick = function() {
                        const details = funcCallDiv.querySelector('.agent-function-details');
                        if (details) {
                            details.classList.toggle('expanded');
                            const toggle = funcHeader.querySelector('.function-toggle');
                            if (toggle) {
                                toggle.textContent = details.classList.contains('expanded') ? '▼' : '▶';
                            }
                        }
                    };
                    funcHeader.innerHTML = `<span class="function-toggle">▶</span> <span class="function-icon">⚙️</span> <span class="function-name">${escapeHtml(funcCall.name)}</span>`;
                    funcCallDiv.appendChild(funcHeader);
                    
                    const detailsDiv = document.createElement('div');
                    detailsDiv.className = 'agent-function-details';
                    
                    const argsLabel = document.createElement('div');
                    argsLabel.className = 'agent-function-label';
                    argsLabel.textContent = 'Arguments:';
                    detailsDiv.appendChild(argsLabel);
                    
                    const funcArgs = document.createElement('div');
                    funcArgs.className = 'agent-function-args';
                    funcArgs.textContent = JSON.stringify(funcCall.arguments, null, 2);
                    detailsDiv.appendChild(funcArgs);
                    
                    const resultLabel = document.createElement('div');
                    resultLabel.className = 'agent-function-label';
                    resultLabel.textContent = 'Result:';
                    detailsDiv.appendChild(resultLabel);
                    
                    const funcResult = document.createElement('div');
                    funcResult.className = 'agent-function-result';
                    funcResult.textContent = JSON.stringify(funcCall.result, null, 2);
                    detailsDiv.appendChild(funcResult);
                    
                    funcCallDiv.appendChild(detailsDiv);
                    messageEl.appendChild(funcCallDiv);
                });
            }
            
            chatContainer.appendChild(messageEl);
        });
        
        // Scroll to bottom after rendering
        requestAnimationFrame(() => {
            chatContainer.scrollTop = chatContainer.scrollHeight;
        });
    } catch (e) {
        console.error('Failed to load chat history:', e);
    }
}

// Agent chat functions
async function sendAgentMessage() {
    const input = document.getElementById('agent-input');
    const message = input.value.trim();
    
    if (!message) return;
    
    // Clear input
    input.value = '';
    
    // Add user message to chat
    const chatContainer = document.getElementById('agent-chat-messages');
    const userMessage = document.createElement('div');
    userMessage.className = 'agent-message agent-message-user';
    const userContent = document.createElement('div');
    userContent.className = 'agent-message-content';
    userContent.textContent = message;
    userMessage.appendChild(userContent);
    chatContainer.appendChild(userMessage);
    
    // Save chat history
    saveChatHistory();
    
    // Scroll to bottom
    chatContainer.scrollTop = chatContainer.scrollHeight;
    
    // Show loading indicator with typing animation
    const loadingMessage = document.createElement('div');
    loadingMessage.className = 'agent-message agent-message-bot';
    loadingMessage.id = 'agent-loading';
    const loadingContent = document.createElement('div');
    loadingContent.className = 'agent-message-content agent-loading-content';
    loadingContent.innerHTML = '<span class="typing-dots"><span></span><span></span><span></span></span>';
    loadingMessage.appendChild(loadingContent);
    chatContainer.appendChild(loadingMessage);
    chatContainer.scrollTop = chatContainer.scrollHeight;
    
    try {
        const response = await fetch('/api/agent/chat', {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json',
            },
            body: JSON.stringify({ message }),
        });
        
        if (!response.ok) {
            throw new Error(`HTTP error! status: ${response.status}`);
        }
        
        const data = await response.json();
        
        // Remove loading indicator
        const loading = document.getElementById('agent-loading');
        if (loading) {
            loading.remove();
        }
        
        // Add agent response (strip markdown if any)
        const agentMessage = document.createElement('div');
        agentMessage.className = 'agent-message agent-message-bot';
        
        // Add natural language message (only if not empty)
        const messageText = stripMarkdown(data.message || '');
        if (messageText.trim()) {
            const agentContent = document.createElement('div');
            agentContent.className = 'agent-message-content';
            agentContent.textContent = messageText;
            agentMessage.appendChild(agentContent);
            
            // Speak the response if TTS is enabled (async, don't wait)
            speakText(messageText).catch(err => {
                console.error('TTS error:', err);
            });
        }
        
        // Add function calls if any
        if (data.function_calls && data.function_calls.length > 0) {
            data.function_calls.forEach((funcCall, index) => {
                const funcCallDiv = document.createElement('div');
                funcCallDiv.className = 'agent-function-call';
                
                const funcHeader = document.createElement('div');
                funcHeader.className = 'agent-function-header';
                funcHeader.style.cursor = 'pointer';
                funcHeader.onclick = function() {
                    const details = funcCallDiv.querySelector('.agent-function-details');
                    if (details) {
                        details.classList.toggle('expanded');
                        const toggle = funcHeader.querySelector('.function-toggle');
                        if (toggle) {
                            toggle.textContent = details.classList.contains('expanded') ? '▼' : '▶';
                        }
                    }
                };
                funcHeader.innerHTML = `<span class="function-toggle">▶</span> <span class="function-icon">⚙️</span> <span class="function-name">${escapeHtml(funcCall.name)}</span>`;
                funcCallDiv.appendChild(funcHeader);
                
                // Details container (collapsible)
                const detailsDiv = document.createElement('div');
                detailsDiv.className = 'agent-function-details';
                
                // Arguments section
                const argsLabel = document.createElement('div');
                argsLabel.className = 'agent-function-label';
                argsLabel.textContent = 'Arguments:';
                detailsDiv.appendChild(argsLabel);
                
                const funcArgs = document.createElement('div');
                funcArgs.className = 'agent-function-args';
                try {
                    funcArgs.textContent = JSON.stringify(funcCall.arguments, null, 2);
                } catch (e) {
                    funcArgs.textContent = String(funcCall.arguments);
                }
                detailsDiv.appendChild(funcArgs);
                
                // Result section
                const resultLabel = document.createElement('div');
                resultLabel.className = 'agent-function-label';
                resultLabel.textContent = 'Result:';
                detailsDiv.appendChild(resultLabel);
                
                const funcResult = document.createElement('div');
                funcResult.className = 'agent-function-result';
                try {
                    funcResult.textContent = JSON.stringify(funcCall.result, null, 2);
                } catch (e) {
                    funcResult.textContent = String(funcCall.result);
                }
                detailsDiv.appendChild(funcResult);
                
                funcCallDiv.appendChild(detailsDiv);
                agentMessage.appendChild(funcCallDiv);
            });
        }
        
        chatContainer.appendChild(agentMessage);
        
        // Save chat history
        saveChatHistory();
        
        // Scroll to bottom
        chatContainer.scrollTop = chatContainer.scrollHeight;
    } catch (error) {
        console.error('Failed to send agent message:', error);
        
        // Remove loading indicator
        const loading = document.getElementById('agent-loading');
        if (loading) {
            loading.remove();
        }
        
        // Show error message
        const errorMessage = document.createElement('div');
        errorMessage.className = 'agent-message agent-message-bot';
        const errorContent = document.createElement('div');
        errorContent.className = 'agent-message-content';
        errorContent.style.color = '#d32f2f';
        errorContent.textContent = `Error: ${error.message}`;
        errorMessage.appendChild(errorContent);
        chatContainer.appendChild(errorMessage);
        
        // Save chat history
        saveChatHistory();
        
        // Scroll to bottom
        chatContainer.scrollTop = chatContainer.scrollHeight;
    }
}

// Mobile menu toggle
document.getElementById('navbarToggle')?.addEventListener('click', function() {
    const menu = document.getElementById('navbarMenu');
    if (menu) {
        menu.classList.toggle('active');
    }
});

// Initialize on page load
loadStatus();
loadTfTree();
loadTopics();
loadLogs();
loadComponents();
loadVisualization();
loadSemanticMap2D();
setupAutoRefresh();
setupAutoRefreshLogs();
setupAutoRefreshComponents();
setupAutoRefreshViz();
setupAutoRefreshMap2D();
setupMap2DInteraction();

// Load chat history after a short delay to ensure DOM is ready
setTimeout(() => {
    loadChatHistory();
}, 100);
