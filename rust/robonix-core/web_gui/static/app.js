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
            // Key format: "prm::camera.capture$provider$version"
            // We want: "prm::camera.capture"
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
    
    switch(type) {
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
        } catch (e) {}
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
    
    switch(type) {
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
        } catch (e) {}
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

// Visualization loading
let autoRefreshVizInterval = null;

async function loadVisualization() {
    await Promise.all([
        loadSemanticMap(),
        loadImageMonitor(),
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
    
    switch(type) {
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

// Close modal when clicking outside
window.onclick = function(event) {
    const rtdlModal = document.getElementById('rtdl-modal');
    const componentModal = document.getElementById('component-modal');
    const componentListModal = document.getElementById('component-list-modal');
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

(function() {
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
    window.addEventListener('scroll', function() {
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

// Initialize on page load
loadStatus();
loadTfTree();
loadTopics();
loadLogs();
loadComponents();
loadVisualization();
setupAutoRefresh();
setupAutoRefreshLogs();
setupAutoRefreshComponents();
setupAutoRefreshViz();
