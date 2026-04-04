(() => {
  function imageEndpointByMode(mode) {
    if (mode === 'rgb') return '/api/frame_rgb.jpg';
    if (mode === 'binary') return '/api/frame_binary.jpg';
    return '/api/frame_gray.jpg';
  }

  function frameUrlForMode(mode) {
    return `${imageEndpointByMode(mode)}?t=${Date.now()}`;
  }

  async function fetchJsonNoStore(url) {
    const response = await fetch(url, { cache: 'no-store' });
    if (!response.ok) {
      throw new Error(`${url} -> ${response.status}`);
    }
    return response.json();
  }

  function formatValue(value) {
    if (typeof value === 'number') {
      return Number.isFinite(value) ? value.toFixed(3) : String(value);
    }
    if (value === undefined || value === null) {
      return 'N/A';
    }
    return String(value);
  }

  function renderStatusList(container, rows) {
    if (!container) return;
    container.innerHTML = '';
    for (const [name, value] of rows) {
      const item = document.createElement('div');
      item.className = 'status-item';
      item.innerHTML = `<strong>${name}</strong>: ${formatValue(value)}`;
      container.appendChild(item);
    }
  }

  window.SharedReceiverCore = {
    imageEndpointByMode,
    frameUrlForMode,
    fetchJsonNoStore,
    formatValue,
    renderStatusList
  };
})();
