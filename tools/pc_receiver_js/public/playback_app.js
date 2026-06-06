(() => {
  const params = new URLSearchParams(window.location.search || '');
  const folder = String(params.get('folder') || '').trim();
  const channel = String(params.get('channel') || '').trim();

  function setMessage(text) {
    const targets = [
      document.getElementById('sourceLabel'),
      document.getElementById('rawStatus')
    ].filter(Boolean);
    targets.forEach((el) => {
      el.textContent = text;
    });
  }

  if (folder) {
    const target = `/?replay=${encodeURIComponent(folder)}`;
    setMessage(`正在跳转到统一控制台回放：${folder}`);
    window.location.replace(target);
    return;
  }

  if (channel) {
    setMessage('当前会话视频回放已合并到统一控制台；请先保存录制，再从已保存回放打开。');
    return;
  }

  setMessage('没有提供回放目录，请从主页面的已保存回放打开。');
})();
