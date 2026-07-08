/* ── State ──────────────────────────────────────────────── */
let selectedFiles = [];
let sessionUploads = 0;

const $ = id => document.getElementById(id);

/* ── Init ───────────────────────────────────────────────── */
document.addEventListener('DOMContentLoaded', () => {
  fetchServerInfo();
  fetchFiles();
  setupDropZone();
  setupButtons();
});

/* ── Server Info / QR ────────────────────────────────────── */
async function fetchServerInfo() {
  try {
    const res = await fetch('/api/info');
    const data = await res.json();

    $('serverUrl').textContent = data.url;
    $('saveDir').textContent = data.uploadDir;
    $('serverStatus').textContent = 'Server Online';
    $('serverBadge').style.opacity = '1';

    if (data.qrDataUrl) {
      const img = document.createElement('img');
      img.src = data.qrDataUrl;
      img.alt = 'QR Code to open uploader';
      $('qrWrapper').innerHTML = '';
      $('qrWrapper').appendChild(img);
    }
  } catch (err) {
    $('serverStatus').textContent = 'Connection Error';
    console.error('Failed to fetch server info:', err);
  }
}

/* ── File List ───────────────────────────────────────────── */
async function fetchFiles() {
  try {
    const res = await fetch('/api/files');
    const data = await res.json();
    if (!data.success) return;

    $('totalFiles').textContent = data.count;

    const list = $('fileList');
    if (data.files.length === 0) {
      list.innerHTML = '<div class="empty-state">No uploads yet 🌱</div>';
      return;
    }

    list.innerHTML = data.files.slice(0, 12).map(f => `
      <div class="file-item">
        <div class="file-thumb">${getFileEmoji(f.name)}</div>
        <div class="file-info">
          <div class="file-name">${f.name}</div>
          <div class="file-meta">${formatSize(f.size)} · ${timeAgo(f.modified)}</div>
        </div>
      </div>
    `).join('');
  } catch (err) {
    console.error('Failed to fetch files:', err);
  }
}

/* ── Drop Zone Setup ─────────────────────────────────────── */
function setupDropZone() {
  const zone = $('dropZone');
  const input = $('fileInput');

  zone.addEventListener('click', () => input.click());
  $('browseBtn').addEventListener('click', e => { e.stopPropagation(); input.click(); });

  zone.addEventListener('dragover', e => { e.preventDefault(); zone.classList.add('drag-over'); });
  zone.addEventListener('dragleave', e => { if (!zone.contains(e.relatedTarget)) zone.classList.remove('drag-over'); });
  zone.addEventListener('drop', e => {
    e.preventDefault();
    zone.classList.remove('drag-over');
    const files = Array.from(e.dataTransfer.files).filter(isImage);
    addFiles(files);
  });

  input.addEventListener('change', () => {
    const files = Array.from(input.files).filter(isImage);
    addFiles(files);
    input.value = '';
  });
}

/* ── Buttons Setup ───────────────────────────────────────── */
function setupButtons() {
  $('uploadBtn').addEventListener('click', handleUpload);
  $('clearBtn').addEventListener('click', clearFiles);
  $('copyBtn').addEventListener('click', copyUrl);
  $('refreshBtn').addEventListener('click', () => {
    $('refreshBtn').style.transition = 'transform 0.4s';
    fetchFiles();
  });
}

/* ── File Management ─────────────────────────────────────── */
function addFiles(newFiles) {
  selectedFiles = [...selectedFiles, ...newFiles];
  renderPreviews();
  updateUploadButton();
}

function removeFile(index) {
  selectedFiles.splice(index, 1);
  renderPreviews();
  updateUploadButton();
}

function clearFiles() {
  selectedFiles = [];
  renderPreviews();
  updateUploadButton();
  $('toast').className = 'toast';
}

function renderPreviews() {
  const section = $('previewSection');
  const grid = $('previewGrid');
  const count = $('previewCount');

  if (selectedFiles.length === 0) {
    section.style.display = 'none';
    return;
  }
  section.style.display = 'block';
  count.textContent = `${selectedFiles.length} file${selectedFiles.length !== 1 ? 's' : ''} selected`;

  grid.innerHTML = '';
  selectedFiles.forEach((file, i) => {
    const div = document.createElement('div');
    div.className = 'preview-item';
    const url = URL.createObjectURL(file);
    div.innerHTML = `
      <img src="${url}" alt="${file.name}" onload="URL.revokeObjectURL('${url}')" />
      <button class="remove-btn" onclick="removeFile(${i})">✕</button>
      <span class="size-badge">${formatSize(file.size)}</span>
    `;
    grid.appendChild(div);
  });
}

function updateUploadButton() {
  const btn = $('uploadBtn');
  const text = $('uploadBtnText');
  if (selectedFiles.length === 0) {
    btn.disabled = true;
    text.textContent = 'Select images to upload';
  } else {
    btn.disabled = false;
    text.textContent = `Upload ${selectedFiles.length} image${selectedFiles.length !== 1 ? 's' : ''}`;
  }
}

/* ── Upload Handler ──────────────────────────────────────── */
async function handleUpload() {
  if (selectedFiles.length === 0) return;

  const btn = $('uploadBtn');
  const text = $('uploadBtnText');
  const progressWrapper = $('progressWrapper');
  const progressFill = $('progressFill');
  const progressText = $('progressText');
  const toast = $('toast');

  // UI — uploading state
  btn.disabled = true;
  text.textContent = 'Uploading...';
  toast.className = 'toast';
  progressWrapper.style.display = 'block';
  progressFill.style.width = '0%';
  progressText.textContent = 'Preparing...';

  const formData = new FormData();
  selectedFiles.forEach(f => formData.append('images', f));

  try {
    // Animate progress bar while uploading
    let fakeProgress = 0;
    const progressInterval = setInterval(() => {
      fakeProgress = Math.min(fakeProgress + Math.random() * 15, 85);
      progressFill.style.width = fakeProgress + '%';
      progressText.textContent = `Uploading... ${Math.round(fakeProgress)}%`;
    }, 200);

    const res = await fetch('/api/upload', { method: 'POST', body: formData });
    const data = await res.json();

    clearInterval(progressInterval);

    if (data.success) {
      progressFill.style.width = '100%';
      progressText.textContent = 'Done!';
      sessionUploads += data.count;
      $('sessionUploads').textContent = sessionUploads;

      toast.className = 'toast success';
      toast.textContent = `✅ ${data.count} image${data.count !== 1 ? 's' : ''} downsampled to 128×128 and saved to your laptop!`;

      clearFiles();
      setTimeout(fetchFiles, 800);
    } else {
      throw new Error(data.message || 'Upload failed');
    }
  } catch (err) {
    $('progressFill').style.width = '0%';
    $('progressText').textContent = '';
    toast.className = 'toast error';
    toast.textContent = `❌ ${err.message}`;
    updateUploadButton();
  }

  setTimeout(() => { progressWrapper.style.display = 'none'; }, 2500);
}

/* ── Copy URL ────────────────────────────────────────────── */
function copyUrl() {
  const url = $('serverUrl').textContent;
  navigator.clipboard.writeText(url).then(() => {
    const btn = $('copyBtn');
    btn.textContent = '✓';
    btn.style.color = 'var(--success)';
    setTimeout(() => { btn.textContent = '⧉'; btn.style.color = ''; }, 1500);
  });
}

/* ── Helpers ─────────────────────────────────────────────── */
function isImage(file) {
  return /^image\//i.test(file.type) || /\.(jpg|jpeg|png|gif|bmp|webp|heic|tiff|svg)$/i.test(file.name);
}

function formatSize(bytes) {
  if (bytes < 1024) return bytes + ' B';
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(1) + ' KB';
  return (bytes / (1024 * 1024)).toFixed(1) + ' MB';
}

function timeAgo(dateStr) {
  const diff = Date.now() - new Date(dateStr).getTime();
  const m = Math.floor(diff / 60000);
  if (m < 1) return 'just now';
  if (m < 60) return m + 'm ago';
  const h = Math.floor(m / 60);
  if (h < 24) return h + 'h ago';
  return Math.floor(h / 24) + 'd ago';
}

function getFileEmoji(name) {
  const ext = name.split('.').pop().toLowerCase();
  const map = { gif: '🎞️', png: '🖼️', svg: '🎨', webp: '🌐', heic: '📷' };
  return map[ext] || '📸';
}
