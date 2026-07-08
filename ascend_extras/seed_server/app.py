import os
import io
import socket
import base64
from pathlib import Path
from datetime import datetime, timezone

import qrcode
from PIL import Image
from flask import Flask, request, jsonify, send_from_directory

# ── Configuration ──────────────────────────────────────────────────────────────
UPLOAD_DIR = Path.home() / "ardu_ws" / "seed_images"   # ← change this path
PORT       = 3000
LR_SIZE    = (128, 128)
# ──────────────────────────────────────────────────────────────────────────────

ALLOWED_EXTENSIONS = {".jpg", ".jpeg", ".png", ".gif", ".bmp", ".webp", ".heic", ".tiff"}
MAX_FILE_SIZE_MB   = 50

UPLOAD_DIR.mkdir(parents=True, exist_ok=True)

app = Flask(__name__, static_folder="public", static_url_path="")


# ── Helpers ────────────────────────────────────────────────────────────────────

def get_local_ip() -> str:
    """Return the LAN IP address of this machine."""
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except Exception:
        return "localhost"


def make_qr_data_url(url: str) -> str:
    """Generate a QR code for *url* and return it as a base64 PNG data URL."""
    qr = qrcode.QRCode(border=2, box_size=6)
    qr.add_data(url)
    qr.make(fit=True)
    img = qr.make_image(fill_color="#a78bfa", back_color="#0f0f1a")
    buf = io.BytesIO()
    img.save(buf, format="PNG")
    b64 = base64.b64encode(buf.getvalue()).decode()
    return f"data:image/png;base64,{b64}"


def downsample_and_save(file_storage) -> dict:
    """
    Read an uploaded file from memory, resize to 128×128 with Lanczos,
    save as JPEG, and return metadata dict.
    The HD original is never written to disk.
    """
    original_name = file_storage.filename
    stem = Path(original_name).stem
    safe_stem = "".join(c if c.isalnum() or c in "_-" else "_" for c in stem)
    timestamp = datetime.now(timezone.utc).strftime("%Y-%m-%dT%H-%M-%S-%f")[:-3] + "Z"
    filename = f"lr_{safe_stem}_{timestamp}.jpg"
    out_path = UPLOAD_DIR / filename

    img_bytes = file_storage.read()
    img = Image.open(io.BytesIO(img_bytes)).convert("RGB")
    img = img.resize(LR_SIZE, Image.LANCZOS)   # Lanczos ≡ INTER_AREA quality
    img.save(out_path, "JPEG", quality=95)

    size = out_path.stat().st_size
    return {
        "originalName": original_name,
        "savedAs": filename,
        "size": size,
        "path": str(out_path),
    }


def is_allowed(filename: str) -> bool:
    return Path(filename).suffix.lower() in ALLOWED_EXTENSIONS


# ── Routes ─────────────────────────────────────────────────────────────────────

@app.route("/")
def index():
    """Serve the frontend."""
    return send_from_directory(app.static_folder, "index.html")


@app.route("/api/info")
def api_info():
    """Return server info + QR code data URL (mirrors Node.js /api/info)."""
    local_ip = get_local_ip()
    url = f"http://{local_ip}:{PORT}"
    qr_data_url = make_qr_data_url(url)
    return jsonify({
        "url":       url,
        "localIP":   local_ip,
        "port":      PORT,
        "uploadDir": str(UPLOAD_DIR),
        "qrDataUrl": qr_data_url,
    })


@app.route("/api/upload", methods=["POST"])
def api_upload():
    """
    Accept multipart/form-data with field 'images' (multiple files allowed).
    Resize each to 128×128 with Pillow and save as lr_*.jpg.
    """
    files = request.files.getlist("images")
    if not files or all(f.filename == "" for f in files):
        return jsonify({"success": False, "message": "No files were uploaded."}), 400

    saved = []
    for f in files:
        if not is_allowed(f.filename):
            return jsonify({
                "success": False,
                "message": f"'{f.filename}' is not an allowed image type."
            }), 400

        # Check size (werkzeug streams; read() is safe for ≤50 MB)
        f.stream.seek(0, 2)
        size_mb = f.stream.tell() / (1024 * 1024)
        f.stream.seek(0)
        if size_mb > MAX_FILE_SIZE_MB:
            return jsonify({
                "success": False,
                "message": f"'{f.filename}' exceeds {MAX_FILE_SIZE_MB} MB limit."
            }), 400

        try:
            result = downsample_and_save(f)
            saved.append(result)
        except Exception as exc:
            return jsonify({"success": False, "message": f"Image processing failed: {exc}"}), 500

    now = datetime.now().strftime("%H:%M:%S")
    print(f"\n✅  {len(saved)} image(s) downsampled & saved at {now}")
    for r in saved:
        print(f"   → {r['savedAs']}  (128×128, {r['size'] / 1024:.1f} KB)")

    return jsonify({"success": True, "count": len(saved), "files": saved})


@app.route("/api/files")
def api_files():
    """List all saved LR images, newest first."""
    try:
        files = []
        for p in UPLOAD_DIR.iterdir():
            if p.suffix.lower() in ALLOWED_EXTENSIONS:
                stat = p.stat()
                files.append({
                    "name":     p.name,
                    "size":     stat.st_size,
                    "modified": datetime.fromtimestamp(stat.st_mtime).isoformat(),
                })
        files.sort(key=lambda x: x["modified"], reverse=True)
        return jsonify({"success": True, "count": len(files), "files": files[:50]})
    except Exception as exc:
        return jsonify({"success": False, "message": str(exc)}), 500


# ── Entry point ────────────────────────────────────────────────────────────────

if __name__ == "__main__":
    local_ip = get_local_ip()
    print("\n╔══════════════════════════════════════════╗")
    print("║       🌱  Seed Image Uploader  🌱        ║")
    print("╠══════════════════════════════════════════╣")
    print(f"║  Local:   http://localhost:{PORT}           ║")
    print(f"║  Network: http://{local_ip}:{PORT}     ║")
    print(f"║  Saving to: {str(UPLOAD_DIR)[:28]}... ║")
    print("╚══════════════════════════════════════════╝")
    print("\n📱 Scan the QR code on the webpage to connect from your phone!\n")

    app.run(host="0.0.0.0", port=PORT, debug=False)
