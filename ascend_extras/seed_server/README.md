# 🌱 Seed Image Uploader

A **lightweight local network image uploader** built with **Flask + Pillow** — optimised for Raspberry Pi and low-power devices. Run the server, upload photos from your phone over Wi-Fi, and every image is **automatically downsampled to 128×128 pixels** and saved to your configured folder. The HD original is **never written to disk**.

Includes a standalone companion script (`hd_to_lr_test.py`) for **live webcam → 128×128** capture.

![Flask](https://img.shields.io/badge/Flask-3.x-000000?style=flat-square&logo=flask&logoColor=white)
![Python](https://img.shields.io/badge/Python-3.8%2B-3776AB?style=flat-square&logo=python&logoColor=white)
![Pillow](https://img.shields.io/badge/Pillow-10.x-brightgreen?style=flat-square)
![OpenCV](https://img.shields.io/badge/OpenCV-4.x-5C3EE8?style=flat-square&logo=opencv&logoColor=white)
![RPi](https://img.shields.io/badge/Raspberry%20Pi-ready-C51A4A?style=flat-square&logo=raspberry-pi&logoColor=white)
![License](https://img.shields.io/badge/License-MIT-blue?style=flat-square)

---

## ✨ Features

- 📐 **Auto 128×128 downsampling** — server-side resize on every upload; HD original never saved
- 📱 **Phone-friendly UI** — fully responsive drag & drop interface
- 🔲 **Auto QR Code** — scan to open the uploader instantly on any device
- 📦 **Multi-file upload** — up to 20 images at once, 50 MB each
- 📁 **Configurable save folder** — one-line change in `app.py`
- 📊 **Live stats** — session counter, total files, recent uploads list
- 🌐 **LAN only** — purely local, no internet required
- 🪶 **Lightweight** — pure Python, minimal dependencies, boots fast on RPi

---

## 🔄 Upload Pipeline

```
Phone / Browser
      │
      │  POST /api/upload  (multipart, ≤ 50 MB each)
      ▼
  Flask  ──►  Pillow: Image.resize((128, 128), LANCZOS)
                                    │
                                    ▼
              ~/ardu_ws/seed_images/lr_<name>_<timestamp>.jpg
```

> Only the **128×128 LR JPEG** is ever written to disk.

---

## 🚀 Quick Start

### 1 — Install dependencies

```bash
git clone https://github.com/your-username/Seed_image_uploader.git
cd Seed_image_uploader

python3 -m venv venv
source venv/bin/activate          # Windows: venv\Scripts\activate

pip install -r requirements.txt
```

### 2 — Configure save folder

Edit the top of `app.py`:

```python
UPLOAD_DIR = Path.home() / "ardu_ws" / "seed_images"   # ← your path here
PORT       = 3000
LR_SIZE    = (128, 128)
```

### 3 — Run

```bash
python app.py
```

```
╔══════════════════════════════════════════╗
║       🌱  Seed Image Uploader  🌱        ║
╠══════════════════════════════════════════╣
║  Local:   http://localhost:3000          ║
║  Network: http://192.168.1.42:3000       ║
╚══════════════════════════════════════════╝
📱 Scan the QR code on the webpage to connect from your phone!
```

---

## 📱 Using from Your Phone

1. Phone and RPi/laptop must be on the **same Wi-Fi**
2. Open `http://localhost:3000` on the host device
3. **Scan the QR code** — the upload page opens on your phone
4. Select images → tap **Upload** → saved as `lr_*.jpg` instantly

> Or type the Network URL shown in the terminal directly into your phone's browser.

---

## 📁 Project Structure

```
Seed_image_uploader/
├── app.py                # Flask server — upload, Pillow resize, QR code, file list
├── hd_to_lr_test.py      # Standalone: webcam → 128×128 → saves to Desktop2
├── requirements.txt      # Python deps: flask, Pillow, qrcode[pil]
├── public/
│   ├── index.html        # Frontend HTML
│   ├── style.css         # Dark glassmorphic UI
│   └── app.js            # Drag & drop, preview, upload, stats
├── .gitignore
└── README.md
```

---

## ⚙️ Configuration

| Setting | Where | Default |
|---------|-------|---------|
| Save folder | `app.py` → `UPLOAD_DIR` | `~/ardu_ws/seed_images` |
| Output size | `app.py` → `LR_SIZE` | `(128, 128)` |
| Port | `app.py` → `PORT` | `3000` |
| Max file size | `app.py` → `MAX_FILE_SIZE_MB` | `50` |
| Resize filter | `app.py` → `Image.LANCZOS` | Lanczos |
| Allowed formats | `app.py` → `ALLOWED_EXTENSIONS` | JPG PNG GIF BMP WEBP HEIC TIFF |

---

## 🌐 API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| `GET` | `/` | Serves the web UI |
| `GET` | `/api/info` | Server IP, port, save path, QR code data URL |
| `POST` | `/api/upload` | Upload images (field: `images`); returns saved LR filenames |
| `GET` | `/api/files` | List saved LR images with size & timestamp |

---

## 🎥 Standalone Webcam Script

`hd_to_lr_test.py` captures live webcam frames and saves 128×128 images to two folders simultaneously. Useful for collecting seed images without a phone.

**Requires:** `pip install opencv-python`

```bash
python3 hd_to_lr_test.py   # press q to stop
```

```
Webcam (HD) ──► cv2.resize(128×128, INTER_AREA)
                    ├──► ~/ardu_ws/seed_images/lr_YYYYMMDD_HHMMSS_NNNN.jpg
                    └──► ~/Desktop2/lr_YYYYMMDD_HHMMSS_NNNN.jpg
```

---

## 📦 Dependencies

| Package | Purpose |
|---------|---------|
| [Flask](https://flask.palletsprojects.com/) | Web framework |
| [Pillow](https://python-pillow.org/) | Image resize to 128×128 JPEG |
| [qrcode\[pil\]](https://pypi.org/project/qrcode/) | QR code generation |
| [opencv-python](https://pypi.org/project/opencv-python/) | Webcam script only (optional) |

---

## 🔒 Notes

- Intended for **local network use only** — add auth before exposing publicly
- HD originals are **never stored**; Pillow processes them in memory
- Output is always **JPEG** regardless of input format
- On Raspberry Pi, add to crontab for auto-start on boot:
  ```
  @reboot cd /path/to/Seed_image_uploader && venv/bin/python app.py >> ~/uploader.log 2>&1 &
  ```

---

## 📄 License

[MIT](LICENSE)
