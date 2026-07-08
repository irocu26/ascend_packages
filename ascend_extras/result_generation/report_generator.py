#!/usr/bin/env python3
"""
report_generator.py — turn a MissionTracker snapshot into deliverable reports.

Produces, into ``report_dir``:
    results_<ts>.json   — full machine-readable snapshot (tasks, telemetry,
                          determined feature coordinates, timeline)
    report_<ts>.html    — self-contained printable report (inline CSS + SVG
                          arena map). Open in a browser, "Print → Save as PDF".
    report_<ts>.pdf     — same content rendered with ReportLab (if installed)

This satisfies the elimination-round "Reporting of Result/Tasks" deliverable.
PDF generation degrades gracefully: if ReportLab is not installed the JSON +
HTML are still written and the returned dict simply omits the 'pdf' key.

No ROS / Flask imports — pure stdlib + optional ReportLab — so it is importable
and testable on its own.
"""

from __future__ import annotations

import html
import json
import os
from datetime import datetime

STATUS_GLYPH = {
    'done': '✔', 'active': '⟳', 'failed': '✖', 'pending': '○',
}
STATUS_WORD = {
    'done': 'COMPLETE', 'active': 'IN PROGRESS', 'failed': 'FAILED', 'pending': 'PENDING',
}
STATUS_COLOR = {
    'done': '#2ecc71', 'active': '#f1c40f', 'failed': '#e74c3c', 'pending': '#7f8c8d',
}


# ════════════════════════════════════════════════════════════════════════════
#  Public entry point
# ════════════════════════════════════════════════════════════════════════════
def generate(snapshot: dict, report_dir: str, *, stamp: str | None = None) -> dict:
    """Write JSON + HTML (+ PDF if possible). Returns {fmt: abspath}."""
    os.makedirs(report_dir, exist_ok=True)
    stamp = stamp or datetime.now().strftime('%Y%m%d_%H%M%S')
    paths: dict[str, str] = {}

    json_path = os.path.join(report_dir, f'results_{stamp}.json')
    with open(json_path, 'w') as f:
        json.dump(snapshot, f, indent=2)
    paths['json'] = os.path.abspath(json_path)

    html_path = os.path.join(report_dir, f'report_{stamp}.html')
    with open(html_path, 'w') as f:
        f.write(render_html(snapshot))
    paths['html'] = os.path.abspath(html_path)

    pdf_path = os.path.join(report_dir, f'report_{stamp}.pdf')
    if render_pdf(snapshot, pdf_path):
        paths['pdf'] = os.path.abspath(pdf_path)

    return paths


# ════════════════════════════════════════════════════════════════════════════
#  Shared helpers
# ════════════════════════════════════════════════════════════════════════════
def _fmt_elapsed(seconds: int) -> str:
    seconds = int(seconds or 0)
    return f'{seconds // 60:02d}:{seconds % 60:02d}'


def _verdict(snap: dict) -> tuple[str, str]:
    """(text, color) overall pass/partial/abort verdict."""
    m = snap['mission']
    if m.get('in_failsafe') or m.get('any_failed'):
        return 'MISSION ABORTED / PARTIAL', '#e74c3c'
    if m['tasks_done'] >= m['tasks_total']:
        return 'ALL TASKS COMPLETE', '#2ecc71'
    return f"{m['tasks_done']}/{m['tasks_total']} TASKS COMPLETE", '#f1c40f'


def build_arena_svg(snap: dict, width: int = 560) -> str:
    """Top-down SVG arena map: boundary, base zone, trail, drone, features."""
    arena = snap['arena']
    ax, ay = arena['x'], arena['y']
    pad = 1.0  # metres of margin around the arena
    vw, vh = ax + 2 * pad, ay + 2 * pad
    scale = width / vw
    height = int(vh * scale)

    def X(mx):  # arena metres → svg px (x stays x)
        return (mx + pad) * scale

    def Y(my):  # flip so +y points up in the image
        return (vh - (my + pad)) * scale

    parts = [
        f'<svg viewBox="0 0 {width} {height}" width="{width}" height="{height}" '
        f'xmlns="http://www.w3.org/2000/svg" font-family="monospace">',
        f'<rect x="0" y="0" width="{width}" height="{height}" fill="#0d1320"/>',
    ]
    # Grid every 1 m
    gx = pad
    while gx <= ax + pad + 1e-6:
        parts.append(f'<line x1="{X(gx - pad)}" y1="{Y(0)}" x2="{X(gx - pad)}" '
                     f'y2="{Y(ay)}" stroke="#1d2740" stroke-width="1"/>')
        gx += 1.0
    gy = 0.0
    while gy <= ay + 1e-6:
        parts.append(f'<line x1="{X(0)}" y1="{Y(gy)}" x2="{X(ax)}" '
                     f'y2="{Y(gy)}" stroke="#1d2740" stroke-width="1"/>')
        gy += 1.0
    # Arena boundary (yellow per rulebook)
    parts.append(
        f'<rect x="{X(0)}" y="{Y(ay)}" width="{ax * scale}" height="{ay * scale}" '
        f'fill="none" stroke="#f4d03f" stroke-width="3"/>')
    # Base zone (1.5 m square at dock corner)
    bz = 1.5
    parts.append(
        f'<rect x="{X(arena["dock_x"])}" y="{Y(arena["dock_y"] + bz)}" '
        f'width="{bz * scale}" height="{bz * scale}" '
        f'fill="#f4d03f22" stroke="#f4d03f" stroke-width="1" stroke-dasharray="4 3"/>')
    parts.append(f'<text x="{X(arena["dock_x"]) + 4}" y="{Y(arena["dock_y"]) - 4}" '
                 f'fill="#f4d03f" font-size="11">BASE</text>')

    # Drone trail
    trail = snap.get('trail', [])
    if len(trail) > 1:
        pts = ' '.join(f'{X(px):.1f},{Y(py):.1f}' for px, py in trail)
        parts.append(f'<polyline points="{pts}" fill="none" stroke="#3498db" '
                     f'stroke-width="2" opacity="0.7"/>')

    # Feature markers (★)
    for f in snap['features']['list']:
        cx, cy = X(f['x']), Y(f['y'])
        parts.append(_star(cx, cy, 9, '#2ecc71'))
        parts.append(f'<text x="{cx + 11}" y="{cy + 4}" fill="#2ecc71" '
                     f'font-size="11">F{f["seed_id"]} ({f["x"]:+.2f},{f["y"]:+.2f})</text>')

    # Drone current position
    t = snap['telemetry']
    if snap['mission']['started']:
        dx, dy = X(t['x']), Y(t['y'])
        parts.append(f'<circle cx="{dx}" cy="{dy}" r="7" fill="#e74c3c" '
                     f'stroke="#fff" stroke-width="2"/>')
        parts.append(f'<text x="{dx + 10}" y="{dy - 8}" fill="#e74c3c" '
                     f'font-size="11">DRONE</text>')

    parts.append('</svg>')
    return ''.join(parts)


def _star(cx, cy, r, color):
    import math
    pts = []
    for i in range(10):
        ang = -math.pi / 2 + i * math.pi / 5
        rad = r if i % 2 == 0 else r * 0.45
        pts.append(f'{cx + rad * math.cos(ang):.1f},{cy + rad * math.sin(ang):.1f}')
    return f'<polygon points="{" ".join(pts)}" fill="{color}"/>'


# ════════════════════════════════════════════════════════════════════════════
#  HTML report
# ════════════════════════════════════════════════════════════════════════════
def render_html(snap: dict) -> str:
    e = html.escape
    m = snap['mission']
    t = snap['telemetry']
    verdict, vcolor = _verdict(snap)
    svg = build_arena_svg(snap)

    task_rows = []
    for task in snap['tasks']:
        color = STATUS_COLOR[task['status']]
        glyph = STATUS_GLYPH[task['status']]
        dur = f'{task["duration_s"]:.0f}s' if task.get('duration_s') is not None else '—'
        started = task.get('started_iso') or '—'
        task_rows.append(
            f'<tr>'
            f'<td style="color:{color};font-weight:bold">{glyph} {e(STATUS_WORD[task["status"]])}</td>'
            f'<td>{e(task["label"])}</td>'
            f'<td>{e(task.get("detail") or "")}</td>'
            f'<td>{e(str(started))}</td>'
            f'<td style="text-align:right">{dur}</td>'
            f'</tr>')

    feat_rows = []
    for f in snap['features']['list']:
        feat_rows.append(
            f'<tr><td>F{e(str(f["seed_id"]))}</td>'
            f'<td style="text-align:right">{f["x"]:+.3f}</td>'
            f'<td style="text-align:right">{f["y"]:+.3f}</td>'
            f'<td style="text-align:right">{f["confidence"]:.2f}</td>'
            f'<td style="font-size:11px;color:#888">{e(os_basename(f.get("hd_path","")))}</td></tr>')
    if not feat_rows:
        feat_rows.append('<tr><td colspan="5" style="color:#888">No features located.</td></tr>')

    event_rows = []
    for ev in snap['events'][:40]:
        ts = (ev.get('t_iso') or '')[-8:]
        event_rows.append(
            f'<tr><td style="color:#888;white-space:nowrap">{e(ts)}</td>'
            f'<td>{e(ev.get("text",""))}</td></tr>')

    failsafe_banner = ''
    if m.get('failsafe'):
        failsafe_banner = (
            f'<div class="banner">⚠ FAILSAFE: {e(m["failsafe"])}</div>')

    return f"""<!DOCTYPE html>
<html lang="en"><head><meta charset="utf-8">
<title>ASCEND Mission Report — {e(snap['team'])}</title>
<style>
  :root {{ color-scheme: dark; }}
  body {{ background:#0a0e17; color:#e6e9ef; font-family:'Segoe UI',Arial,sans-serif;
          margin:0; padding:32px; }}
  .wrap {{ max-width:1000px; margin:0 auto; }}
  h1 {{ margin:0 0 2px; font-size:26px; }}
  .sub {{ color:#8b95a7; margin-bottom:18px; }}
  .verdict {{ display:inline-block; padding:8px 18px; border-radius:8px;
             font-weight:700; color:#0a0e17; font-size:18px; margin:8px 0 22px; }}
  .grid {{ display:grid; grid-template-columns:1fr 1fr; gap:18px; }}
  .card {{ background:#121826; border:1px solid #20293c; border-radius:10px; padding:16px; }}
  .card h2 {{ margin:0 0 12px; font-size:15px; text-transform:uppercase;
             letter-spacing:1px; color:#7f8fb0; }}
  table {{ width:100%; border-collapse:collapse; font-size:13px; }}
  th,td {{ padding:6px 8px; border-bottom:1px solid #20293c; text-align:left; }}
  th {{ color:#7f8fb0; font-weight:600; }}
  .kv {{ display:flex; justify-content:space-between; padding:5px 0;
        border-bottom:1px solid #1a2233; font-size:14px; }}
  .kv span:first-child {{ color:#8b95a7; }}
  .banner {{ background:#e74c3c; color:#fff; padding:10px 16px; border-radius:8px;
            font-weight:700; margin:12px 0; }}
  .full {{ grid-column:1 / -1; }}
  footer {{ margin-top:26px; color:#5b6577; font-size:12px; text-align:center; }}
  @media print {{ body {{ background:#fff; color:#000; }}
                 .card {{ border-color:#ccc; }} }}
</style></head>
<body><div class="wrap">
  <h1>ASCEND — Elimination Round Result Report</h1>
  <div class="sub">Team {e(snap['team'])} &nbsp;•&nbsp; IRoC-U 2026 &nbsp;•&nbsp;
      Generated {e(snap['generated'])}</div>
  <div class="verdict" style="background:{vcolor}">{e(verdict)}</div>
  {failsafe_banner}

  <div class="grid">
    <div class="card">
      <h2>Mission Summary</h2>
      <div class="kv"><span>Final state</span><span>{e(m['state'])} — {e(m['state_label'])}</span></div>
      <div class="kv"><span>Mission duration</span><span>{_fmt_elapsed(m['elapsed_s'])}</span></div>
      <div class="kv"><span>Tasks completed</span><span>{m['tasks_done']}/{m['tasks_total']}</span></div>
      <div class="kv"><span>Features located</span><span>{snap['features']['found']}/{snap['features']['total']}</span></div>
      <div class="kv"><span>Survey coverage</span><span>{e(snap['survey']['progress'])} ({snap['survey']['pct']:.0f}%)</span></div>
      <div class="kv"><span>Final battery</span><span>{t['battery']:.1f}%</span></div>
      <div class="kv"><span>Arena</span><span>{snap['arena']['x']:.2f} × {snap['arena']['y']:.2f} m</span></div>
    </div>
    <div class="card">
      <h2>Arena Map</h2>
      <div style="text-align:center">{svg}</div>
    </div>

    <div class="card full">
      <h2>Task Performance (7 Elimination Tasks)</h2>
      <table>
        <tr><th>Status</th><th>Task</th><th>Detail</th><th>Started</th><th style="text-align:right">Duration</th></tr>
        {''.join(task_rows)}
      </table>
    </div>

    <div class="card">
      <h2>Determined Feature Coordinates</h2>
      <table>
        <tr><th>ID</th><th style="text-align:right">X (m)</th><th style="text-align:right">Y (m)</th>
            <th style="text-align:right">Conf.</th><th>Matched LR image</th></tr>
        {''.join(feat_rows)}
      </table>
    </div>
    <div class="card">
      <h2>Data Validation</h2>
      <p style="font-size:13px;color:#b5bdcc;line-height:1.5">
        Each located feature was confirmed by matching the on-board HD capture
        against the pre-uploaded <b>128×128 low-resolution (LR)</b> seed images
        using SIFT feature matching with RANSAC inlier verification. The
        <i>Conf.</i> column is the normalised inlier ratio; the matched LR seed
        filename identifies which reference image the detection corresponds to.
        See Video-2 / design report for the full LR generation &amp; comparison
        methodology.</p>
      <div class="kv"><span>Validated features</span><span>{snap['features']['found']}/{snap['features']['total']}</span></div>
    </div>

    <div class="card full">
      <h2>Mission Timeline</h2>
      <table>{''.join(event_rows)}</table>
    </div>
  </div>
  <footer>ASCEND autonomous mission control — result_generation dashboard ·
     {e(snap['generated'])}</footer>
</div></body></html>"""


def os_basename(path: str) -> str:
    return os.path.basename(path) if path else ''


# ════════════════════════════════════════════════════════════════════════════
#  PDF report (optional — ReportLab)
# ════════════════════════════════════════════════════════════════════════════
def render_pdf(snap: dict, pdf_path: str) -> bool:
    """Render a PDF with ReportLab. Returns False (no crash) if unavailable."""
    try:
        from reportlab.lib import colors
        from reportlab.lib.pagesizes import A4
        from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
        from reportlab.lib.units import mm
        from reportlab.platypus import (
            SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle)
    except Exception:
        return False

    styles = getSampleStyleSheet()
    title = ParagraphStyle('t', parent=styles['Title'], fontSize=18)
    h2 = ParagraphStyle('h2', parent=styles['Heading2'], fontSize=12,
                        textColor=colors.HexColor('#1f3a5f'))
    small = ParagraphStyle('s', parent=styles['Normal'], fontSize=9)

    m = snap['mission']
    t = snap['telemetry']
    verdict, vcolor = _verdict(snap)
    story = []

    story.append(Paragraph('ASCEND — Elimination Round Result Report', title))
    story.append(Paragraph(
        f"Team {snap['team']} &nbsp; • &nbsp; IRoC-U 2026 &nbsp; • &nbsp; "
        f"Generated {snap['generated']}", small))
    story.append(Spacer(1, 6))
    vp = ParagraphStyle('v', parent=styles['Normal'], fontSize=13,
                        textColor=colors.white, backColor=colors.HexColor(vcolor),
                        alignment=1, spaceBefore=4, spaceAfter=4, leading=22)
    story.append(Paragraph(f'<b>{verdict}</b>', vp))
    if m.get('failsafe'):
        fb = ParagraphStyle('fb', parent=small, textColor=colors.white,
                            backColor=colors.HexColor('#e74c3c'), alignment=1, leading=16)
        story.append(Paragraph(f'⚠ FAILSAFE: {m["failsafe"]}', fb))
    story.append(Spacer(1, 10))

    # Summary table
    summary = [
        ['Final state', f"{m['state']} — {m['state_label']}"],
        ['Mission duration', _fmt_elapsed(m['elapsed_s'])],
        ['Tasks completed', f"{m['tasks_done']}/{m['tasks_total']}"],
        ['Features located', f"{snap['features']['found']}/{snap['features']['total']}"],
        ['Survey coverage', f"{snap['survey']['progress']} ({snap['survey']['pct']:.0f}%)"],
        ['Final battery', f"{t['battery']:.1f}%"],
        ['Arena', f"{snap['arena']['x']:.2f} x {snap['arena']['y']:.2f} m"],
    ]
    story.append(Paragraph('Mission Summary', h2))
    story.append(_kv_table(summary, Table, TableStyle, colors, mm))
    story.append(Spacer(1, 12))

    # Task table
    story.append(Paragraph('Task Performance (7 Elimination Tasks)', h2))
    task_data = [['Status', 'Task', 'Detail', 'Duration']]
    style_rows = []
    for i, task in enumerate(snap['tasks'], start=1):
        dur = f'{task["duration_s"]:.0f}s' if task.get('duration_s') is not None else '—'
        task_data.append([STATUS_WORD[task['status']], task['label'],
                          task.get('detail') or '', dur])
        style_rows.append(('TEXTCOLOR', (0, i), (0, i),
                          colors.HexColor(STATUS_COLOR[task['status']])))
    tbl = Table(task_data, colWidths=[26 * mm, 42 * mm, 70 * mm, 18 * mm])
    tbl.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#1f3a5f')),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('FONTSIZE', (0, 0), (-1, -1), 8),
        ('GRID', (0, 0), (-1, -1), 0.4, colors.HexColor('#cccccc')),
        ('VALIGN', (0, 0), (-1, -1), 'MIDDLE'),
        ('FONTNAME', (0, 1), (0, -1), 'Helvetica-Bold'),
    ] + style_rows))
    story.append(tbl)
    story.append(Spacer(1, 12))

    # Feature table
    story.append(Paragraph('Determined Feature Coordinates', h2))
    feat_data = [['ID', 'X (m)', 'Y (m)', 'Conf.', 'Matched LR image']]
    for f in snap['features']['list']:
        feat_data.append([f'F{f["seed_id"]}', f'{f["x"]:+.3f}', f'{f["y"]:+.3f}',
                         f'{f["confidence"]:.2f}', os_basename(f.get('hd_path', ''))])
    if len(feat_data) == 1:
        feat_data.append(['—', '—', '—', '—', 'No features located'])
    ftbl = Table(feat_data, colWidths=[16 * mm, 24 * mm, 24 * mm, 18 * mm, 74 * mm])
    ftbl.setStyle(TableStyle([
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#1f3a5f')),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('FONTSIZE', (0, 0), (-1, -1), 8),
        ('GRID', (0, 0), (-1, -1), 0.4, colors.HexColor('#cccccc')),
    ]))
    story.append(ftbl)
    story.append(Spacer(1, 12))

    story.append(Paragraph('Data Validation', h2))
    story.append(Paragraph(
        'Each located feature was confirmed by matching the on-board HD capture '
        'against the pre-uploaded 128x128 low-resolution (LR) seed images using '
        'SIFT + RANSAC inlier verification. The confidence value is the '
        'normalised inlier ratio; the matched LR seed filename identifies the '
        'reference image. See Video-2 / design report for the full LR '
        'generation and comparison methodology.', small))

    try:
        SimpleDocTemplate(
            pdf_path, pagesize=A4,
            leftMargin=18 * mm, rightMargin=18 * mm,
            topMargin=16 * mm, bottomMargin=16 * mm,
        ).build(story)
        return True
    except Exception:
        return False


def _kv_table(rows, Table, TableStyle, colors, mm):
    tbl = Table(rows, colWidths=[45 * mm, 110 * mm])
    tbl.setStyle(TableStyle([
        ('FONTSIZE', (0, 0), (-1, -1), 9),
        ('TEXTCOLOR', (0, 0), (0, -1), colors.HexColor('#555555')),
        ('LINEBELOW', (0, 0), (-1, -1), 0.3, colors.HexColor('#dddddd')),
        ('TOPPADDING', (0, 0), (-1, -1), 3),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3),
    ]))
    return tbl


if __name__ == '__main__':
    # Quick smoke test with a synthetic snapshot.
    import sys
    demo = {
        'team': 'ASCEND', 'generated': datetime.now().isoformat(timespec='seconds'),
        'arena': {'x': 10.67, 'y': 7.62, 'dock_x': 0.0, 'dock_y': 0.0},
        'mission': {'state': 'COMPLETE', 'state_label': 'Mission finished',
                    'elapsed_s': 372, 'started': True, 'complete': True,
                    'failsafe': '', 'in_failsafe': False, 'tasks_done': 7,
                    'tasks_total': 7, 'any_failed': False},
        'telemetry': {'x': 0.1, 'y': 0.05, 'z': 0.0, 'pose_source': 'Optical Flow',
                      'battery': 96.0, 'battery_seen': True, 'battery_low': False},
        'survey': {'done_wp': 20, 'total_wp': 20, 'progress': '20/20', 'pct': 100.0},
        'features': {'found': 3, 'total': 3, 'list': [
            {'seed_id': 1, 'x': 3.2, 'y': 1.8, 'confidence': 0.91, 'hd_path': '/x/lr_a.jpg'},
            {'seed_id': 2, 'x': 7.1, 'y': 4.4, 'confidence': 0.83, 'hd_path': '/x/lr_b.jpg'},
            {'seed_id': 3, 'x': 5.0, 'y': 6.0, 'confidence': 0.77, 'hd_path': '/x/lr_c.jpg'}]},
        'tasks': [{'key': k, 'label': l, 'status': 'done', 'started_at': 1, 'completed_at': 2,
                   'started_iso': '2026-06-21T10:00:00', 'completed_iso': '2026-06-21T10:01:00',
                   'duration_s': 30.0, 'detail': 'ok'} for k, l in [
            ('takeoff', 'Autonomous Take-Off'), ('survey', 'Autonomous Survey'),
            ('coordinate', 'Coordinate Determination'), ('landing', 'Autonomous Landing'),
            ('charging', 'Autonomous Charging'), ('validation', 'Autonomous Data Validation'),
            ('reporting', 'Reporting of Result/Tasks')]],
        'trail': [[0, 0], [1, 1], [3, 2], [6, 4], [8, 5], [5, 6], [0, 0]],
        'events': [{'t': 1, 't_iso': '2026-06-21T10:00:00', 'kind': 'system', 'text': 'demo'}],
        'report_paths': {},
    }
    out = generate(demo, sys.argv[1] if len(sys.argv) > 1 else '/tmp/ascend_results_demo')
    print('Wrote:', json.dumps(out, indent=2))
