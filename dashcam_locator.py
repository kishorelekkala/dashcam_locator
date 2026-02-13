"""
QGIS Dashcam Viewer (Click-to-Open Frames)

Usage:
1) Run the script in the QGIS Python console.
2) Click the camera icon in the toolbar.
3) The first time, it will ask you to select:
   - the geolocated_videos.html file
   - the Frames Root folder
4) After selecting the paths, click anywhere on the map near a route to open the
   closest dashcam frame sequence in the dock widget.

"""

import os
import re
from pathlib import Path
from collections import defaultdict

from qgis.PyQt.QtCore import Qt, QTimer, QSettings
from qgis.PyQt.QtGui import QPixmap, QIcon
from qgis.PyQt.QtWidgets import (
    QAction, QMessageBox, QDockWidget, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QPushButton, QSlider, QFileDialog
)
from qgis.gui import QgsMapToolEmitPoint
from qgis.core import (
    QgsProject, QgsPointXY, QgsCoordinateTransform, QgsCoordinateReferenceSystem
)

# =============================================================================
# CONFIG (SAFE DEFAULTS)
# =============================================================================

# Click-to-coverage snap threshold (meters; best with a metric project CRS like EPSG:25833)
NEAREST_SEARCH_METERS = 25.0

# Store every Nth polyline vertex.
# 1 = max accuracy (slower), 3 = good default, 5 = faster but less precise.
POLYLINE_STRIDE = 3

# Grid acceleration cell size in meters (controls nearest search speed)
GRID_CELL_M = 150.0

# UI autoplay speed (ms per frame)
PLAY_INTERVAL_MS = 120

# QSettings keys (persist across sessions)
SETTINGS_GROUP = "DashcamViewer"
SETTINGS_HTML_KEY = f"{SETTINGS_GROUP}/html_path"
SETTINGS_FRAMES_KEY = f"{SETTINGS_GROUP}/frames_root"

# =============================================================================
# REGEX / FORMATS
# =============================================================================

IMG_RE = re.compile(r"frame_(\d+)\.(jpg|jpeg|png)$", re.IGNORECASE)

FOLDER_A = re.compile(r"^(\d{8}_\d{6})_")             # 20251114_084413_0303_N_A
FOLDER_B = re.compile(r"^frames-(\d{8})T(\d{6})Z-")   # frames-20251114T084413Z-...

# HTML parsing patterns
MARKER_DEF_RE = re.compile(
    r"var\s+(marker_[A-Za-z0-9_]+)\s*=\s*L\.marker\(\s*\[\s*([0-9\.\-]+)\s*,\s*([0-9\.\-]+)\s*\]",
    re.MULTILINE
)
POLY_DEF_RE = re.compile(
    r"var\s+(poly_line_[A-Za-z0-9_]+)\s*=\s*L\.polyline\(\s*\[\[(.*?)\]\]\s*,\s*\{",
    re.DOTALL
)
PAIR_RE = re.compile(r"\[\s*([0-9\.\-]+)\s*,\s*([0-9\.\-]+)\s*\]")
TOOLTIP_RE = re.compile(
    r"([A-Za-z0-9_]+)\.bindTooltip\(\s*`[^`]*<b>([^<]+)</b>",
    re.MULTILINE
)

# =============================================================================
# LOGGING / NOTIFICATION HELPERS
# =============================================================================

def _status(msg: str, timeout_ms: int = 4000):
    """Write a small, non-intrusive message in QGIS status bar."""
    try:
        iface.mainWindow().statusBar().showMessage(msg, timeout_ms)
    except Exception:
        pass


def _warn(title: str, msg: str):
    QMessageBox.warning(None, title, msg)


def _info(title: str, msg: str):
    QMessageBox.information(None, title, msg)


# =============================================================================
# INPUT SELECTION + SETTINGS
# =============================================================================

def ask_for_inputs(start_dir: str = ""):
    """
    Ask the user to select:
      1) geolocated_videos.html
      2) Frames Root folder
    """
    html_path, _ = QFileDialog.getOpenFileName(
        None,
        "Select geolocated_videos.html",
        start_dir or "",
        "HTML Files (*.html)"
    )
    if not html_path:
        return "", ""

    frames_root = QFileDialog.getExistingDirectory(
        None,
        "Select Frames Root Folder",
        str(Path(html_path).parent)
    )
    if not frames_root:
        return "", ""

    return html_path, frames_root


def load_paths_from_settings():
    s = QSettings()
    html_path = s.value(SETTINGS_HTML_KEY, "", type=str)
    frames_root = s.value(SETTINGS_FRAMES_KEY, "", type=str)
    return html_path, frames_root


def save_paths_to_settings(html_path: str, frames_root: str):
    s = QSettings()
    s.setValue(SETTINGS_HTML_KEY, html_path)
    s.setValue(SETTINGS_FRAMES_KEY, frames_root)


def paths_valid(html_path: str, frames_root: str) -> bool:
    return bool(html_path and frames_root and os.path.exists(html_path) and os.path.exists(frames_root))


# =============================================================================
# CORE HELPERS
# =============================================================================

def _key_from_runid(run_id: str) -> str:
    """RUN_ID: YYYYMMDD_HHMMSS_XXXX_N_A -> KEY: YYYYMMDD_HHMMSS"""
    m = re.search(r"(\d{8})_(\d{6})", run_id)
    return f"{m.group(1)}_{m.group(2)}" if m else ""


def build_frames_index(frames_root: str) -> dict:
    """
    Index all available frame folders under FRAMES_ROOT:
      KEY 'YYYYMMDD_HHMMSS' -> [folder_paths]
    """
    root = Path(frames_root)
    if not root.exists():
        raise FileNotFoundError(f"Frames root not found: {frames_root}")

    idx = {}
    for p in root.rglob("*"):
        if not p.is_dir():
            continue

        name = p.name

        mA = FOLDER_A.match(name)
        if mA:
            idx.setdefault(mA.group(1), []).append(str(p))
            continue

        mB = FOLDER_B.match(name)
        if mB:
            idx.setdefault(f"{mB.group(1)}_{mB.group(2)}", []).append(str(p))
            continue

    return idx


def list_frames_in_folder(folder_path: str):
    """Return sorted list of frame files (recursive)."""
    folder = Path(folder_path)
    files = []
    for f in folder.rglob("*"):
        if f.is_file() and IMG_RE.search(f.name):
            files.append(str(f))

    def key_fn(fp):
        m = IMG_RE.search(Path(fp).name)
        return int(m.group(1)) if m else 10**18

    files.sort(key=key_fn)
    return files


def pick_best_folder(folder_paths):
    """If multiple folders match same key, choose folder with most frames."""
    best = folder_paths[0]
    best_count = -1
    for fp in folder_paths:
        try:
            count = sum(1 for f in Path(fp).rglob("*") if f.is_file() and IMG_RE.search(f.name))
        except Exception:
            count = -1
        if count > best_count:
            best = fp
            best_count = count
    return best


def parse_html_objects(html_path: str):
    """
    Parse HTML and return:
      - obj_to_runid: object_name -> RUN_ID (from bindTooltip <b>RUN_ID</b>)
      - markers: [(obj_name, lon, lat), ...]
      - polylines: [(obj_name, [(lon,lat),...]), ...]
    """
    with open(html_path, "r", encoding="utf-8", errors="ignore") as f:
        txt = f.read()

    obj_to_runid = {}
    for m in TOOLTIP_RE.finditer(txt):
        obj_to_runid[m.group(1).strip()] = m.group(2).strip()

    markers = []
    for m in MARKER_DEF_RE.finditer(txt):
        obj = m.group(1)
        lat = float(m.group(2))
        lon = float(m.group(3))
        markers.append((obj, lon, lat))

    polylines = []
    for m in POLY_DEF_RE.finditer(txt):
        obj = m.group(1)
        inside = m.group(2)
        pairs = PAIR_RE.findall(inside)
        coords = []
        for lat_s, lon_s in pairs:
            try:
                coords.append((float(lon_s), float(lat_s)))
            except Exception:
                pass
        if len(coords) >= 2:
            polylines.append((obj, coords))

    return obj_to_runid, markers, polylines


def build_click_index(obj_to_runid, markers, polylines, project_crs):
    """
    Build a grid index of snap points in project CRS.
    Polyline points contain 't' progress along the drive (0..1),
    enabling approximate frame jump.
    """
    wgs84 = QgsCoordinateReferenceSystem("EPSG:4326")
    xform = QgsCoordinateTransform(wgs84, project_crs, QgsProject.instance())

    grid = defaultdict(list)
    total_points = 0
    linked_polylines = 0
    linked_markers = 0

    # Polylines
    stride = max(1, int(POLYLINE_STRIDE))
    for obj, coords in polylines:
        run_id = obj_to_runid.get(obj, "")
        if not run_id:
            continue
        key = _key_from_runid(run_id)
        if not key:
            continue

        linked_polylines += 1
        n = len(coords)

        for i in range(0, n, stride):
            lon, lat = coords[i]
            t = i / (n - 1) if n > 1 else 0.0
            try:
                p = xform.transform(QgsPointXY(lon, lat))
            except Exception:
                continue

            x, y = p.x(), p.y()
            ix = int(x // GRID_CELL_M)
            iy = int(y // GRID_CELL_M)
            grid[(ix, iy)].append({
                "x": x, "y": y,
                "run_id": run_id,
                "key": key,
                "t": t,
                "source": "polyline"
            })
            total_points += 1

    # Markers (fallback snap)
    for obj, lon, lat in markers:
        run_id = obj_to_runid.get(obj, "")
        if not run_id:
            continue
        key = _key_from_runid(run_id)
        if not key:
            continue

        linked_markers += 1
        try:
            p = xform.transform(QgsPointXY(lon, lat))
        except Exception:
            continue

        x, y = p.x(), p.y()
        ix = int(x // GRID_CELL_M)
        iy = int(y // GRID_CELL_M)
        grid[(ix, iy)].append({
            "x": x, "y": y,
            "run_id": run_id,
            "key": key,
            "t": None,
            "source": "marker"
        })
        total_points += 1

    return grid, total_points, linked_polylines, linked_markers


def nearest_point(click_x, click_y, grid):
    """Find nearest point using 3x3 neighbor cells."""
    ix = int(click_x // GRID_CELL_M)
    iy = int(click_y // GRID_CELL_M)

    best = None
    best_d = 1e18

    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            pts = grid.get((ix + dx), None)
            pts = grid.get((ix + dx, iy + dy), [])
            for p in pts:
                ddx = p["x"] - click_x
                ddy = p["y"] - click_y
                d = (ddx * ddx + ddy * ddy) ** 0.5
                if d < best_d:
                    best_d = d
                    best = p

    return best, best_d


# =============================================================================
# UI: Dock-based viewer
# =============================================================================

class DashcamDock(QDockWidget):
    def __init__(self, parent=None):
        super().__init__("Dashcam Viewer", parent)
        self.setObjectName("DashcamViewerDock")

        self.frame_files = []
        self.i = 0

        container = QWidget()
        v = QVBoxLayout(container)

        self.title = QLabel("Ready. Click camera icon → click on route.")
        self.title.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.img = QLabel("No image")
        self.img.setAlignment(Qt.AlignCenter)

        self.info = QLabel("")
        self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)

        self.btn_prev = QPushButton("◀ Prev")
        self.btn_next = QPushButton("Next ▶")
        self.btn_play = QPushButton("▶ Play")
        self.btn_stop = QPushButton("⏸ Stop")

        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(0)
        self.slider.setValue(0)

        row = QHBoxLayout()
        row.addWidget(self.btn_prev)
        row.addWidget(self.btn_next)
        row.addWidget(self.btn_play)
        row.addWidget(self.btn_stop)

        v.addWidget(self.title)
        v.addWidget(self.img, stretch=1)
        v.addWidget(self.slider)
        v.addWidget(self.info)
        v.addLayout(row)

        self.setWidget(container)

        self.btn_prev.clicked.connect(self.prev)
        self.btn_next.clicked.connect(self.next)
        self.slider.valueChanged.connect(self.on_slider)

        self.timer = QTimer(self)
        self.timer.setInterval(PLAY_INTERVAL_MS)
        self.timer.timeout.connect(self.next)

        self.btn_play.clicked.connect(self.play)
        self.btn_stop.clicked.connect(self.stop)

    def set_frames(self, frame_files, start_index=0, title_text=""):
        self.frame_files = frame_files or []
        self.i = max(0, min(start_index, len(self.frame_files) - 1)) if self.frame_files else 0

        self.slider.blockSignals(True)
        self.slider.setMinimum(0)
        self.slider.setMaximum(max(0, len(self.frame_files) - 1))
        self.slider.setValue(self.i)
        self.slider.blockSignals(False)

        if title_text:
            self.title.setText(title_text)

        self.stop()
        self.render()

    def render(self):
        if not self.frame_files:
            self.img.setText("No frames found.")
            self.info.setText("")
            return

        fp = self.frame_files[self.i]
        pix = QPixmap(fp)
        if pix.isNull():
            self.img.setText(f"Failed to load:\n{fp}")
        else:
            self.img.setPixmap(pix.scaled(self.img.size(), Qt.KeepAspectRatio, Qt.SmoothTransformation))

        self.info.setText(f"{self.i + 1}/{len(self.frame_files)}\n{fp}")

        self.slider.blockSignals(True)
        self.slider.setValue(self.i)
        self.slider.blockSignals(False)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        self.render()

    def prev(self):
        if self.frame_files:
            self.i = max(0, self.i - 1)
            self.render()

    def next(self):
        if self.frame_files:
            self.i = min(len(self.frame_files) - 1, self.i + 1)
            self.render()

    def on_slider(self, val):
        if self.frame_files:
            self.i = max(0, min(val, len(self.frame_files) - 1))
            self.render()

    def play(self):
        if self.frame_files:
            self.timer.start()

    def stop(self):
        self.timer.stop()


# =============================================================================
# Map Tool: click-to-open
# =============================================================================

class DashcamMapTool(QgsMapToolEmitPoint):
    def __init__(self, canvas, grid, frames_index, dock: DashcamDock):
        super().__init__(canvas)
        self.canvas = canvas
        self.grid = grid
        self.frames_index = frames_index
        self.dock = dock

    def canvasReleaseEvent(self, e):
        click_pt = self.toMapCoordinates(e.pos())  # project CRS
        best, d_m = nearest_point(click_pt.x(), click_pt.y(), self.grid)

        if not best or d_m > NEAREST_SEARCH_METERS:
            _status(f"Dashcam: no coverage within {NEAREST_SEARCH_METERS:.0f} m (nearest {d_m:.1f} m).")
            return

        key = best["key"]
        run_id = best["run_id"]
        t = best["t"]
        source = best["source"]

        folders = self.frames_index.get(key, [])
        if not folders:
            _status(f"Dashcam: no local folder for key {key}.")
            _warn(
                "Dashcam Viewer",
                f"No local frames folder found for:\n{key}\n\nFrom HTML:\n{run_id}\n\n"
                "Check if the session is missing in your local dataset."
            )
            return

        folder = pick_best_folder(folders)
        frames = list_frames_in_folder(folder)
        if not frames:
            _status("Dashcam: folder found but no frames inside.")
            _warn("Dashcam Viewer", f"Folder found but no frames inside:\n{folder}")
            return

        # Jump to an approximate frame:
        # - For polyline points: use progress t
        # - For marker points: open mid-point
        if t is not None:
            start_idx = int(max(0.0, min(1.0, t)) * (len(frames) - 1))
        else:
            start_idx = len(frames) // 2

        title = f"{run_id}  |  key={key}  |  {source}  |  d≈{d_m:.1f} m"
        self.dock.set_frames(frames, start_index=start_idx, title_text=title)
        self.dock.show()

        _status(f"Dashcam: opened {run_id} (jump {start_idx + 1}/{len(frames)}).")


# =============================================================================
# Controller: toolbar icons + lifecycle
# =============================================================================

class DashcamController:
    def __init__(self, iface):
        self.iface = iface
        self.canvas = iface.mapCanvas()

        self.action_viewer = None
        self.action_change_paths = None

        self.tool = None
        self.dock = None

        self.frames_index = None
        self.grid = None

        self.html_path = ""
        self.frames_root = ""

    def _ensure_paths(self, force_select: bool = False) -> bool:
        """
        Ensure valid paths exist.
        - If settings contain valid paths: use them.
        - Else (or if force_select): ask user once, then save in settings.
        """
        if not force_select:
            html_path, frames_root = load_paths_from_settings()
            if paths_valid(html_path, frames_root):
                self.html_path = html_path
                self.frames_root = frames_root
                return True

        # Ask user (one-time) and persist
        start_dir = ""
        prev_html, _ = load_paths_from_settings()
        if prev_html and os.path.exists(prev_html):
            start_dir = str(Path(prev_html).parent)

        html_path, frames_root = ask_for_inputs(start_dir)
        if not html_path or not frames_root:
            return False

        if not paths_valid(html_path, frames_root):
            _warn("Dashcam Viewer", "Selected paths are not valid. Please try again.")
            return False

        self.html_path = html_path
        self.frames_root = frames_root
        save_paths_to_settings(self.html_path, self.frames_root)
        return True

    def _build(self) -> bool:
        """(Re)build indexes and UI from current paths."""
        # Index local frames
        try:
            self.frames_index = build_frames_index(self.frames_root)
        except Exception as ex:
            _warn("Dashcam Viewer", f"Failed indexing frames:\n{ex}")
            return False

        if not self.frames_index:
            _warn("Dashcam Viewer", "No frame folders found under the selected Frames Root.")
            return False

        # Parse HTML
        obj_to_runid, markers, polylines = parse_html_objects(self.html_path)
        if not obj_to_runid:
            _warn("Dashcam Viewer", "No bindTooltip <b>RUN_ID</b> found in HTML.")
            return False

        # Dock UI
        if self.dock is None:
            self.dock = DashcamDock(self.iface.mainWindow())
            self.iface.addDockWidget(Qt.RightDockWidgetArea, self.dock)

        # Build snap grid
        project_crs = QgsProject.instance().crs()
        self.grid, total_pts, poly_ok, marker_ok = build_click_index(
            obj_to_runid, markers, polylines, project_crs
        )

        if not self.grid:
            _warn("Dashcam Viewer", "Failed to build snap index (grid is empty).")
            return False

        _status(
            f"Dashcam ready: {total_pts} snap points (poly={poly_ok}, marker={marker_ok}). "
            "Click camera icon, then click on a route."
        )
        return True

    def start(self):
        # Ensure paths (load from settings or ask once)
        ok = self._ensure_paths(force_select=False)
        if not ok:
            _status("Dashcam: cancelled (no valid paths).")
            return

        # Build indexes
        if not self._build():
            return

        # Toolbar actions (viewer + change paths)
        if self.action_viewer is None:
            icon = QIcon(":/images/themes/default/mIconCamera.svg")
            self.action_viewer = QAction(icon, "Dashcam Viewer", self.iface.mainWindow())
            self.action_viewer.setCheckable(True)
            self.action_viewer.toggled.connect(self.on_toggled)
            self.iface.addToolBarIcon(self.action_viewer)

        if self.action_change_paths is None:
            icon2 = QIcon(":/images/themes/default/mActionFileOpen.svg")
            self.action_change_paths = QAction(icon2, "Change Dashcam Paths", self.iface.mainWindow())
            self.action_change_paths.triggered.connect(self.change_paths)
            self.iface.addToolBarIcon(self.action_change_paths)

        _status("Dashcam loaded. Use camera icon to click/open frames. Use 'Change Dashcam Paths' to switch datasets.")

    def change_paths(self):
        # Turn off tool while reconfiguring
        if self.action_viewer and self.action_viewer.isChecked():
            self.action_viewer.setChecked(False)

        ok = self._ensure_paths(force_select=True)
        if not ok:
            _status("Dashcam: paths not changed.")
            return

        if not self._build():
            return

        _info("Dashcam Viewer", "Paths updated successfully.\n\nYou can now enable the camera tool and click on a route.")

    def on_toggled(self, checked: bool):
        if checked:
            if not self.grid or not self.frames_index:
                _warn("Dashcam Viewer", "Dashcam index not ready. Try 'Change Dashcam Paths' and load again.")
                if self.action_viewer:
                    self.action_viewer.setChecked(False)
                return

            self.tool = DashcamMapTool(self.canvas, self.grid, self.frames_index, self.dock)
            self.canvas.setMapTool(self.tool)
            if self.dock:
                self.dock.show()
            _status("Dashcam tool ON: click on route to open frames.")
        else:
            if self.tool:
                self.canvas.unsetMapTool(self.tool)
            self.tool = None
            _status("Dashcam tool OFF.")


# =============================================================================
# Bootstrap (keep global reference, remove old toolbar icons)
# =============================================================================

try:
    _DASHCAM_VIEWER
except NameError:
    _DASHCAM_VIEWER = None

# Remove old icons if rerun
if _DASHCAM_VIEWER:
    try:
        if getattr(_DASHCAM_VIEWER, "action_viewer", None):
            iface.removeToolBarIcon(_DASHCAM_VIEWER.action_viewer)
        if getattr(_DASHCAM_VIEWER, "action_change_paths", None):
            iface.removeToolBarIcon(_DASHCAM_VIEWER.action_change_paths)
    except Exception:
        pass

_DASHCAM_VIEWER = DashcamController(iface)
_DASHCAM_VIEWER.start()
