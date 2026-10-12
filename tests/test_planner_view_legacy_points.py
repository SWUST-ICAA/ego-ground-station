import os
import subprocess
import sys

import numpy as np


def test_legacy_point_lists_are_renderable():
    env = dict(os.environ, QT_QPA_PLATFORM="offscreen")
    code = (
        "from PyQt5 import QtWidgets; app=QtWidgets.QApplication([]); "
        "from ground_station.planner_view import PlannerView; view=PlannerView(); "
        "view.show_aircraft(1); view.canvas.layer='both'; view.canvas.data={'voxel_map': [[1, 2, 3]], "
        "'voxel_inflated': [[2, 3, 4]], 'position': [0, 0, 0]}; "
        "view.canvas.rebuild(); assert not view.canvas.render_error; "
        "assert len(view.canvas.meshes) == 2; print('ok')"
    )
    result = subprocess.run([sys.executable, "-c", code], env=env, text=True, capture_output=True)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "ok"


def test_draw_arrays_ignores_lost_opengl_context(monkeypatch):
    from OpenGL import error as gl_error
    from PyQt5 import QtWidgets
    from ground_station import planner_view

    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    canvas = planner_view.CloudCanvas()

    def fail(*args, **kwargs):
        raise gl_error.Error("no valid context")

    monkeypatch.setattr(planner_view.G, "glEnableClientState", fail)
    canvas.draw_arrays(np.zeros((1, 3), dtype=np.float32),
                       np.zeros((1, 3), dtype=np.float32), planner_view.G.GL_POINTS)


def test_main_selects_glx_backend_on_linux_x11():
    """PyOpenGL mis-detects EGL under WSLg and then every vertex-array call
    fails, so the entry point must pin the GLX backend on Linux/X11."""
    if not sys.platform.startswith("linux"):
        return
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env.pop("PYOPENGL_PLATFORM", None)
    env["DISPLAY"] = ":0"
    env["QT_QPA_PLATFORM"] = "offscreen"
    code = "import main, os; print(os.environ.get('PYOPENGL_PLATFORM'))"
    result = subprocess.run([sys.executable, "-c", code], env=env, text=True,
                            capture_output=True, cwd=root)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "glx"


def test_main_respects_explicit_pyopengl_platform():
    if not sys.platform.startswith("linux"):
        return
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    env = dict(os.environ)
    env["PYOPENGL_PLATFORM"] = "egl"
    env["DISPLAY"] = ":0"
    env["QT_QPA_PLATFORM"] = "offscreen"
    code = "import main, os; print(os.environ.get('PYOPENGL_PLATFORM'))"
    result = subprocess.run([sys.executable, "-c", code], env=env, text=True,
                            capture_output=True, cwd=root)
    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "egl"
