"""Installation settings; importing this module never starts hardware control."""
import os
from pathlib import Path
import sys

# Writable data belongs to the selected workspace, never to site-packages.
ROOT = Path(os.environ.get('G1D_VR_WORKSPACE', Path.cwd())).expanduser().resolve()
SOURCE_ROOT = Path(__file__).resolve().parents[1]


def configured_path(name, default):
    value = Path(os.environ.get(name, str(default))).expanduser()
    return value if value.is_absolute() else ROOT / value


PYTHON = str(configured_path('G1D_VR_PYTHON', sys.executable))
CAMERA_PYTHON = str(configured_path('G1D_VR_CAMERA_PYTHON', PYTHON))
XR_ROOT = configured_path('G1D_VR_XR_ROOT', 'deps/xr_teleoperate')
CLIENT_SOURCE = configured_path('G1D_VR_CLIENT_SOURCE', 'deps/ial_g1d/src')
URDF = configured_path('G1D_VR_URDF', 'deps/xr_teleoperate/assets/g1/g1_body29_hand14.urdf')
RECORD_ROOT = configured_path('G1D_VR_RECORD_ROOT', 'records')
STATE_ROOT = configured_path('G1D_VR_STATE_ROOT', 'state')
INPUT_ROOT = configured_path('G1D_VR_INPUT_ROOT', STATE_ROOT / 'input')
CERT_FILE = configured_path('G1D_VR_CERT_FILE', 'certs/cert.pem')
KEY_FILE = configured_path('G1D_VR_KEY_FILE', 'certs/key.pem')
DDS_INTERFACE = os.environ.get('G1D_VR_DDS_INTERFACE', 'eth0')
ROBOT_HOST = os.environ.get('G1D_VR_ROBOT_HOST', '127.0.0.1')

# An installed client/XR package can also satisfy these imports. Configured
# source checkouts are appended, so they do not silently shadow installed code.
for source_path in (CLIENT_SOURCE, XR_ROOT):
    if str(source_path) not in sys.path:
        sys.path.append(str(source_path))


def child_environment(*, state_root=None):
    """Keep package imports and resolved workspace stable across child cwd."""
    env = dict(os.environ)
    env.update(
        G1D_VR_WORKSPACE=str(ROOT),
        G1D_VR_STATE_ROOT=str(STATE_ROOT if state_root is None else state_root),
        G1D_VR_RECORD_ROOT=str(RECORD_ROOT),
        G1D_VR_INPUT_ROOT=str(INPUT_ROOT),
        OPENBLAS_NUM_THREADS='1', OMP_NUM_THREADS='1', MKL_NUM_THREADS='1',
    )
    paths = [str(CLIENT_SOURCE), str(XR_ROOT)]
    # Editable checkouts can share their own source package with a recorder
    # interpreter. A wheel must be installed in each interpreter instead of
    # exposing the control environment's entire site-packages to the camera.
    if SOURCE_ROOT.name == 'src' and (SOURCE_ROOT.parent / 'pyproject.toml').is_file():
        paths.insert(0, str(SOURCE_ROOT))
    if env.get('PYTHONPATH'):
        paths.append(env['PYTHONPATH'])
    env['PYTHONPATH'] = os.pathsep.join(paths)
    return env
