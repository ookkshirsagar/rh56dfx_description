#!/usr/bin/env python3
"""Render the README media (animated GIF + grasp-pose strip) from the URDF and STL meshes, off-screen with VTK.

usage:
  xacro urdf/rh56dfx.xacro side:=right > /tmp/rh56dfx_right.urdf
  python3 tools/render_media.py /tmp/rh56dfx_right.urdf meshes/right/visual docs/media

requires: vtk, numpy, pillow
"""
import math, os, sys, xml.etree.ElementTree as ET
import numpy as np
import vtk
from vtk.util.numpy_support import vtk_to_numpy
from PIL import Image

URDF, MESH_DIR, OUT = sys.argv[1:4]
os.makedirs(OUT, exist_ok=True)
W = H = 560
BG_TOP, BG_BOT = (0.10, 0.12, 0.15), (0.03, 0.04, 0.06)

root = ET.parse(URDF).getroot()
joints = {}
for j in root.findall("joint"):
    o = j.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    ax = j.find("axis")
    m = j.find("mimic")
    joints[j.get("name")] = dict(
        type=j.get("type"), parent=j.find("parent").get("link"), child=j.find("child").get("link"),
        xyz=xyz, rpy=rpy, axis=[float(v) for v in ax.get("xyz").split()] if ax is not None else [0, 0, 1],
        mimic=(m.get("joint"), float(m.get("multiplier", 1)), float(m.get("offset", 0))) if m is not None else None)
child_of = {j["child"]: n for n, j in joints.items()}
links = {}
for l in root.findall("link"):
    vis = l.find("visual")
    if vis is not None and vis.find("geometry/mesh") is not None:
        links[l.get("name")] = os.path.basename(vis.find("geometry/mesh").get("filename"))


def rpy_m(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def axis_m(a, q):
    a = np.array(a) / np.linalg.norm(a)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + math.sin(q) * K + (1 - math.cos(q)) * K @ K


def fk(q):
    cache = {}

    def T(link):
        if link in cache:
            return cache[link]
        if link not in child_of:
            M = np.eye(4)
        else:
            j = joints[child_of[link]]
            M = np.eye(4)
            M[:3, :3] = rpy_m(*j["rpy"])
            M[:3, 3] = j["xyz"]
            if j["type"] in ("revolute", "continuous"):
                name = child_of[link]
                if j["mimic"]:
                    src, mul, off = j["mimic"]
                    v = q.get(src, 0) * mul + off
                else:
                    v = q.get(name, 0)
                R = np.eye(4)
                R[:3, :3] = axis_m(j["axis"], v)
                M = M @ R
            M = T(j["parent"]) @ M
        cache[link] = M
        return M
    return {l: T(l) for l in links}


def color(link):
    if any(k in link for k in ("rubber", "pad")):
        return (0.09, 0.10, 0.11), 0.15
    if "tip" in link:
        return (0.12, 0.13, 0.14), 0.2
    if any(k in link for k in ("wrist", "hand_base", "palm")):
        return (0.74, 0.76, 0.79), 0.6
    return (0.86, 0.88, 0.90), 0.55


ren = vtk.vtkRenderer()
ren.GradientBackgroundOn()
ren.SetBackground(*BG_BOT)
ren.SetBackground2(*BG_TOP)
win = vtk.vtkRenderWindow()
win.SetOffScreenRendering(1)
win.SetSize(W, H)
win.SetMultiSamples(8)
win.AddRenderer(ren)
ren.SetUseFXAA(True)

actors = {}
for link, fn in links.items():
    r = vtk.vtkSTLReader()
    r.SetFileName(os.path.join(MESH_DIR, fn))
    nrm = vtk.vtkPolyDataNormals()
    nrm.SetInputConnection(r.GetOutputPort())
    nrm.SetFeatureAngle(50)
    mp = vtk.vtkPolyDataMapper()
    mp.SetInputConnection(nrm.GetOutputPort())
    a = vtk.vtkActor()
    a.SetMapper(mp)
    c, spec = color(link)
    p = a.GetProperty()
    p.SetColor(*c)
    p.SetSpecular(spec)
    p.SetSpecularPower(40)
    p.SetDiffuse(0.85)
    p.SetAmbient(0.12)
    ren.AddActor(a)
    actors[link] = a

ren.RemoveAllLights()
for pos, inten, col in (((1, -1, 1.5), 1.0, (1, 0.98, 0.95)), ((-1.5, 0.5, 0.5), 0.45, (0.75, 0.9, 1.0)),
                        ((0, 1.5, -0.5), 0.6, (0.35, 0.85, 1.0))):
    lt = vtk.vtkLight()
    lt.SetLightTypeToSceneLight()
    lt.SetPosition(*pos)
    lt.SetFocalPoint(0, 0, 0)
    lt.SetIntensity(inten)
    lt.SetColor(*col)
    ren.AddLight(lt)


def pose(q):
    for link, M in fk(q).items():
        t = vtk.vtkTransform()
        t.SetMatrix(M.flatten().tolist())
        actors[link].SetUserTransform(t)


FINGERS = ["index", "middle", "ring", "little"]
P = next(n for n in joints if n.endswith("index_1_joint"))[:-len("index_1_joint")]   # joint-name prefix


def q_of(f, t1, t2):
    q = {f"{P}{n}_1_joint": f[i] if isinstance(f, list) else f for i, n in enumerate(FINGERS)}
    q[f"{P}thumb_1_joint"] = t1
    q[f"{P}thumb_2_joint"] = t2
    return q


cam = ren.GetActiveCamera()


def setup_camera(az, el):
    pose(q_of(0.0, 0.0, 0.0))
    ren.ResetCamera()
    cam.SetFocalPoint(*cam.GetFocalPoint())
    cam.SetViewUp(0, 0, 1)
    b = ren.ComputeVisiblePropBounds()
    c = [(b[0] + b[1]) / 2, (b[2] + b[3]) / 2, (b[4] + b[5]) / 2]
    cam.SetFocalPoint(*c)
    cam.SetPosition(c[0] + 0.45, c[1], c[2])
    cam.SetViewUp(0, 0, 1)
    cam.Azimuth(az)
    cam.Elevation(el)
    ren.ResetCamera()
    cam.Zoom(1.05)
    return c


def grab():
    win.Render()
    w2i = vtk.vtkWindowToImageFilter()
    w2i.SetInput(win)
    w2i.SetInputBufferTypeToRGB()
    w2i.ReadFrontBufferOff()
    w2i.Update()
    img = w2i.GetOutput()
    x, y, _ = img.GetDimensions()
    arr = vtk_to_numpy(img.GetPointData().GetScalars()).reshape(y, x, 3)[::-1]
    return Image.fromarray(arr.copy())


def smooth(u):
    return u * u * (3 - 2 * u)


# ---- animated GIF: open -> power grasp -> open, camera gently orbiting ----
AZ0, EL0 = -35, 18
setup_camera(AZ0, EL0)
base_cam = (cam.GetPosition(), cam.GetFocalPoint(), cam.GetViewUp())
frames = []
N = 48
for i in range(N):
    u = i / N
    g = smooth(min(1, max(0, (math.sin(2 * math.pi * u - math.pi / 2) + 1) / 2)))
    pose(q_of(1.25 * g, 0.2 + 0.95 * g, 0.42 * g))
    cam.SetPosition(*base_cam[0]); cam.SetFocalPoint(*base_cam[1]); cam.SetViewUp(*base_cam[2])
    cam.Azimuth(28 * math.sin(2 * math.pi * u))
    frames.append(grab().resize((440, 440), Image.LANCZOS))
frames[0].save(os.path.join(OUT, "rh56dfx.gif"), save_all=True, append_images=frames[1:], duration=60, loop=0,
               optimize=True, disposal=2)

# ---- pose strip: open / pinch / tripod / power ----
POSES = [
    ("open", q_of(0.0, 0.0, 0.0)),
    ("pinch", q_of([1.0, 0.25, 0.25, 0.25], 1.15, 0.35)),
    ("tripod", q_of([0.95, 0.95, 0.2, 0.2], 1.15, 0.38)),
    ("power", q_of(1.3, 1.2, 0.48)),
]
cam.SetPosition(*base_cam[0]); cam.SetFocalPoint(*base_cam[1]); cam.SetViewUp(*base_cam[2])
tiles = []
for name, q in POSES:
    pose(q)
    im = grab()
    tiles.append(im.resize((360, 360), Image.LANCZOS))
strip = Image.new("RGB", (360 * 4, 360))
for i, t in enumerate(tiles):
    strip.paste(t, (360 * i, 0))
strip.save(os.path.join(OUT, "grasp_poses.png"), optimize=True)
print("ok", len(frames), "frames")
