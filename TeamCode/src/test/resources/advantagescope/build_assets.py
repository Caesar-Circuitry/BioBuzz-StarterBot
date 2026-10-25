"""Build AdvantageScope assets for the BioBuzz sim from the official 2026-2027 field model.

Outputs (in ./assets):
  Field3d_BioBuzzSim/   field model with both hives removed + game piece models
  Robot_BioBuzzRedHive/ red hive, origin on its pivot axis, drawn level
  Robot_BioBuzzBlueHive/ blue hive, same
"""
import copy, json, os, shutil, sys
import numpy as np
from pygltflib import GLTF2

SRC = sys.argv[1]           # model.glb
ATT = sys.argv[2]           # folder with config.json + model_0..2.glb
OUT = "assets"
M = 39.3701
PEDRO_HALF = 141.5 / 2      # FTCodeSim's field-centre offset

def local(n):
    if n.matrix: return np.array(n.matrix, dtype=float).reshape(4, 4).T
    T = np.eye(4); R = np.eye(4); S = np.eye(4)
    if n.translation: T[:3, 3] = n.translation
    if n.rotation:
        x, y, z, w = n.rotation
        R[:3, :3] = [[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]]
    if n.scale: S[0, 0], S[1, 1], S[2, 2] = n.scale
    return T @ R @ S

RX90 = np.array([[1,0,0,0],[0,0,-1,0],[0,1,0,0],[0,0,0,1]], float)   # glTF Y-up -> Z-up (field config rotation)
PIVOT_Z = 43.95 / M

g = GLTF2().load(SRC)
parent = {c: i for i, n in enumerate(g.nodes) for c in (n.children or [])}
def world(i):
    chain = []
    while True:
        chain.append(i)
        if i not in parent: break
        i = parent[i]
    W = np.eye(4)
    for j in reversed(chain): W = W @ local(g.nodes[j])
    return W

hives = {}
for i, n in enumerate(g.nodes):
    if n.name and n.name.startswith("am-5853-Red Hive"): hives["Red"] = i
    if n.name and n.name.startswith("am-5853-Blue Hive"): hives["Blue"] = i
assert len(hives) == 2, hives

def hive_matrix(color):
    """glTF field coords of the hive subtree -> glTF coords of a robot model whose
    origin is on the pivot axis and which is drawn at hive angle 0 (level)."""
    i = hives[color]
    P = world(parent[i])                      # parent world transform (field glTF frame)
    # pivot in model (Z-up) frame: x = hive centre, y = 0, z = 43.95 in
    cx = 12.75 / M * (-1 if color == "Red" else 1)
    T = np.eye(4); T[:3, 3] = [-cx, 0, -PIVOT_Z]
    # current angle (radians): positive raises side +1 = +y. Red starts with -y raised.
    phi = np.radians(-30 if color == "Red" else 30)
    c, s = np.cos(phi), np.sin(phi)
    # model frame (x=across, y=along, z=up) -> level hive frame (across, along, up)
    U = np.eye(4)
    U[1, :3] = [0, c, s]      # a = along*cos + up*sin
    U[2, :3] = [0, -s, c]     # h = -along*sin + up*cos
    # level hive frame -> AdvantageScope data frame (D): Dx = -along, Dy = across, Dz = up
    D = np.array([[0,-1,0,0],[1,0,0,0],[0,0,1,0],[0,0,0,1]], float)
    # D (Z-up) -> robot glTF (Y-up), undone by the config's x+90 rotation: (Dx, Dz, -Dy)
    G = np.array([[1,0,0,0],[0,0,1,0],[0,-1,0,0],[0,0,0,1]], float)
    return G @ D @ U @ T @ RX90 @ P, (cx, phi)

def write_subset(out_path, root_indices, root_matrix=None):
    h = copy.deepcopy(g)
    if root_matrix is None:
        h.scenes[0].nodes = root_indices
    else:
        from pygltflib import Node
        wrapper = Node(name="BioBuzzSimRoot", matrix=[float(v) for v in root_matrix.T.flatten()], children=root_indices)
        h.nodes.append(wrapper)
        h.scenes[0].nodes = [len(h.nodes) - 1]
    h.save_binary(out_path)

if os.path.exists(OUT): shutil.rmtree(OUT)

# ---- hives ----
for color in ("Red", "Blue"):
    d = os.path.join(OUT, f"Robot_BioBuzz{color}Hive"); os.makedirs(d)
    Mx, info = hive_matrix(color)
    write_subset(os.path.join(d, "raw.glb"), [hives[color]], Mx)
    json.dump({
        "name": f"BioBuzz {color} Hive",
        "isFTC": True,
        "disableSimplification": False,
        "rotations": [{"axis": "x", "degrees": 90}],
        "position": [0, 0, 0],
        "cameras": [],
        "components": []
    }, open(os.path.join(d, "config.json"), "w"), indent=2)
    print(color, "pivot x (model in)", info[0] * M, "start angle deg", np.degrees(info[1]))

# ---- field without hives ----
d = os.path.join(OUT, "Field3d_BioBuzzSim"); os.makedirs(d)
h = copy.deepcopy(g)
root = parent[hives["Red"]]
assert root == parent[hives["Blue"]]
h.nodes[root].children = [c for c in h.nodes[root].children if c not in hives.values()]
h.save_binary(os.path.join(d, "raw.glb"))
cfg = json.load(open(os.path.join(ATT, "config.json")))
cfg["name"] = "2026-2027 Field (BioBuzz Sim)"
cfg.pop("locales", None)
for gp in cfg["gamePieces"]: gp.pop("locales", None)
json.dump(cfg, open(os.path.join(d, "config.json"), "w"), indent=2)
for k in range(3):
    shutil.copy(os.path.join(ATT, f"model_{k}.glb"), os.path.join(d, f"model_{k}.glb"))
print("field root", g.nodes[root].name, "children now", len(h.nodes[root].children))
