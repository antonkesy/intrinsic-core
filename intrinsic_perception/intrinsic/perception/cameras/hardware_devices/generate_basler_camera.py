# Copyright 2026 Intrinsic Innovation LLC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Procedural 3D visual and collision model generator for Basler camera bodies.

Features:
- Enclosure: Clean, watertight 29x29 mm chamfered body with flat bottom
  (no tripod boss/box).
- C-Mount Collar: Round collar for ace classic; stepped collar for ace 2
  (rear block continuing 29x29 mm body profile, front flat-cut collar with
  corner lobes).
- Connectors: Includes functional circular connector protrusion with
  exact CAD-verified dimensions (Hirose HR10 on ace classic, M8 6-pin on ace 2).
  RJ45 and status LEDs are excluded for a clean, lightweight simulation model.
- Watertight collision geometry covering body, collar, and connector.

Coordinate System Convention:
- Optical axis is along +Z (forward).
- +X is Camera Right (from camera perspective looking forward; -X when looking
  at rear face).
- +Y is Camera Down (conforming to Intrinsic/ROS optical convention:
  +Z fwd, +X right, +Y down).
  The camera bottom face (tripod mount) is along +Y, and the top face is along
  -Y.
- Origin (0, 0, 0) is at the C-Mount flange mounting plane.
- Camera body extends into negative Z:
    - ace (classic): C-Mount collar Z in [-12.0, 0.0] mm, body Z in
      [-54.0, -12.0] mm, connector sticks out 6.30 mm to Z = -60.30 mm.
    - ace 2: C-Mount collar Z in [-10.87, 0.0] mm (rear base Z in
      [-10.87, -6.58] mm, front cut collar Z in [-6.58, 0.0] mm), body Z in
      [-55.46, -10.87] mm, connector sticks out 6.71 mm to Z = -62.17 mm.
- Connector position (when looking at the rear face from behind):
    - ace (classic): Hirose HR10 on LOWER LEFT (-X = -7.90 mm, +Y = +6.20 mm).
    - ace 2: M8 6-pin on LOWER RIGHT (+X = +5.80 mm, +Y = +8.10 mm).
"""

import argparse
import dataclasses
import os
from typing import Dict
from typing import List
from typing import Optional
from typing import Tuple

import numpy as np
from scipy.spatial import Delaunay
import trimesh
from trimesh import creation


# -----------------------------------------------------------------------------
# Configurations (all dimensions in millimeters)
# -----------------------------------------------------------------------------
@dataclasses.dataclass
class BodyConfig:
  width: float = 29.0
  height: float = 29.0
  depth: float = 42.0  # ace classic; ace 2 is 44.59
  chamfer: float = 1.0  # Corner chamfer


@dataclasses.dataclass
class MountConfig:
  outer_diameter: float = 27.86  # Front flange (ace) or corner diameter (ace 2)
  collar_length: float = 12.0  # ace classic; ace 2 is 10.87
  inner_bore: float = 25.4  # 1-inch C-Mount thread
  outer_diameter_body: Optional[float] = None  # Body junction diameter (ace)
  collar_base_length: float = 0.0  # Rear collar block length (ace 2)
  baffle_length: float = 5.93  # Internal baffle tunnel
  baffle_inner_diameter: float = 16.0  # Inner bore of baffle
  baffle_step_z_offset: float = 0.40  # Step offset forward of body face
  has_flats: bool = False  # ace 2 has 4-sided flat cuts
  flat_width: float = 27.87  # Across X
  flat_height: float = 28.15  # Across Y


@dataclasses.dataclass
class ConnectorConfig:
  pos_x: float = 0.0
  pos_y: float = 0.0
  length: float = 6.30  # Protrusion length
  outer_diameter: float = 8.85  # Main barrel diameter
  inner_diameter: float = 7.25  # Inner bore diameter
  collar_diameter: float = 0.0  # Optional base collar / flange
  collar_length: float = 0.0


@dataclasses.dataclass
class CameraPreset:
  name: str
  description: str
  body: BodyConfig
  mount: MountConfig
  connector: Optional[ConnectorConfig] = None


def get_preset_catalog() -> Dict[str, CameraPreset]:
  """Returns catalog of Basler camera presets (dimensions in mm)."""
  catalog = {}

  # 1. Basler ace (classic) - matches drawing IB018802 01,
  #    STEP CAD "Basler ace GigE C-Mount v01.STP"
  # Body 29x29x42 mm, collar 12 mm -> total body 54.0 mm,
  # Hirose sticks out 6.30 mm (total 60.30 mm).
  # Collar outer surface has 1.5 deg draft angle:
  # dia 27.86 mm at front, dia 28.49 mm at body.
  # Internal cavity: C-Mount thread bore, step at Z = -11.60 mm to dia 16.0 mm,
  # tunnel length 5.93 mm to sensor.
  # Rear layout (looking at rear face):
  # Hirose on LOWER LEFT (-X = -7.90 mm, +Y = +6.20 mm)
  # Hirose: HR10A-7R-6PB (8.85 mm dia barrel, 11.0 mm dia base collar,
  # 7.25 mm inner bore)
  catalog["ace"] = CameraPreset(
      name="ace",
      description=(
          "Basler ace classic (29x29x54 mm body, C-Mount, Hirose HR10"
          " connector)"
      ),
      body=BodyConfig(
          width=29.0,
          height=29.0,
          depth=42.0,
      ),
      mount=MountConfig(
          outer_diameter=27.86,  # Front flange (matches 1.5 deg taper)
          outer_diameter_body=28.49,  # At body junction
          collar_length=12.0,
          inner_bore=25.4,
          baffle_length=5.93,  # Face 204 in STEP CAD
          baffle_inner_diameter=16.0,  # Face 204 in STEP CAD
          baffle_step_z_offset=0.40,  # Forward of body face (at Z = -11.60 mm)
      ),
      connector=ConnectorConfig(
          pos_x=-7.90,  # Left side looking at rear (-X)
          pos_y=6.20,  # Lower half (+Y in camera optical frame)
          length=6.30,  # Protrusion
          outer_diameter=8.85,  # Main barrel (HR10A-7R-6PB)
          inner_diameter=7.25,  # Inner bore
          collar_diameter=11.00,  # Base mounting collar
          collar_length=0.85,
      ),
  )

  # 2. Basler ace 2 - drawing IB027652 03, STEP "Basler ace2 GigE.stp"
  # Body 29x29x44.59 mm, collar 10.87 mm -> total body 55.46 mm,
  # M8 connector sticks out 6.71 mm (total 62.17 mm).
  # Rear layout (looking at rear face):
  # M8 6-pin on LOWER RIGHT (+X = +5.80 mm, +Y = +8.10 mm)
  # Connector: M8 6-pin receptacle (10.0 mm uniform outer barrel,
  # 5.60 mm inner bore)
  catalog["ace2"] = CameraPreset(
      name="ace2",
      description=(
          "Basler ace 2 (29x29x55.5 mm body, C-Mount with flats, M8 6-pin"
          " connector)"
      ),
      body=BodyConfig(
          width=29.0,
          height=29.0,
          depth=44.59,
      ),
      mount=MountConfig(
          outer_diameter=29.60,
          collar_length=10.87,
          collar_base_length=4.295,  # Rear block continuing 29x29 mm body
          has_flats=True,
          flat_width=27.87,
          flat_height=28.15,
          baffle_length=6.96,  # Cavity to sensor
          baffle_inner_diameter=16.60,  # Inner bore in ace 2
          baffle_step_z_offset=0.30,
      ),
      connector=ConnectorConfig(
          pos_x=5.80,  # Right side looking at rear (+X)
          pos_y=8.10,  # Lower half (+Y in camera optical frame)
          length=6.71,  # Protrusion
          outer_diameter=10.00,  # Uniform barrel (matches CAD & drawing)
          inner_diameter=5.60,  # Inner bore
      ),
  )

  return catalog


# -----------------------------------------------------------------------------
# Color & Material Palettes (PBR GLTF)
# -----------------------------------------------------------------------------
def make_material(
    name: str,
    base_color: Tuple[float, float, float, float],
    metallic: float = 0.1,
    roughness: float = 0.5,
) -> trimesh.visual.material.PBRMaterial:
  return trimesh.visual.material.PBRMaterial(
      name=name,
      baseColorFactor=base_color,
      metallicFactor=metallic,
      roughnessFactor=roughness,
  )


MAT_BASLER_LIGHT_GREY = make_material(
    "BaslerLightGrey", (0.85, 0.89, 0.90, 1.0), 0.15, 0.35
)
MAT_ALUMINUM_SILVER = make_material(
    "AluminumSilver", (0.78, 0.79, 0.74, 1.0), 0.75, 0.30
)
MAT_BLACK_INSULATOR = make_material(
    "BlackInsulator", (0.05, 0.05, 0.05, 1.0), 0.05, 0.70
)


def apply_mat(
    mesh: trimesh.Trimesh, mat: trimesh.visual.material.PBRMaterial
) -> trimesh.Trimesh:
  """Applies a PBR material to a mesh and sets neutral white vertex colors."""
  mesh.visual = trimesh.visual.ColorVisuals(mesh=mesh)
  mesh.visual.material = mat
  color_white = [255, 255, 255, 255]
  mesh.visual.vertex_colors = np.tile(color_white, (len(mesh.vertices), 1))
  return mesh


# -----------------------------------------------------------------------------
# Geometry Helper Functions
# -----------------------------------------------------------------------------
def create_chamfered_box(
    width: float,
    height: float,
    length: float,
    chamfer: float,
    front_hole_radius: float,
    hole_sections: int = 36,
) -> trimesh.Trimesh:
  """Creates a box extruded along Z with 4 chamfered longitudinal edges and
  a front hole.
  """
  w2 = width / 2.0
  h2 = height / 2.0
  c = chamfer
  poly = [
      (-w2 + c, -h2),
      (w2 - c, -h2),
      (w2, -h2 + c),
      (w2, h2 - c),
      (w2 - c, h2),
      (-w2 + c, h2),
      (-w2, h2 - c),
      (-w2, -h2 + c),
  ]
  n = len(poly)
  bottom = [[x, y, 0.0] for x, y in poly]
  center_bot = [0.0, 0.0, 0.0]

  # Front face has a circular aperture for the collar/baffle cavity
  angles = np.linspace(0, 2 * np.pi, hole_sections, endpoint=False)
  top = [[x, y, length] for x, y in poly]
  hole_pts_top = [
      [front_hole_radius * np.cos(a), front_hole_radius * np.sin(a), length]
      for a in angles
  ]

  vertices = np.array(bottom + top + [center_bot] + hole_pts_top)
  faces = []
  idx_bot_center = 2 * n
  idx_hole_start = 2 * n + 1

  for i in range(n):
    next_i = (i + 1) % n
    faces.append([idx_bot_center, next_i, i])
    faces.append([i, next_i, n + next_i])
    faces.append([i, n + next_i, n + i])

  top_2d = np.array(
      poly
      + [
          [front_hole_radius * np.cos(a), front_hole_radius * np.sin(a)]
          for a in angles
      ]
  )
  tri = Delaunay(top_2d)
  centers = top_2d[tri.simplices].mean(axis=1)
  r_centers = np.hypot(centers[:, 0], centers[:, 1])
  for s in tri.simplices[r_centers > front_hole_radius * 0.999]:
    mapped = [n + idx if idx < n else idx_hole_start + (idx - n) for idx in s]
    p0, p1, p2 = top_2d[s[0]], top_2d[s[1]], top_2d[s[2]]
    cross_z = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (
        p2[0] - p0[0]
    )
    if cross_z > 0:
      faces.append(mapped)
    else:
      faces.append([mapped[0], mapped[2], mapped[1]])

  mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
  return mesh


def create_hollow_chamfered_prism(
    width: float,
    height: float,
    length: float,
    chamfer: float,
    r_inner: float,
    sections: int = 36,
) -> trimesh.Trimesh:
  """Creates a watertight hollow prism with chamfered outer edges and inner
  cylindrical bore.
  """
  w2 = width / 2.0
  h2 = height / 2.0
  c = chamfer
  poly = [
      (-w2 + c, -h2),
      (w2 - c, -h2),
      (w2, -h2 + c),
      (w2, h2 - c),
      (w2 - c, h2),
      (-w2 + c, h2),
      (-w2, h2 - c),
      (-w2, -h2 + c),
  ]
  n_poly = len(poly)
  angles = np.linspace(0, 2 * np.pi, sections, endpoint=False)
  hole_pts = np.column_stack(
      [r_inner * np.cos(angles), r_inner * np.sin(angles)]
  )

  pts_2d = np.vstack([poly, hole_pts])
  tri = Delaunay(pts_2d)
  centers = pts_2d[tri.simplices].mean(axis=1)
  r_c = np.hypot(centers[:, 0], centers[:, 1])
  cap_simplices = tri.simplices[r_c > r_inner * 0.999]

  bot_outer = np.column_stack([poly, np.zeros(n_poly)])
  bot_inner = np.column_stack([hole_pts, np.zeros(sections)])
  top_outer = np.column_stack([poly, np.full(n_poly, length)])
  top_inner = np.column_stack([hole_pts, np.full(sections, length)])

  vertices = np.vstack([bot_outer, bot_inner, top_outer, top_inner])

  idx_bi = n_poly
  idx_to = n_poly + sections
  idx_ti = 2 * n_poly + sections

  faces = []
  # 1. Bottom end cap (z=0, normal -Z)
  for s in cap_simplices:
    p0, p1, p2 = pts_2d[s[0]], pts_2d[s[1]], pts_2d[s[2]]
    cross_z = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (
        p2[0] - p0[0]
    )
    if cross_z < 0:
      faces.append(list(s))
    else:
      faces.append([s[0], s[2], s[1]])

  # 2. Top end cap (z=length, normal +Z)
  for s in cap_simplices:
    s_top = [idx + (n_poly + sections) for idx in s]
    p0, p1, p2 = pts_2d[s[0]], pts_2d[s[1]], pts_2d[s[2]]
    cross_z = (p1[0] - p0[0]) * (p2[1] - p0[1]) - (p1[1] - p0[1]) * (
        p2[0] - p0[0]
    )
    if cross_z > 0:
      faces.append(s_top)
    else:
      faces.append([s_top[0], s_top[2], s_top[1]])

  # 3. Outer lateral walls (8 quads)
  for i in range(n_poly):
    ni = (i + 1) % n_poly
    faces.append([i, ni, idx_to + ni])
    faces.append([i, idx_to + ni, idx_to + i])

  # 4. Inner cylindrical bore (sections quads, normal inward)
  for i in range(sections):
    ni = (i + 1) % sections
    faces.append([idx_bi + i, idx_ti + ni, idx_bi + ni])
    faces.append([idx_bi + i, idx_ti + i, idx_ti + ni])

  mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
  return mesh


def create_conical_annulus(
    r_inner: float,
    r_outer_body: float,
    r_outer_front: float,
    height: float,
    sections: int = 36,
) -> trimesh.Trimesh:
  """Creates a watertight conical collar with constant inner bore and tapered
  outer radius.
  """
  angles = np.linspace(0, 2 * np.pi, sections, endpoint=False)
  cos_a = np.cos(angles)
  sin_a = np.sin(angles)

  bot_in = np.column_stack(
      [r_inner * cos_a, r_inner * sin_a, np.zeros(sections)]
  )
  bot_out = np.column_stack(
      [r_outer_body * cos_a, r_outer_body * sin_a, np.zeros(sections)]
  )
  top_in = np.column_stack(
      [r_inner * cos_a, r_inner * sin_a, np.full(sections, height)]
  )
  top_out = np.column_stack(
      [r_outer_front * cos_a, r_outer_front * sin_a, np.full(sections, height)]
  )

  vertices = np.vstack([bot_in, bot_out, top_in, top_out])
  n = sections
  faces = []
  for i in range(n):
    ni = (i + 1) % n
    bi, bo = i, n + i
    b_ni, b_no = ni, n + ni
    ti, to = 2 * n + i, 3 * n + i
    t_ni, t_no = 2 * n + ni, 3 * n + ni

    # Bottom face (z=0, normal pointing -Z towards body)
    faces.append([bo, bi, b_ni])
    faces.append([bo, b_ni, b_no])
    # Top face (z=height, normal pointing +Z towards front)
    faces.append([ti, to, t_no])
    faces.append([ti, t_no, t_ni])
    # Outer conical surface (normal pointing outward)
    faces.append([bo, b_no, t_no])
    faces.append([bo, t_no, to])
    # Inner cylindrical surface (normal pointing inward)
    faces.append([bi, ti, t_ni])
    faces.append([bi, t_ni, b_ni])

  mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
  return mesh


def create_baffle_cup(
    r_in: float,
    r_out: float,
    height: float,
    floor_thick: float = 0.5,
    sections: int = 36,
) -> trimesh.Trimesh:
  """Creates a watertight stepped baffle cup with closed sensor plane floor.

  - Top annular rim at z = floor_thick + height: connects r_in to r_out
    (normal +Z)
  - Inner cylinder at r = r_in: from z = floor_thick to z = floor_thick + height
    (normal inward)
  - Cavity floor at z = floor_thick: circular disk of radius r_in closing the
    inner bore (normal +Z)
  - Outer cylinder at r = r_out: from z = 0 to z = floor_thick + height
    (normal outward)
  - Back face at z = 0: closed circular disk of radius r_out (normal -Z)
  """
  n = sections
  angles = np.linspace(0, 2 * np.pi, n, endpoint=False)
  cos_a = np.cos(angles)
  sin_a = np.sin(angles)

  z_bot = 0.0
  z_floor = floor_thick
  z_top = floor_thick + height

  out_bot = np.column_stack([r_out * cos_a, r_out * sin_a, np.full(n, z_bot)])
  out_top = np.column_stack([r_out * cos_a, r_out * sin_a, np.full(n, z_top)])
  in_top = np.column_stack([r_in * cos_a, r_in * sin_a, np.full(n, z_top)])
  in_floor = np.column_stack([r_in * cos_a, r_in * sin_a, np.full(n, z_floor)])
  c_bot = np.array([[0.0, 0.0, z_bot]])
  c_floor = np.array([[0.0, 0.0, z_floor]])

  vertices = np.vstack([out_bot, out_top, in_top, in_floor, c_bot, c_floor])
  idx_c_bot = 4 * n
  idx_c_floor = 4 * n + 1

  faces = []
  for i in range(n):
    ni = (i + 1) % n
    ob = i
    ob_n = ni
    ot = n + i
    ot_n = n + ni
    it = 2 * n + i
    it_n = 2 * n + ni
    ifl = 3 * n + i
    ifl_n = 3 * n + ni

    # 1. Outer cylinder (normal outward)
    faces.append([ob, ob_n, ot_n])
    faces.append([ob, ot_n, ot])

    # 2. Top annular rim (z_top, normal +Z towards collar)
    faces.append([it, ot, ot_n])
    faces.append([it, ot_n, it_n])

    # 3. Inner cylinder (normal inward towards center)
    faces.append([ifl, it, it_n])
    faces.append([ifl, it_n, ifl_n])

    # 4. Cavity floor (z_floor, normal +Z closing the bottom of the bore)
    faces.append([idx_c_floor, ifl, ifl_n])

    # 5. Outer back face (z_bot, normal -Z into camera interior)
    faces.append([idx_c_bot, ob_n, ob])

  mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
  return mesh


def create_cut_annulus(
    r_inner: float,
    r_outer: float,
    flat_w: float,
    flat_h: float,
    height: float,
    sections: int = 72,
) -> trimesh.Trimesh:
  """Creates a watertight cylindrical collar cut by 4 flats with an inner
  bore.
  """
  angles = np.linspace(0, 2 * np.pi, sections, endpoint=False)
  extra_angles = []
  for dx in [-flat_w / 2.0, flat_w / 2.0]:
    if abs(dx) < r_outer:
      y = np.sqrt(r_outer**2 - dx**2)
      for dy in [-y, y]:
        extra_angles.append(np.arctan2(dy, dx) % (2 * np.pi))
  for dy in [-flat_h / 2.0, flat_h / 2.0]:
    if abs(dy) < r_outer:
      x = np.sqrt(r_outer**2 - dy**2)
      for dx in [-x, x]:
        extra_angles.append(np.arctan2(dy, dx) % (2 * np.pi))
  all_angles = np.unique(np.concatenate([angles, extra_angles]))
  all_angles.sort()

  pts_in = []
  pts_out = []
  for a in all_angles:
    ca, sa = np.cos(a), np.sin(a)
    r_cut_x = abs(flat_w / (2.0 * ca)) if abs(ca) > 1e-6 else r_outer
    r_cut_y = abs(flat_h / (2.0 * sa)) if abs(sa) > 1e-6 else r_outer
    r_out = min(r_outer, r_cut_x, r_cut_y)
    pts_in.append([r_inner * ca, r_inner * sa])
    pts_out.append([r_out * ca, r_out * sa])

  n = len(all_angles)
  bot_in = [[x, y, 0.0] for x, y in pts_in]
  bot_out = [[x, y, 0.0] for x, y in pts_out]
  top_in = [[x, y, height] for x, y in pts_in]
  top_out = [[x, y, height] for x, y in pts_out]

  vertices = np.array(bot_in + bot_out + top_in + top_out)
  faces = []
  for i in range(n):
    ni = (i + 1) % n
    bi, bo = i, n + i
    b_ni, b_no = ni, n + ni
    ti, to = 2 * n + i, 3 * n + i
    t_ni, t_no = 2 * n + ni, 3 * n + ni

    faces.append([bo, bi, b_ni])
    faces.append([bo, b_ni, b_no])
    faces.append([ti, to, t_no])
    faces.append([ti, t_no, t_ni])
    faces.append([bo, b_no, t_no])
    faces.append([bo, t_no, to])
    faces.append([bi, ti, t_ni])
    faces.append([bi, t_ni, b_ni])

  mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=True)
  return mesh


# -----------------------------------------------------------------------------
# Camera Body Builder (Clean Enclosure & Functional Hirose Connector)
# -----------------------------------------------------------------------------
def build_camera_body(preset: CameraPreset) -> List[trimesh.Trimesh]:
  """Builds clean, lightweight camera body components matching CAD drawings."""
  parts = []
  body = preset.body
  mount = preset.mount

  # 1. Main Housing Box
  # Extends from z_body_rear to z_body_front (-mount.collar_length)
  # Cross-section: 29.0 x 29.0 mm with 1.0 mm corner chamfers
  z_body_rear = -mount.collar_length - body.depth

  body_box = create_chamfered_box(
      width=body.width,
      height=body.height,
      length=body.depth,
      chamfer=body.chamfer,
      front_hole_radius=mount.inner_bore / 2.0,
  )
  body_box.apply_translation([0, 0, z_body_rear])
  parts.append(apply_mat(body_box, MAT_BASLER_LIGHT_GREY))

  # 2. C-Mount Collar (Z in [-collar_length, 0.0])
  if mount.has_flats:
    # 4-sided flat-cut collar (ace 2, matches drawing IB027652 03 and STEP CAD)
    if mount.collar_base_length > 0:
      # ace 2 stepped collar:
      # Rear part (close to body): hollow chamfered prism continuing 29x29 mm
      # body profile.
      # Front part: flat-cut collar with corner lobes and flats.
      h_base = mount.collar_base_length
      h_front = mount.collar_length - h_base

      collar_base = create_hollow_chamfered_prism(
          width=body.width,
          height=body.height,
          length=h_base,
          chamfer=body.chamfer,
          r_inner=mount.inner_bore / 2.0,
      )
      collar_base.apply_translation([0, 0, -mount.collar_length])

      collar_front = create_cut_annulus(
          r_inner=mount.inner_bore / 2.0,
          r_outer=mount.outer_diameter / 2.0,
          flat_w=mount.flat_width,
          flat_h=mount.flat_height,
          height=h_front,
      )
      collar_front.apply_translation([0, 0, -h_front])

      collar_outer = trimesh.util.concatenate([collar_base, collar_front])
    else:
      collar_outer = create_cut_annulus(
          r_inner=mount.inner_bore / 2.0,
          r_outer=mount.outer_diameter / 2.0,
          flat_w=mount.flat_width,
          flat_h=mount.flat_height,
          height=mount.collar_length,
      )
      collar_outer.apply_translation([0, 0, -mount.collar_length])
    parts.append(apply_mat(collar_outer, MAT_ALUMINUM_SILVER))
  else:
    # Round collar with draft angle (ace classic, matches drawing IB018802 01
    # and STEP CAD).
    r_body = (mount.outer_diameter_body or mount.outer_diameter) / 2.0
    collar_outer = create_conical_annulus(
        r_inner=mount.inner_bore / 2.0,
        r_outer_body=r_body,
        r_outer_front=mount.outer_diameter / 2.0,
        height=mount.collar_length,
        sections=36,
    )
    collar_outer.apply_translation([0, 0, -mount.collar_length])
    parts.append(apply_mat(collar_outer, MAT_ALUMINUM_SILVER))

  # Collar internal step/baffle at rear of collar
  # In STEP CAD, the annular step is located at z_step (Z = -11.60 mm for ace),
  # and a cylindrical cavity extends for baffle_length to the optical sensor
  # plane. create_baffle_cup creates a closed cup with a solid floor at
  # z_sensor closing the model at the rear of the optical cavity.
  z_step = -mount.collar_length + mount.baffle_step_z_offset
  z_sensor = z_step - mount.baffle_length
  floor_thick = 0.5  # Floor thickness

  baffle = create_baffle_cup(
      r_in=mount.baffle_inner_diameter / 2.0,
      r_out=mount.inner_bore / 2.0,
      height=mount.baffle_length,
      floor_thick=floor_thick,
      sections=36,
  )
  baffle.apply_translation([0, 0, z_sensor - floor_thick])
  parts.append(apply_mat(baffle, MAT_ALUMINUM_SILVER))

  # 3. Rear Connector (circular extension at Z = z_body_rear)
  conn = preset.connector
  if conn is not None:
    r_out = conn.outer_diameter / 2.0
    r_in = conn.inner_diameter / 2.0

    # Main outer barrel
    main_shell = creation.annulus(
        r_min=r_in,
        r_max=r_out,
        height=conn.length,
        sections=32,
    )
    main_shell.apply_translation(
        [conn.pos_x, conn.pos_y, z_body_rear - conn.length / 2.0]
    )
    parts.append(apply_mat(main_shell, MAT_ALUMINUM_SILVER))

    # Base collar / neck if configured (e.g. HR10 base flange on ace classic)
    if conn.collar_diameter > 0 and conn.collar_length > 0:
      c_shell = creation.cylinder(
          radius=conn.collar_diameter / 2.0,
          height=conn.collar_length,
          sections=32,
      )
      c_shell.apply_translation(
          [conn.pos_x, conn.pos_y, z_body_rear - conn.collar_length / 2.0]
      )
      parts.append(apply_mat(c_shell, MAT_ALUMINUM_SILVER))

    # Dark circular insulator insert
    insulator_len = min(2.0, conn.length * 0.4)
    insulator = creation.cylinder(
        radius=r_in, height=insulator_len, sections=24
    )
    insulator.apply_translation([
        conn.pos_x,
        conn.pos_y,
        z_body_rear - conn.length + insulator_len / 2.0 + 0.5,
    ])
    parts.append(apply_mat(insulator, MAT_BLACK_INSULATOR))

  return parts


# -----------------------------------------------------------------------------
# Collision Geometry Builder
# -----------------------------------------------------------------------------
def build_collision_geometry(preset: CameraPreset) -> trimesh.Trimesh:
  """Builds clean, watertight collision geometry for the camera body (in mm)."""
  collision_boxes = []
  body = preset.body
  mount = preset.mount

  z_body_rear = -mount.collar_length - body.depth
  z_body_center = -mount.collar_length - (body.depth / 2.0)

  # 1. Main Camera Body Box
  body_box = creation.box(extents=[body.width, body.height, body.depth])
  body_box.apply_translation([0, 0, z_body_center])
  collision_boxes.append(body_box)

  # 2. C-Mount Collar Box
  if mount.has_flats:
    h_base = mount.collar_base_length
    h_front = mount.collar_length - h_base
    if h_base > 0:
      cbox_base = creation.box(extents=[body.width, body.height, h_base])
      cbox_base.apply_translation([0, 0, -mount.collar_length + h_base / 2.0])
      collision_boxes.append(cbox_base)
    cbox_front = creation.box(
        extents=[mount.flat_width, mount.flat_height, h_front]
    )
    cbox_front.apply_translation([0, 0, -h_front / 2.0])
    collision_boxes.append(cbox_front)
  else:
    collar_dia = mount.outer_diameter_body or mount.outer_diameter
    collar = creation.box(extents=[collar_dia, collar_dia, mount.collar_length])
    collar.apply_translation([0, 0, -mount.collar_length / 2.0])
    collision_boxes.append(collar)

  # 3. Rear Connector Protrusion Box
  if preset.connector is not None:
    conn = preset.connector
    dia = max(conn.outer_diameter, conn.collar_diameter)
    cbox = creation.box(extents=[dia, dia, conn.length])
    cbox.apply_translation(
        [conn.pos_x, conn.pos_y, z_body_rear - conn.length / 2.0]
    )
    collision_boxes.append(cbox)

  merged = trimesh.util.concatenate(collision_boxes)
  return merged.convex_hull


# -----------------------------------------------------------------------------
# Main CLI
# -----------------------------------------------------------------------------
def main():
  catalog = get_preset_catalog()

  parser = argparse.ArgumentParser(
      description=(
          "Generate procedural visual and collision 3D models for a Basler"
          " camera body."
      )
  )
  parser.add_argument(
      "--model",
      default="ace",
      choices=list(catalog.keys()),
      help="Camera model preset to generate (default: ace)",
  )
  parser.add_argument(
      "--output-visual",
      default=None,
      help="Explicit output path for the visual GLB file",
  )
  parser.add_argument(
      "--output-collision",
      default=None,
      help="Explicit output path for the collision GLB file",
  )
  parser.add_argument(
      "--output-dir",
      "--output",
      default=None,
      help=(
          "Target directory for generated assets (default: current working"
          " directory)"
      ),
  )
  parser.add_argument(
      "--list",
      action="store_true",
      help="List available camera presets and exit",
  )

  args = parser.parse_args()

  if args.list:
    print("Available Basler Camera Presets:")
    for key, preset in sorted(catalog.items()):
      print(f"  {key:20s}: {preset.description}")
    return

  preset = catalog[args.model]
  output_dir = args.output_dir or "."

  vis_path = args.output_visual or os.path.join(
      output_dir, f"basler_{preset.name}_visual.glb"
  )
  col_path = args.output_collision or os.path.join(
      output_dir, f"basler_{preset.name}_collision.glb"
  )

  print(
      f"Generating Basler camera model '{args.model}' ({preset.description})..."
  )

  # Scale factor from internal millimeters to standard SI meters for export
  scale_to_meters = 0.001

  # Build visual geometry (computed in mm, scaled to meters for export)
  parts = build_camera_body(preset)
  for part in parts:
    part.apply_scale(scale_to_meters)
  scene = trimesh.Scene(parts)
  os.makedirs(os.path.dirname(os.path.abspath(vis_path)), exist_ok=True)
  scene.export(vis_path)
  print(f"  Wrote visual model: {vis_path}")

  # Build collision geometry (computed in mm, scaled to meters for export)
  collision_mesh = build_collision_geometry(preset)
  collision_mesh.apply_scale(scale_to_meters)
  os.makedirs(os.path.dirname(os.path.abspath(col_path)), exist_ok=True)
  collision_mesh.export(col_path)
  print(f"  Wrote collision model: {col_path}")

  print("Done!")


if __name__ == "__main__":
  main()
