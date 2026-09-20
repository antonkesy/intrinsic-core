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

"""Generic CLI tool to generate ChArUco board simulation assets."""

import os
import string

from absl import app
from absl import flags
from absl import logging
import cv2
import numpy as np
from PIL import Image
import trimesh
import trimesh.visual

_NAME = flags.DEFINE_string("name", None, "Model name.", required=True)
_ROWS = flags.DEFINE_integer("rows", None, "Number of rows.", required=True)
_COLS = flags.DEFINE_integer("cols", None, "Number of columns.", required=True)
_CHECKER_MM = flags.DEFINE_float(
    "checker_mm", None, "Checker size in mm.", required=True
)
_MARKER_MM = flags.DEFINE_float(
    "marker_mm", None, "Marker size in mm.", required=True
)
_DICT_ID = flags.DEFINE_string(
    "dict_id", None, "OpenCV dictionary enum string.", required=True
)
_BOARD_WIDTH_MM = flags.DEFINE_float(
    "board_width_mm", None, "Board width in mm.", required=True
)
_BOARD_HEIGHT_MM = flags.DEFINE_float(
    "board_height_mm", None, "Board height in mm.", required=True
)
_OUT_DIR = flags.DEFINE_string(
    "out_dir", None, "Output directory.", required=True
)
_SDF_TEMPLATE_PATH = flags.DEFINE_string(
    "sdf_template", None, "Path to SDF template.", required=True
)
_CONFIG_TEMPLATE_PATH = flags.DEFINE_string(
    "config_template", None, "Path to config template.", required=True
)
_URI_PREFIX = flags.DEFINE_string(
    "uri_prefix",
    "model://intrinsic/perception/calibration/charuco_boards",
    "URI prefix for model glb.",
)


def create_textured_box(w, h, t, texture_path, texture_name):
  """Creates a 3D box mesh with explicit UV mapping for the top face."""
  vertices = [
      # Top (z=0)
      [-w / 2, -h / 2, 0],
      [w / 2, -h / 2, 0],
      [w / 2, h / 2, 0],
      [-w / 2, h / 2, 0],
      # Bottom (z=-t)
      [-w / 2, -h / 2, -t],
      [w / 2, -h / 2, -t],
      [w / 2, h / 2, -t],
      [-w / 2, h / 2, -t],
      # Front (y=-h/2)
      [-w / 2, -h / 2, 0],
      [w / 2, -h / 2, 0],
      [w / 2, -h / 2, -t],
      [-w / 2, -h / 2, -t],
      # Back (y=h/2)
      [-w / 2, h / 2, 0],
      [w / 2, h / 2, 0],
      [w / 2, h / 2, -t],
      [-w / 2, h / 2, -t],
      # Left (x=-w/2)
      [-w / 2, -h / 2, 0],
      [-w / 2, h / 2, 0],
      [-w / 2, h / 2, -t],
      [-w / 2, -h / 2, -t],
      # Right (x=w/2)
      [w / 2, -h / 2, 0],
      [w / 2, h / 2, 0],
      [w / 2, h / 2, -t],
      [w / 2, -h / 2, -t],
  ]

  faces = [
      # Top
      [0, 1, 2],
      [0, 2, 3],
      # Bottom
      [4, 7, 6],
      [4, 6, 5],
      # Front
      [8, 11, 10],
      [8, 10, 9],
      # Back
      [12, 13, 14],
      [12, 14, 15],
      # Left
      [16, 17, 18],
      [16, 18, 19],
      # Right
      [20, 23, 22],
      [20, 22, 21],
  ]

  uvs = [
      # Top UVs
      [0, 0],
      [1, 0],
      [1, 1],
      [0, 1],
      # Bottom UVs
      [0, 0],
      [0, 0],
      [0, 0],
      [0, 0],
      # Front UVs
      [0, 0],
      [0, 0],
      [0, 0],
      [0, 0],
      # Back UVs
      [0, 0],
      [0, 0],
      [0, 0],
      [0, 0],
      # Left UVs
      [0, 0],
      [0, 0],
      [0, 0],
      [0, 0],
      # Right UVs
      [0, 0],
      [0, 0],
      [0, 0],
      [0, 0],
  ]

  material = trimesh.visual.material.PBRMaterial(
      baseColorTexture=Image.open(texture_path),
      name=texture_name,
  )
  visuals = trimesh.visual.TextureVisuals(uv=uvs, material=material)
  mesh = trimesh.Trimesh(
      vertices=vertices, faces=faces, visual=visuals, validate=False
  )
  return mesh


def generate_board() -> None:
  """Generates a single ChArUco board simulation asset package."""
  thickness_m = 0.006  # 6mm plate thickness
  px_per_m = 10000  # 10 pixels per mm

  checker_m = _CHECKER_MM.value / 1000.0
  marker_m = _MARKER_MM.value / 1000.0
  board_width = _BOARD_WIDTH_MM.value / 1000.0
  board_height = _BOARD_HEIGHT_MM.value / 1000.0

  model_name = _NAME.value
  out_dir = _OUT_DIR.value
  os.makedirs(out_dir, exist_ok=True)

  # 1. Generate ChArUco Texture PNG using OpenCV
  dict_enum = getattr(cv2.aruco, _DICT_ID.value)
  aruco_dict = cv2.aruco.getPredefinedDictionary(dict_enum)
  board = cv2.aruco.CharucoBoard(
      (_COLS.value, _ROWS.value), checker_m, marker_m, aruco_dict
  )
  # See https://github.com/opencv/opencv/issues/23152, as of 2026-05-19,
  # calib.io uses the legacy pattern.
  board.setLegacyPattern(True)

  grid_w_px = int(_COLS.value * checker_m * px_per_m)
  grid_h_px = int(_ROWS.value * checker_m * px_per_m)
  grid_img = board.generateImage((grid_w_px, grid_h_px), marginSize=0)

  board_width_px = int(board_width * px_per_m)
  board_height_px = int(board_height * px_per_m)
  plate_img = np.ones((board_height_px, board_width_px), dtype=np.uint8) * 255

  # Center grid on plate
  margin_x = (board_width_px - grid_w_px) // 2
  margin_y = (board_height_px - grid_h_px) // 2
  plate_img[
      margin_y : margin_y + grid_h_px, margin_x : margin_x + grid_w_px
  ] = grid_img

  # Add identifying text label to the bottom-left margin
  checker_str = f"{_CHECKER_MM.value:g}"
  marker_str = f"{_MARKER_MM.value:g}"
  dict_prefix = _DICT_ID.value.rsplit("_", 1)[0]
  dict_str = f"AruCo {dict_prefix}"
  label_text = (
      f"{_ROWS.value}x{_COLS.value} | Checker Size: {checker_str} mm | Marker"
      f" Size: {marker_str} mm | Dictionary: {dict_str}."
  )
  font = cv2.FONT_HERSHEY_SIMPLEX
  font_scale = 0.9
  color = (0.0, 0.0, 0.0)  # black text
  thickness = 1
  text_x = int(margin_x)
  text_y = int(board_height_px - (margin_y // 3))
  cv2.putText(
      plate_img,
      label_text,
      (text_x, text_y),
      font,
      font_scale,
      color,
      thickness,
      cv2.LINE_AA,
  )

  texture_name = f"{model_name}_texture.png"
  texture_path = os.path.join(out_dir, texture_name)
  cv2.imwrite(texture_path, plate_img)

  # 2. Generate 3D CAD Model (.glb)
  mesh = create_textured_box(
      board_width, board_height, thickness_m, texture_path, texture_name
  )
  glb_path = os.path.join(out_dir, f"{model_name}.glb")
  mesh.export(glb_path, file_type="glb")

  # 3. Generate SDF File
  with open(_SDF_TEMPLATE_PATH.value, "r", encoding="utf-8") as f:
    sdf_template = string.Template(f.read())
  with open(_CONFIG_TEMPLATE_PATH.value, "r", encoding="utf-8") as f:
    config_template = string.Template(f.read())

  uri_path = f"{_URI_PREFIX.value}/{model_name}.glb"
  collision_z = f"{-thickness_m / 2:.4f}"
  sdf_content = sdf_template.substitute(
      model_name=model_name,
      uri_path=uri_path,
      collision_z=collision_z,
      board_width=board_width,
      board_height=board_height,
      thickness_m=thickness_m,
  )
  with open(
      os.path.join(out_dir, f"{model_name}.sdf"), "w", encoding="utf-8"
  ) as f:
    f.write(sdf_content)

  # 4. Generate model.config
  config_content = config_template.substitute(
      model_name=model_name,
      cols=_COLS.value,
      rows=_ROWS.value,
      checker_mm=_CHECKER_MM.value,
      marker_mm=_MARKER_MM.value,
      board_width_mm=_BOARD_WIDTH_MM.value,
      board_height_mm=_BOARD_HEIGHT_MM.value,
  )
  with open(
      os.path.join(out_dir, f"{model_name}.config"), "w", encoding="utf-8"
  ) as f:
    f.write(config_content)

  logging.info("Successfully generated %s in %s", model_name, out_dir)


def main(_) -> None:
  generate_board()


if __name__ == "__main__":
  app.run(main)
