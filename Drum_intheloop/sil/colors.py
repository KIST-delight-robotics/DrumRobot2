"""
Reusable SIL color palette.

All values are normalized RGBA tuples for PyBullet.
Change these values directly if you want to retheme the robot quickly.
"""

from typing import Dict, Optional, Tuple

Color = Tuple[float, float, float, float]
RgbColor = Tuple[float, float, float]

CHARCOAL: Color = (0.08, 0.08, 0.08, 1.0)
GRAPHITE: Color = (0.17, 0.18, 0.20, 1.0)
STEEL: Color = (0.34, 0.37, 0.41, 1.0)
SILVER: Color = (0.67, 0.69, 0.72, 1.0)

IVORY: Color = (0.95, 0.93, 0.89, 1.0)
BEIGE: Color = (0.84, 0.74, 0.63, 1.0)
CAMEL: Color = (0.73, 0.60, 0.47, 1.0)
BROWN: Color = (0.47, 0.31, 0.21, 1.0)

BRICK: Color = (0.69, 0.16, 0.14, 1.0)
CRIMSON: Color = (0.55, 0.11, 0.15, 1.0)
TERRACOTTA: Color = (0.74, 0.36, 0.24, 1.0)
OCHRE: Color = (0.73, 0.57, 0.19, 1.0)

OLIVE: Color = (0.48, 0.49, 0.21, 1.0)
SAGE: Color = (0.55, 0.62, 0.54, 1.0)
TEAL: Color = (0.20, 0.46, 0.46, 1.0)
SLATE: Color = (0.33, 0.40, 0.55, 1.0)


PALETTE_16: Dict[str, Color] = {
    "charcoal": CHARCOAL,
    "graphite": GRAPHITE,
    "steel": STEEL,
    "silver": SILVER,
    "ivory": IVORY,
    "beige": BEIGE,
    "camel": CAMEL,
    "brown": BROWN,
    "brick": BRICK,
    "crimson": CRIMSON,
    "terracotta": TERRACOTTA,
    "ochre": OCHRE,
    "olive": OLIVE,
    "sage": SAGE,
    "teal": TEAL,
    "slate": SLATE,
}


def rgb(color: Color) -> RgbColor:
    return color[0], color[1], color[2]


# Keep None to use the PyBullet default background.
GUI_BACKGROUND_RGB: Optional[RgbColor] = (1, 1, 1)

# The floor can still be lightly tinted even when the background is default.
PLANE_RGBA: Color = None

ROBOT_THEME: Dict[str, Color] = {
    "base_link": CHARCOAL,
    "waist": STEEL,
    "head": STEEL,
    "head_2": GRAPHITE,
    "left_shoulder_1": CHARCOAL,
    "left_shoulder_2": BRICK,
    "right_shoulder_1": CHARCOAL,
    "right_shoulder_2": BRICK,
    "left_elbow": BRICK,
    "left_wrist": CAMEL,
    "right_elbow": BRICK,
    "right_wrist": CAMEL,
}
