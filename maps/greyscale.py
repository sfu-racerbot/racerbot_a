# This code is developed from https://github.com/TeoIlie/F1TENTH_Racetracks with slight modifications, all credits go to Theodor Ilie
#!/usr/bin/env python3

"""
Convert map image PNG to grayscale for use in the simulator.

Usage:
    python3 convert_to_grayscale.py --map Drift # Change Drift to the map name you want to turn into grey scale.
"""

import argparse
from pathlib import Path

from PIL import Image

def main():
    parser = argparse.ArgumentParser(
        description="Convert RGB to grayscale for use in simulator",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--map", type=str, default="Drift", help="Map name (default: Drift)")

    args = parser.parse_args()

    # Paths
    script_dir = Path(__file__).parent
    map_path = script_dir / args.map
    map_name = args.map
    img_path = map_path / f"{map_name}.png"

    if not map_path.exists():
        raise ValueError(f"ERROR: Map directory not found: {map_path}")

    # Load the RGB image
    img = Image.open(img_path)
    print(f"Original - Mode: {img.mode}, Size: {img.size}")

    # Convert to grayscale
    img_gray = img.convert("L")
    print(f"Converted - Mode: {img_gray.mode}, Size: {img_gray.size}")

    # Save as grayscale
    out_path = map_path / f"{map_name}_greyscale.png"
    img_gray.save(out_path)
    print(f"Saved grayscale version as {map_name}_greyscale.png")


if __name__ == "__main__":
    main()