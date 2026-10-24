"""Backward-compatible executable for the new segmentation-only lane node.

The projection stage now lives in lane_projection.py. Use lane_detection.launch.py
to start four per-camera segmenters and the shared projector.
"""
from imprimis_perception.segmentation import main


if __name__ == '__main__':
    main()
