"""Korean licence plate OCR from a USB camera, published as a ROS 2 topic.

The modules are split by dependency so the cheap parts stay testable:

* ``normalize``, ``debounce`` — standard library only.
* ``ocr`` — needs Tesseract (``tesserocr`` binding, or the CLI as a fallback).
* ``camera``, ``debug``, ``reader`` — need OpenCV.
* ``probe`` — a ROS-free command line front end for bring-up.
* ``plate_ocr_node`` — the rclpy node; a thin wrapper over ``reader``.
"""
