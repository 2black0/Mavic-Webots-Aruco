"""
ArUco Marker Detection Module
============================

This module provides detection of ArUco markers in camera frames and
visualization helpers for debug output. ArUco markers are square fiducial 
markers that can be detected efficiently in images, providing precise
position estimation for landing targets.
"""

import cv2
from cv2 import aruco
from .config import ControlParams


class ArucoDetector:
    """
    Detects ArUco markers and draws helper overlays on a BGR frame.
    
    This class encapsulates the OpenCV ArUco detector with appropriate
    configuration for drone landing applications. It handles both detection
    and visualization in a single interface.
    
    Attributes:
        dict: ArUco dictionary containing marker patterns to detect.
        det: ArUco detector instance configured with detection parameters.
    """

    def __init__(self):
        """
        Initialize ArUco detector with the configured dictionary type.
        
        Reads the ArUco dictionary type from ControlParams and initializes
        the detector with default detection parameters.
        """
        cfg        = ControlParams()
        self.dict  = aruco.getPredefinedDictionary(getattr(aruco, cfg.aruco_dict_type))
        self.det   = aruco.ArucoDetector(self.dict, aruco.DetectorParameters())

    # ------------------------------------------------------------------ #
    def detect(self, img):
        """
        Detect ArUco markers in an image and draw visual overlays.
        
        This method:
        1. Converts the image to grayscale for marker detection
        2. Detects markers using the configured detector
        3. Draws marker outlines on the input image when detected
        4. Adds crosshair lines for visual alignment reference
        
        Parameters
        ----------
        img : np.ndarray
            BGR image (uint8, h×w×3) where markers will be detected.
            This image is modified in-place with visual overlays.
            
        Returns
        -------
        tuple
            (corners, ids, rejected, img) where:
            - corners: List of corner coordinates for each detected marker
            - ids: Array of marker IDs corresponding to detected markers
            - rejected: List of rejected candidate markers
            - img: Original image with visual overlays added
        
        Notes
        -----
        The return signature matches the legacy implementation for compatibility.
        Corners are in the format expected by the drone controller positioning system.
        """
        # Convert to grayscale for more robust marker detection
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # Detect markers in the grayscale image
        corners, ids, rejected = self.det.detectMarkers(gray)

        if ids is not None:
            # Draw marker outlines and ID labels on the original image
            aruco.drawDetectedMarkers(img, corners)
            
            # Draw crosshair lines for visual alignment reference
            h, w = img.shape[:2]
            cv2.line(img, (w // 2, 0), (w // 2, h), (255, 255, 0), 1)  # Vertical line
            cv2.line(img, (0, h // 2), (w, h // 2), (255, 255, 0), 1)  # Horizontal line

        return corners, ids, rejected, img