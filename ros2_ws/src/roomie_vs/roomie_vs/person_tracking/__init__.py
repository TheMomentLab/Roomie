"""
Person tracking module for roomie vision service.
Provides real-time person detection and tracking using ByteTrack + YOLOv8n.
"""

from .person_tracker import PersonTracker

__all__ = ['PersonTracker'] 