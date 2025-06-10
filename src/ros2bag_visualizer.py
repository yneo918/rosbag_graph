#!/usr/bin/env python3
"""
ROS2 Bag Data Visualizer with PyQt6

Features:
- Load and visualize ROS2 bag files (.db3 and .mcap)
- Search/filter topics by name
- Support for array types (Int32MultiArray, Float32MultiArray, etc.) with individual element selection
- Handles Python array.array type commonly used in ROS2 messages
- Time series and X-Y plotting
- Multiple plots in tabs with renameable tabs
- Editable plot properties (title, axis labels, legend)
- Time series display modes: Absolute time or Elapsed time with configurable origin
- Delete selected fields with Delete key or context menu
- Double-click to quickly add fields to plot or rename tabs

Common ROS2 array message types supported:
- std_msgs/Float32MultiArray: data field contains array('f', [values...])
- std_msgs/Int32MultiArray: data field contains array('i', [values...])
- std_msgs/Float64MultiArray: data field contains array('d', [values...])
- sensor_msgs/PointCloud2: data field contains bytes or array
- sensor_msgs/Image: data field contains array of pixel values

Requirements:
- PyQt6
- matplotlib
- rosbag2-py
- rclpy
- ros-<distro>-rosbag2-storage-mcap (for MCAP support)
"""

import sys
import os
import json
import array
import inspect
import threading
import numpy as np
from datetime import datetime
from collections import defaultdict
from typing import Dict, List, Any, Tuple, Union

from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QFileDialog, QListWidget, QListWidgetItem,
    QSplitter, QGroupBox, QComboBox, QLabel, QCheckBox,
    QTreeWidget, QTreeWidgetItem, QTabWidget, QMessageBox,
    QProgressBar, QStatusBar, QLineEdit, QSpinBox, QMenu,
    QDialog, QDialogButtonBox, QFormLayout, QTextEdit, QDoubleSpinBox
)
from PyQt6.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt6.QtGui import QIcon, QAction, QKeyEvent

import matplotlib
# Set matplotlib backend before importing other matplotlib modules
matplotlib.use('QtAgg')  # Use QtAgg for PyQt6 compatibility
from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qtagg import NavigationToolbar2QT as NavigationToolbar
from matplotlib.figure import Figure
import matplotlib.pyplot as plt

# Constants
DEFAULT_ARRAY_DISPLAY_LIMIT = 20
LARGE_ARRAY_THRESHOLD = 10
MAX_ARRAY_PREVIEW = 100
PROGRESS_UPDATE_INTERVAL = 100
PROGRESS_VISUAL_UPDATE_INTERVAL = 50  # More frequent visual updates
LINE_TRUNCATE_LENGTH = 2000
VALUE_PREVIEW_LENGTH = 50

# Loading progress phases
PROGRESS_OPENING_BAG = 5       # 5% for opening bag
PROGRESS_READING_METADATA = 10 # 10% for reading metadata
PROGRESS_READING_MESSAGES = 90 # 90% for reading messages
PROGRESS_FINALIZING = 98       # 98% for finalizing
PROGRESS_COMPLETE = 100        # 100% complete

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    HAS_ROS2 = True
except ImportError:
    HAS_ROS2 = False
    print("Warning: ROS2 packages not found. Install rosbag2-py and rclpy.")
    print("For MCAP support, also install: ros-<distro>-rosbag2-storage-mcap")


# Custom Exceptions
class RosbagVisualizerError(Exception):
    """Base exception for RosbagVisualizer"""
    pass


class BagFileError(RosbagVisualizerError):
    """Exception raised when bag file operations fail"""
    pass


class MessageDeserializationError(RosbagVisualizerError):
    """Exception raised when message deserialization fails"""
    pass


class PlotError(RosbagVisualizerError):
    """Exception raised during plotting operations"""
    pass

class PlotPropertiesDialog(QDialog):
    """Dialog for editing plot properties"""
    def __init__(self, plot_info, parent=None):
        super().__init__(parent)
        self.plot_info = plot_info
        self.init_ui()
        
    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("Edit Plot Properties")
        self.setModal(True)
        self.resize(500, 400)
        
        layout = QFormLayout()
        
        # Title
        self.title_edit = QLineEdit(self.plot_info.get('title', ''))
        layout.addRow("Title:", self.title_edit)
        
        # X-axis label
        self.xlabel_edit = QLineEdit(self.plot_info.get('xlabel', ''))
        layout.addRow("X-axis Label:", self.xlabel_edit)
        
        # Y-axis label
        self.ylabel_edit = QLineEdit(self.plot_info.get('ylabel', ''))
        layout.addRow("Y-axis Label:", self.ylabel_edit)
        
        # Legend labels
        legends_label = QLabel("Legend Labels:")
        layout.addRow(legends_label)
        
        self.legend_edits = {}
        for legend in self.plot_info.get('legends', []):
            legend_edit = QLineEdit(legend)
            self.legend_edits[legend] = legend_edit
            layout.addRow(f"  {legend}:", legend_edit)
        
        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        layout.addRow(buttons)
        
        self.setLayout(layout)
        
    def get_properties(self):
        """Get edited properties"""
        return {
            'title': self.title_edit.text(),
            'xlabel': self.xlabel_edit.text(),
            'ylabel': self.ylabel_edit.text(),
            'legend_map': {
                old: edit.text() 
                for old, edit in self.legend_edits.items()
                if edit.text() != old
            }
        }


class DataLoaderThread(QThread):
    """Thread for loading rosbag data in background"""
    progress = pyqtSignal(int)
    status = pyqtSignal(str)
    finished = pyqtSignal(dict)
    error = pyqtSignal(str)
    
    def __init__(self, bag_path, storage_id='sqlite3'):
        super().__init__()
        self.bag_path = bag_path
        self.storage_id = storage_id
        self._lock = threading.Lock()
        self._is_cancelled = False
    
    @property
    def is_cancelled(self):
        with self._lock:
            return self._is_cancelled
    
    def cancel(self):
        """Cancel the loading process"""
        with self._lock:
            self._is_cancelled = True
        
    def run(self):
        """Load rosbag data"""
        reader = None
        try:
            self.status.emit(f"Loading rosbag data from {os.path.basename(self.bag_path)}...")
            
            # Initialize rosbag reader
            storage_options = StorageOptions(uri=self.bag_path, storage_id=self.storage_id)
            converter_options = ConverterOptions(
                input_serialization_format='cdr',
                output_serialization_format='cdr'
            )
            
            reader = SequentialReader()
            
            try:
                reader.open(storage_options, converter_options)
            except (FileNotFoundError, OSError) as e:
                raise BagFileError(f"Cannot access bag file: {str(e)}")
            except Exception as e:
                raise BagFileError(f"Failed to open bag file: {str(e)}\n"
                                 f"Make sure you have the required rosbag2 storage plugin installed.")
            
            # Get metadata
            topic_types = reader.get_all_topics_and_types()
            self.progress.emit(PROGRESS_READING_METADATA)
            self.status.emit("Reading bag metadata...")
            
            if not topic_types:
                self.error.emit("No topics found in the bag file.")
                return
            
            # Dictionary to store messages by topic
            messages = defaultdict(list)
            topic_info = {}
            
            # Process topic types
            for topic_type in topic_types:
                topic_name = topic_type.name
                msg_type = topic_type.type
                topic_info[topic_name] = {
                    'type': msg_type,
                    'count': 0
                }
            
            # Read messages
            self.status.emit("Loading messages...")
            total_messages = 0
            processed_raw_messages = 0
            last_progress = PROGRESS_READING_METADATA
            
            # First pass: count total messages for accurate progress
            messages_to_process = 0
            temp_reader = SequentialReader()
            try:
                temp_reader.open(storage_options, converter_options)
                while temp_reader.has_next():
                    temp_reader.read_next()
                    messages_to_process += 1
                temp_reader = None
            except Exception:
                # If counting fails, fall back to incremental progress
                messages_to_process = 0
            
            while reader.has_next() and not self.is_cancelled:
                try:
                    (topic, data, timestamp) = reader.read_next()
                    processed_raw_messages += 1
                    
                    # Update progress based on actual completion if we know total count
                    if messages_to_process > 0:
                        # Calculate accurate progress
                        progress_percentage = PROGRESS_READING_METADATA + (
                            (processed_raw_messages / messages_to_process) * 
                            (PROGRESS_READING_MESSAGES - PROGRESS_READING_METADATA)
                        )
                        last_progress = min(progress_percentage, PROGRESS_READING_MESSAGES - 1)
                    else:
                        # Fallback to incremental progress
                        if processed_raw_messages % PROGRESS_VISUAL_UPDATE_INTERVAL == 0:
                            if last_progress < 30:
                                increment = 1.5
                            elif last_progress < 60:
                                increment = 1.0
                            elif last_progress < 85:
                                increment = 0.8
                            else:
                                increment = 0.3
                            last_progress = min(last_progress + increment, PROGRESS_READING_MESSAGES - 1)
                    
                    # Update progress every message for smooth progress
                    if processed_raw_messages % max(1, PROGRESS_VISUAL_UPDATE_INTERVAL // 5) == 0:
                        self.progress.emit(int(last_progress))
                        self.status.emit(f"Processing messages... {processed_raw_messages} processed")
                    
                    if topic in topic_info:
                        # Get message type
                        msg_type = topic_info[topic]['type']
                        
                        try:
                            # Deserialize message
                            msg_class = get_message(msg_type)
                            msg = deserialize_message(data, msg_class)
                            
                            # Convert message to dictionary
                            msg_dict = self._message_to_dict(msg)
                            msg_dict['_timestamp'] = timestamp
                            
                            messages[topic].append(msg_dict)
                            topic_info[topic]['count'] += 1
                            total_messages += 1
                                
                        except Exception as e:
                            # Log but continue with other messages
                            print(f"MessageDeserializationError for {topic}: {e}")
                            
                except Exception as e:
                    print(f"Error reading message: {e}")
                    processed_raw_messages += 1  # Count failed reads too
                    continue
            
            # Ensure we reach the reading messages completion
            self.progress.emit(PROGRESS_READING_MESSAGES)
            self.status.emit(f"Completed processing {processed_raw_messages} messages")
            
            # Prepare result
            self.status.emit("Finalizing data...")
            self.progress.emit(PROGRESS_FINALIZING)
            
            result = {
                'messages': dict(messages),
                'topic_info': topic_info,
                'total_messages': total_messages
            }
            
            self.progress.emit(PROGRESS_COMPLETE)
            self.status.emit(f"Successfully loaded {total_messages} messages from {len(messages)} topics")
            self.finished.emit(result)
            
        except BagFileError as e:
            self.error.emit(str(e))
        except Exception as e:
            self.error.emit(f"Unexpected error loading rosbag: {str(e)}")
        finally:
            # Ensure reader is properly closed
            if reader is not None:
                try:
                    # SequentialReader doesn't have an explicit close method
                    # The reader will be cleaned up when it goes out of scope
                    reader = None
                except Exception:
                    # Ignore cleanup errors
                    pass
    
    def _message_to_dict(self, msg) -> dict:
        """Convert ROS message to dictionary"""
        result = {}
        
        # Get all fields
        try:
            # Get field names and types
            if hasattr(msg, 'get_fields_and_field_types'):
                fields = msg.get_fields_and_field_types()
            else:
                # Alternative method for some message types
                fields = {}
                for attr in dir(msg):
                    if not attr.startswith('_') and not callable(getattr(msg, attr)):
                        fields[attr] = type(getattr(msg, attr)).__name__
        except Exception as e:
            print(f"Error getting fields: {e}")
            return result
            
        for field_name in fields:
            try:
                field_value = getattr(msg, field_name)
                
                # Handle different field types
                if hasattr(field_value, 'get_fields_and_field_types'):
                    # Nested message
                    result[field_name] = self._message_to_dict(field_value)
                elif isinstance(field_value, array.array):
                    # Python array type - convert to list
                    result[field_name] = field_value.tolist()
                elif isinstance(field_value, np.ndarray):
                    # NumPy array - convert to list
                    result[field_name] = field_value.tolist()
                elif isinstance(field_value, (list, tuple)):
                    # Array or list
                    processed_items = []
                    for item in field_value:
                        if hasattr(item, 'get_fields_and_field_types'):
                            processed_items.append(self._message_to_dict(item))
                        elif isinstance(item, array.array):
                            processed_items.append(item.tolist())
                        elif isinstance(item, np.ndarray):
                            processed_items.append(item.tolist())
                        else:
                            processed_items.append(item)
                    result[field_name] = processed_items
                elif isinstance(field_value, bytes):
                    # Convert bytes to list of integers for display
                    result[field_name] = list(field_value)
                else:
                    # Primitive type
                    result[field_name] = field_value
                    
            except Exception as e:
                print(f"Error processing field {field_name}: {e}")
                result[field_name] = None
                
        return result
    


class PlotCanvas(FigureCanvas):
    """Matplotlib canvas for plotting"""
    def __init__(self, parent=None):
        self.figure = Figure(figsize=(8, 6))
        super().__init__(self.figure)
        self.setParent(parent)
        
        # Create subplot
        self.ax = self.figure.add_subplot(111)
        self.ax.grid(True, alpha=0.3)
        
        # Store plot data
        self.plot_data = []
        self.plot_lines = {}
        self._supports_elapsed_time = True
        
    def clear_plot(self):
        """Clear the plot"""
        self.ax.clear()
        self.ax.grid(True, alpha=0.3)
        self.plot_data = []
        self.plot_lines = {}
        self.draw()
        
    def plot_time_series(self, timestamps, values, label="", style="-", use_elapsed_time=False, time_origin=None):
        """Plot time series data"""
        if not timestamps or not values:
            print("Warning: No data to plot")
            return None
        
        if use_elapsed_time:
            # Convert to elapsed time
            if time_origin is None:
                time_origin = timestamps[0]
            times = [(t - time_origin) / 1e9 for t in timestamps]  # Convert to seconds
            xlabel = "Elapsed Time (s)"
        else:
            # Convert timestamps to datetime
            times = [datetime.fromtimestamp(t / 1e9) for t in timestamps]
            xlabel = "Time"
        
        # Plot data
        line = self.ax.plot(times, values, style, label=label)[0]
        if hasattr(self, 'plot_lines'):
            self.plot_lines[label] = line
        else:
            self.plot_lines = {label: line}
        
        # Format x-axis
        if not use_elapsed_time:
            self.figure.autofmt_xdate()
        
        if label:
            self.ax.legend()
            
        self.ax.set_xlabel(xlabel)
        self.ax.set_ylabel("Value")
        self.ax.grid(True, alpha=0.3)
        
        self.draw()
        return line
        
    def plot_xy(self, x_values, y_values, label="", style="-"):
        """Plot X-Y data"""
        if not x_values or not y_values:
            print("Warning: No data to plot")
            return None
            
        line = self.ax.plot(x_values, y_values, style, label=label)[0]
        if hasattr(self, 'plot_lines'):
            self.plot_lines[label] = line
        else:
            self.plot_lines = {label: line}
        
        if label:
            self.ax.legend()
            
        self.draw()
        return line
    
    def get_plot_info(self):
        """Get plot information"""
        legends = []
        if hasattr(self, 'plot_lines') and self.plot_lines:
            legends = list(self.plot_lines.keys())
        elif self.ax.get_legend():
            handles, labels = self.ax.get_legend_handles_labels()
            legends = labels
        
        return {
            'title': self.ax.get_title(),
            'xlabel': self.ax.get_xlabel(),
            'ylabel': self.ax.get_ylabel(),
            'legends': legends
        }
    
    def update_labels(self, xlabel="", ylabel="", title=""):
        """Update plot labels"""
        if xlabel:
            self.ax.set_xlabel(xlabel)
        if ylabel:
            self.ax.set_ylabel(ylabel)
        if title:
            self.ax.set_title(title)
        self.draw()
    
    def update_legend_labels(self, label_map):
        """Update legend labels"""
        if hasattr(self, 'plot_lines'):
            for old_label, new_label in label_map.items():
                if old_label in self.plot_lines:
                    line = self.plot_lines[old_label]
                    line.set_label(new_label)
                    self.plot_lines[new_label] = self.plot_lines.pop(old_label)
        else:
            if self.ax.get_legend():
                handles, labels = self.ax.get_legend_handles_labels()
                new_labels = []
                for label in labels:
                    new_labels.append(label_map.get(label, label))
                self.ax.legend(handles, new_labels)
                self.draw()
                return
        
        self.ax.legend()
        self.draw()
    
    def remove_plot_line(self, label):
        """Remove a specific plot line"""
        if hasattr(self, 'plot_lines') and label in self.plot_lines:
            line = self.plot_lines[label]
            line.remove()
            del self.plot_lines[label]
            
            # Update legend
            if self.plot_lines:
                self.ax.legend()
            else:
                legend = self.ax.get_legend()
                if legend:
                    legend.remove()
            
            self.draw()
            return True
        return False
    
    def get_plot_lines(self):
        """Get list of plot line labels"""
        if hasattr(self, 'plot_lines'):
            return list(self.plot_lines.keys())
        return []


class DataFieldSelector(QWidget):
    """Widget for selecting data fields from messages"""
    field_selected = pyqtSignal(str, list)  # topic, field_path
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.message_data = {}
        
    def init_ui(self):
        """Initialize UI"""
        layout = QVBoxLayout()
        
        # Tree widget for field selection
        self.tree = QTreeWidget()
        self.tree.setHeaderLabels(["Field", "Type", "Value Sample"])
        self.tree.itemClicked.connect(self.on_item_clicked)
        self.tree.itemDoubleClicked.connect(self.on_item_double_clicked)
        self.tree.setToolTip("Click to select field, double-click on arrays to expand range")
        
        layout.addWidget(self.tree)
        self.setLayout(layout)
        
    def set_message_data(self, topic: str, messages: List[dict]):
        """Set message data for a topic"""
        self.message_data[topic] = messages
        self.update_tree(topic, messages[0] if messages else {})
        
    def update_tree(self, topic: str, sample_msg: dict):
        """Update tree with message fields"""
        self.tree.clear()
        
        # Create root item
        root = QTreeWidgetItem(self.tree)
        root.setText(0, topic)
        root.setExpanded(True)
        
        # Add fields
        self._add_fields_to_tree(root, sample_msg, [])
        
    def _add_fields_to_tree(self, parent: QTreeWidgetItem, data: Any, path: List[str]):
        """Recursively add fields to tree"""
        if isinstance(data, dict):
            for key, value in data.items():
                if key.startswith('_'):
                    continue  # Skip internal fields
                    
                item = QTreeWidgetItem(parent)
                item.setText(0, key)
                
                new_path = path + [key]
                item.setData(0, Qt.ItemDataRole.UserRole, new_path)
                
                if isinstance(value, dict):
                    item.setText(1, "dict")
                    self._add_fields_to_tree(item, value, new_path)
                elif isinstance(value, list):
                    item.setText(1, f"list[{len(value)}]")
                    if value:
                        # Check if it's a simple array (numbers, strings, etc.)
                        if len(value) > 0 and isinstance(value[0], (int, float, str, bool)):
                            # For simple arrays, create individual index items
                            # Check if this is a large array that needs expansion option
                            if len(value) > DEFAULT_ARRAY_DISPLAY_LIMIT:
                                expand_item = QTreeWidgetItem(item)
                                expand_item.setText(0, f"[Double-click to show all {len(value)} elements]")
                                expand_item.setText(1, "expand")
                                expand_item.setData(0, Qt.ItemDataRole.UserRole, None)
                            
                            # Show first N elements by default
                            show_count = min(len(value), DEFAULT_ARRAY_DISPLAY_LIMIT)
                            for idx in range(show_count):
                                idx_item = QTreeWidgetItem(item)
                                idx_item.setText(0, f"[{idx}]")
                                idx_item.setText(1, type(value[idx]).__name__)
                                idx_item.setText(2, str(value[idx])[:VALUE_PREVIEW_LENGTH])
                                idx_item.setData(0, Qt.ItemDataRole.UserRole, new_path + [idx])
                                
                            if len(value) > show_count:
                                more_item = QTreeWidgetItem(item)
                                more_item.setText(0, f"... and {len(value) - show_count} more elements")
                                more_item.setText(1, "info")
                                more_item.setData(0, Qt.ItemDataRole.UserRole, None)
                        elif len(value) > 0 and isinstance(value[0], dict):
                            # For arrays of complex objects
                            if len(value) > LARGE_ARRAY_THRESHOLD:
                                # Add expansion option for large arrays
                                expand_item = QTreeWidgetItem(item)
                                expand_item.setText(0, f"[Double-click to expand {len(value)} elements]")
                                expand_item.setText(1, "expand")
                                expand_item.setData(0, Qt.ItemDataRole.UserRole, None)
                                
                                # Show first few elements
                                for idx in range(min(len(value), 3)):
                                    idx_item = QTreeWidgetItem(item)
                                    idx_item.setText(0, f"[{idx}]")
                                    idx_item.setText(1, "dict")
                                    idx_item.setData(0, Qt.ItemDataRole.UserRole, new_path + [idx])
                                    self._add_fields_to_tree(idx_item, value[idx], new_path + [idx])
                            else:
                                # Show all elements for small arrays
                                for idx, item_value in enumerate(value):
                                    idx_item = QTreeWidgetItem(item)
                                    idx_item.setText(0, f"[{idx}]")
                                    idx_item.setText(1, "dict")
                                    idx_item.setData(0, Qt.ItemDataRole.UserRole, new_path + [idx])
                                    self._add_fields_to_tree(idx_item, item_value, new_path + [idx])
                    else:
                        # Empty list
                        empty_item = QTreeWidgetItem(item)
                        empty_item.setText(0, "(empty)")
                        empty_item.setText(1, "info")
                else:
                    item.setText(1, type(value).__name__)
                    item.setText(2, str(value)[:VALUE_PREVIEW_LENGTH])
                    
        elif isinstance(data, list) and data:
            # Handle list at root level
            for i, item_data in enumerate(data[:MAX_ARRAY_PREVIEW]):  # Show first MAX_ARRAY_PREVIEW elements
                item = QTreeWidgetItem(parent)
                item.setText(0, f"[{i}]")
                new_path = path + [i]
                item.setData(0, Qt.ItemDataRole.UserRole, new_path)
                
                if isinstance(item_data, dict):
                    item.setText(1, "dict")
                    self._add_fields_to_tree(item, item_data, new_path)
                else:
                    item.setText(1, type(item_data).__name__)
                    item.setText(2, str(item_data)[:VALUE_PREVIEW_LENGTH])
                    
    def on_item_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle item click"""
        field_path = item.data(0, Qt.ItemDataRole.UserRole)
        if field_path:
            # Find topic
            topic_item = item
            while topic_item.parent():
                topic_item = topic_item.parent()
            topic = topic_item.text(0)
            
            self.field_selected.emit(topic, field_path)
            
    def on_item_double_clicked(self, item: QTreeWidgetItem, column: int):
        """Handle item double click - expand arrays"""
        # Check if this is an expand item or if the parent is an array
        if item.text(1) == "expand" or "[Double-click to expand" in item.text(0) or "[Double-click to show all" in item.text(0):
            # This is an expand item, show dialog to select range
            parent = item.parent()
            if parent:
                field_path = parent.data(0, Qt.ItemDataRole.UserRole)
                if field_path:
                    # Get topic
                    topic_item = parent
                    while topic_item.parent():
                        topic_item = topic_item.parent()
                    topic = topic_item.text(0)
                    
                    # Get the array data
                    if topic in self.message_data:
                        messages = self.message_data[topic]
                        if messages:
                            # Navigate to the array
                            sample_msg = messages[0]
                            value = sample_msg
                            for field in field_path:
                                if isinstance(value, dict) and field in value:
                                    value = value[field]
                                elif isinstance(value, list) and isinstance(field, int) and field < len(value):
                                    value = value[field]
                                else:
                                    break
                            
                            if isinstance(value, list):
                                # Show array range dialog
                                self._show_array_range_dialog(parent, field_path, len(value))
        else:
            # Check if the clicked item itself is an array
            field_path = item.data(0, Qt.ItemDataRole.UserRole)
            if field_path and item.text(1).startswith("list["):
                # Get topic
                topic_item = item
                while topic_item.parent():
                    topic_item = topic_item.parent()
                topic = topic_item.text(0)
                
                # Get the array data
                if topic in self.message_data:
                    messages = self.message_data[topic]
                    if messages:
                        # Navigate to the array
                        sample_msg = messages[0]
                        value = sample_msg
                        for field in field_path:
                            if isinstance(value, dict) and field in value:
                                value = value[field]
                            elif isinstance(value, list) and isinstance(field, int) and field < len(value):
                                value = value[field]
                            else:
                                break
                        
                        if isinstance(value, list) and len(value) > 20:
                            # Show array range dialog for large arrays
                            self._show_array_range_dialog(item, field_path, len(value))
                                
    def _show_array_range_dialog(self, parent_item: QTreeWidgetItem, field_path: List[str], array_len: int):
        """Show dialog to select array range"""
        from PyQt6.QtWidgets import QDialog, QDialogButtonBox, QFormLayout
        
        dialog = QDialog(self)
        dialog.setWindowTitle("Select Array Range")
        dialog.setModal(True)
        
        layout = QFormLayout()
        
        # Info label
        info_label = QLabel(f"Array contains {array_len} elements")
        layout.addRow(info_label)
        
        # Start index
        start_spin = QSpinBox()
        start_spin.setMinimum(0)
        start_spin.setMaximum(array_len - 1)
        start_spin.setValue(0)
        layout.addRow("Start Index:", start_spin)
        
        # End index
        end_spin = QSpinBox()
        end_spin.setMinimum(0)
        end_spin.setMaximum(array_len - 1)
        end_spin.setValue(min(array_len - 1, 99))
        layout.addRow("End Index:", end_spin)
        
        # Update end spin when start changes
        def update_end_min():
            end_spin.setMinimum(start_spin.value())
        start_spin.valueChanged.connect(update_end_min)
        
        # Buttons
        buttons = QDialogButtonBox(
            QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel
        )
        buttons.accepted.connect(dialog.accept)
        buttons.rejected.connect(dialog.reject)
        layout.addRow(buttons)
        
        dialog.setLayout(layout)
        
        if dialog.exec() == QDialog.DialogCode.Accepted:
            start_idx = start_spin.value()
            end_idx = end_spin.value()
            
            # Clear existing array items (except expand item)
            for i in range(parent_item.childCount() - 1, -1, -1):
                child = parent_item.child(i)
                if child.text(1) not in ["expand", "info"]:
                    parent_item.removeChild(child)
            
            # Add selected range
            for idx in range(start_idx, min(end_idx + 1, array_len)):
                idx_item = QTreeWidgetItem(parent_item)
                idx_item.setText(0, f"[{idx}]")
                idx_item.setText(1, "element")
                idx_item.setData(0, Qt.ItemDataRole.UserRole, field_path + [idx])
                
                # Get sample value if available
                topic_item = parent_item
                while topic_item.parent():
                    topic_item = topic_item.parent()
                topic = topic_item.text(0)
                
                if topic in self.message_data:
                    messages = self.message_data[topic]
                    if messages:
                        sample_msg = messages[0]
                        value = sample_msg
                        for field in field_path + [idx]:
                            if isinstance(value, dict) and field in value:
                                value = value[field]
                            elif isinstance(value, list) and isinstance(field, int) and field < len(value):
                                value = value[field]
                            else:
                                value = None
                                break
                        if value is not None:
                            idx_item.setText(2, str(value)[:50])
            
            parent_item.setExpanded(True)


class RosbagVisualizer(QMainWindow):
    """Main application window"""
    def __init__(self):
        super().__init__()
        self.bag_data = {}
        self.current_plots = []
        self.time_options_widget = None  # Will be created in init_ui
        self.init_ui()
        
        # Initialize time options visibility after UI is created
        if self.time_options_widget:
            self.time_options_widget.setVisible(True)  # Default to Time Series
        
    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("ROS2 Bag Visualizer")
        self.setGeometry(100, 100, 1400, 800)
        
        # Create central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        
        # Create main layout
        main_layout = QHBoxLayout(central_widget)
        
        # Create splitter
        splitter = QSplitter(Qt.Orientation.Horizontal)
        main_layout.addWidget(splitter)
        
        # Left panel
        left_panel = self.create_left_panel()
        splitter.addWidget(left_panel)
        
        # Right panel
        right_panel = self.create_right_panel()
        splitter.addWidget(right_panel)
        
        # Set splitter sizes
        splitter.setSizes([400, 1000])
        
        # Create menu bar
        self.create_menu_bar()
        
        # Create status bar
        self.status_bar = QStatusBar()
        self.setStatusBar(self.status_bar)
        
        # Create progress bar
        self.progress_bar = QProgressBar()
        self.progress_bar.setVisible(False)
        self.progress_bar.setMinimum(0)
        self.progress_bar.setMaximum(100)
        self.progress_bar.setValue(0)
        self.status_bar.addPermanentWidget(self.progress_bar)
        
    def create_menu_bar(self):
        """Create menu bar"""
        menubar = self.menuBar()
        
        # File menu
        file_menu = menubar.addMenu("File")
        
        open_action = QAction("Open Rosbag Directory", self)
        open_action.triggered.connect(self.open_rosbag)
        file_menu.addAction(open_action)
        
        file_menu.addSeparator()
        
        exit_action = QAction("Exit", self)
        exit_action.triggered.connect(self.close)
        file_menu.addAction(exit_action)
        
        # View menu
        view_menu = menubar.addMenu("View")
        
        clear_action = QAction("Clear Current Plot", self)
        clear_action.triggered.connect(self.clear_current_plot)
        view_menu.addAction(clear_action)
        
        clear_all_action = QAction("Clear All Plots", self)
        clear_all_action.triggered.connect(self.clear_plots)
        view_menu.addAction(clear_all_action)
        
        view_menu.addSeparator()
        
        new_tab_action = QAction("New Plot Tab", self)
        new_tab_action.setShortcut("Ctrl+T")
        new_tab_action.triggered.connect(self.add_new_plot_tab)
        view_menu.addAction(new_tab_action)
        
        close_tab_action = QAction("Close Current Tab", self)
        close_tab_action.setShortcut("Ctrl+W")
        close_tab_action.triggered.connect(self.close_current_tab)
        view_menu.addAction(close_tab_action)
        
        rename_tab_action = QAction("Rename Current Tab", self)
        rename_tab_action.setShortcut("F2")
        rename_tab_action.triggered.connect(self.rename_current_tab)
        view_menu.addAction(rename_tab_action)
        
        view_menu.addSeparator()
        
        edit_plot_action = QAction("Edit Plot Properties", self)
        edit_plot_action.setShortcut("Ctrl+E")
        edit_plot_action.triggered.connect(self.edit_plot_properties)
        view_menu.addAction(edit_plot_action)
        
        view_menu.addSeparator()
        
        remove_line_action = QAction("Remove Plot Line", self)
        remove_line_action.setShortcut("Ctrl+D")
        remove_line_action.triggered.connect(self.remove_plot_line)
        view_menu.addAction(remove_line_action)
        
    def create_left_panel(self):
        """Create left panel with controls"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Load button
        self.load_btn = QPushButton("Load Rosbag Directory")
        self.load_btn.clicked.connect(self.open_rosbag)
        self.load_btn.setToolTip("Select a directory containing .db3 or .mcap files")
        layout.addWidget(self.load_btn)
        
        # Topics list with search
        topics_group = QGroupBox("Topics")
        topics_layout = QVBoxLayout()
        
        # Search box
        search_layout = QHBoxLayout()
        search_layout.addWidget(QLabel("Search:"))
        self.topic_search = QLineEdit()
        self.topic_search.setPlaceholderText("Filter topics...")
        self.topic_search.textChanged.connect(self.filter_topics)
        self.topic_search.setToolTip("Type to filter topic names")
        search_layout.addWidget(self.topic_search)
        topics_layout.addLayout(search_layout)
        
        self.topics_list = QListWidget()
        self.topics_list.itemClicked.connect(self.on_topic_selected)
        self.topics_list.setToolTip("Click a topic to view its fields")
        topics_layout.addWidget(self.topics_list)
        
        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)
        
        # Field selector
        fields_group = QGroupBox("Fields")
        fields_layout = QVBoxLayout()
        
        self.field_selector = DataFieldSelector()
        self.field_selector.field_selected.connect(self.on_field_selected)
        fields_layout.addWidget(self.field_selector)
        
        fields_group.setLayout(fields_layout)
        layout.addWidget(fields_group)
        
        # Plot controls
        plot_controls = QGroupBox("Plot Controls")
        controls_layout = QVBoxLayout()
        
        # Plot type
        plot_type_layout = QHBoxLayout()
        plot_type_layout.addWidget(QLabel("Plot Type:"))
        self.plot_type_combo = QComboBox()
        self.plot_type_combo.addItems(["Time Series", "X-Y Plot"])
        self.plot_type_combo.currentTextChanged.connect(self.on_plot_type_changed)
        self.plot_type_combo.setToolTip("Time Series: plot values over time\nX-Y Plot: plot one value against another")
        plot_type_layout.addWidget(self.plot_type_combo)
        controls_layout.addLayout(plot_type_layout)
        
        # Time axis options (for Time Series)
        self.time_options_widget = QWidget()
        time_options_layout = QVBoxLayout(self.time_options_widget)
        
        # Time display mode
        time_mode_layout = QHBoxLayout()
        time_mode_layout.addWidget(QLabel("Time Axis:"))
        self.time_mode_combo = QComboBox()
        self.time_mode_combo.addItems(["Absolute Time", "Elapsed Time"])
        self.time_mode_combo.setToolTip("Choose between absolute timestamps or elapsed time from origin")
        time_mode_layout.addWidget(self.time_mode_combo)
        time_options_layout.addLayout(time_mode_layout)
        
        # Elapsed time origin
        origin_layout = QHBoxLayout()
        origin_layout.addWidget(QLabel("Origin:"))
        self.time_origin_combo = QComboBox()
        self.time_origin_combo.addItems(["First Data Point", "Bag Start Time", "Custom Time"])
        self.time_origin_combo.setEnabled(False)
        self.time_origin_combo.setToolTip("Select the reference point for elapsed time calculation")
        origin_layout.addWidget(self.time_origin_combo)
        time_options_layout.addLayout(origin_layout)
        
        self.time_mode_combo.currentTextChanged.connect(self.on_time_mode_changed)
        
        controls_layout.addWidget(self.time_options_widget)
        
        # Data time range options
        self.time_range_widget = QWidget()
        time_range_layout = QVBoxLayout(self.time_range_widget)
        
        # Enable time range filtering
        time_filter_layout = QHBoxLayout()
        self.time_filter_checkbox = QCheckBox("Filter by Time Range")
        self.time_filter_checkbox.setChecked(False)
        self.time_filter_checkbox.toggled.connect(self.on_time_filter_toggled)
        self.time_filter_checkbox.setToolTip("Enable to filter data by specific time range")
        time_filter_layout.addWidget(self.time_filter_checkbox)
        time_range_layout.addLayout(time_filter_layout)
        
        # Time range controls
        time_controls_layout = QVBoxLayout()
        
        # Start time
        start_time_layout = QHBoxLayout()
        start_time_layout.addWidget(QLabel("Start Time (s):"))
        self.start_time_spinbox = QDoubleSpinBox()
        self.start_time_spinbox.setRange(0.0, 999999.0)
        self.start_time_spinbox.setValue(0.0)
        self.start_time_spinbox.setDecimals(3)
        self.start_time_spinbox.setEnabled(False)
        self.start_time_spinbox.setToolTip("Start time in seconds from bag beginning")
        start_time_layout.addWidget(self.start_time_spinbox)
        time_controls_layout.addLayout(start_time_layout)
        
        # End time
        end_time_layout = QHBoxLayout()
        end_time_layout.addWidget(QLabel("End Time (s):"))
        self.end_time_spinbox = QDoubleSpinBox()
        self.end_time_spinbox.setRange(0.0, 999999.0)
        self.end_time_spinbox.setValue(100.0)
        self.end_time_spinbox.setDecimals(3)
        self.end_time_spinbox.setEnabled(False)
        self.end_time_spinbox.setToolTip("End time in seconds from bag beginning")
        end_time_layout.addWidget(self.end_time_spinbox)
        time_controls_layout.addLayout(end_time_layout)
        
        time_range_layout.addLayout(time_controls_layout)
        controls_layout.addWidget(self.time_range_widget)
        
        # Add plot button
        self.add_plot_btn = QPushButton("Add to Plot")
        self.add_plot_btn.clicked.connect(self.add_to_plot)
        self.add_plot_btn.setEnabled(False)
        self.add_plot_btn.setToolTip("Add selected fields to the current plot")
        controls_layout.addWidget(self.add_plot_btn)
        
        # Clear plots button
        self.clear_plots_btn = QPushButton("Clear Current Plot")
        self.clear_plots_btn.clicked.connect(self.clear_current_plot)
        self.clear_plots_btn.setToolTip("Clear all data from the current plot")
        controls_layout.addWidget(self.clear_plots_btn)
        
        # Edit plot properties button
        self.edit_plot_btn = QPushButton("Edit Plot Properties")
        self.edit_plot_btn.clicked.connect(self.edit_plot_properties)
        self.edit_plot_btn.setToolTip("Edit title, axis labels, and legend names (Ctrl+E)")
        controls_layout.addWidget(self.edit_plot_btn)
        
        # Remove plot line button
        self.remove_line_btn = QPushButton("Remove Plot Line")
        self.remove_line_btn.clicked.connect(self.remove_plot_line)
        self.remove_line_btn.setToolTip("Remove a specific plot line (Ctrl+D)")
        controls_layout.addWidget(self.remove_line_btn)
        
        plot_controls.setLayout(controls_layout)
        layout.addWidget(plot_controls)
        
        # Selected fields list
        selected_group = QGroupBox("Selected Fields")
        selected_layout = QVBoxLayout()
        
        self.selected_fields_list = QListWidget()
        self.selected_fields_list.setContextMenuPolicy(Qt.ContextMenuPolicy.CustomContextMenu)
        self.selected_fields_list.customContextMenuRequested.connect(self.show_field_context_menu)
        self.selected_fields_list.itemSelectionChanged.connect(self.on_selected_field_changed)
        self.selected_fields_list.setSelectionMode(QListWidget.SelectionMode.ExtendedSelection)
        self.selected_fields_list.itemDoubleClicked.connect(self.on_selected_field_double_clicked)
        self.selected_fields_list.setToolTip("Double-click to add to plot, Delete key to remove, right-click for menu")
        selected_layout.addWidget(self.selected_fields_list)
        
        # Install event filter for keyboard shortcuts
        self.selected_fields_list.installEventFilter(self)
        
        # Button layout
        button_layout = QHBoxLayout()
        
        # Remove selected button
        self.remove_selected_btn = QPushButton("Remove Selected")
        self.remove_selected_btn.clicked.connect(self.remove_selected_fields)
        self.remove_selected_btn.setEnabled(False)
        self.remove_selected_btn.setToolTip("Remove selected fields (Delete key)")
        button_layout.addWidget(self.remove_selected_btn)
        
        # Clear all button
        self.clear_all_fields_btn = QPushButton("Clear All")
        self.clear_all_fields_btn.clicked.connect(self.clear_all_fields_with_confirmation)
        self.clear_all_fields_btn.setEnabled(False)
        self.clear_all_fields_btn.setToolTip("Remove all selected fields with confirmation")
        button_layout.addWidget(self.clear_all_fields_btn)
        
        selected_layout.addLayout(button_layout)
        
        selected_group.setLayout(selected_layout)
        layout.addWidget(selected_group)
        
        return panel
        
    def create_right_panel(self):
        """Create right panel with plot area"""
        panel = QWidget()
        layout = QVBoxLayout(panel)
        
        # Tab controls
        tab_controls = QHBoxLayout()
        
        # New tab button
        self.new_tab_btn = QPushButton("New Plot Tab")
        self.new_tab_btn.clicked.connect(self.add_new_plot_tab)
        self.new_tab_btn.setToolTip("Create a new plot tab (Ctrl+T)")
        tab_controls.addWidget(self.new_tab_btn)
        
        # Close current tab button
        self.close_tab_btn = QPushButton("Close Current Tab")
        self.close_tab_btn.clicked.connect(self.close_current_tab)
        self.close_tab_btn.setToolTip("Close the current tab (Ctrl+W)")
        tab_controls.addWidget(self.close_tab_btn)
        
        # Tab info label
        self.tab_info_label = QLabel("Plots: 1")
        self.tab_info_label.setToolTip("Double-click tab name to rename")
        tab_controls.addWidget(self.tab_info_label)
        
        tab_controls.addStretch()
        layout.addLayout(tab_controls)
        
        # Create tab widget for multiple plots
        self.plot_tabs = QTabWidget()
        self.plot_tabs.setTabsClosable(True)
        self.plot_tabs.tabCloseRequested.connect(self.close_tab)
        self.plot_tabs.setMovable(True)  # Allow tab reordering
        self.plot_tabs.currentChanged.connect(self.on_tab_changed)
        self.plot_tabs.tabBarDoubleClicked.connect(self.on_tab_double_clicked)
        self.plot_tabs.setToolTip("Double-click tab name to rename, drag to reorder")
        layout.addWidget(self.plot_tabs)
        
        # Create first plot tab (no specific plot type initially)
        self.add_plot_tab()
        self.update_tab_info()
        
        # Ensure all initial canvases are compatible
        for i in range(self.plot_tabs.count()):
            tab = self.plot_tabs.widget(i)
            if tab:
                canvas = tab.findChild(PlotCanvas)
                if canvas:
                    self._ensure_canvas_compatibility(canvas)
        
        return panel
        
    def add_plot_tab(self, title=None, plot_type=None, time_mode=None):
        """Add a new plot tab"""
        if title is None:
            # Generate unique title based on plot type and time mode
            existing_titles = [self.plot_tabs.tabText(i) for i in range(self.plot_tabs.count())]
            plot_num = 1
            
            if plot_type == "Time Series" and time_mode:
                time_suffix = " (Elapsed)" if time_mode == "Elapsed Time" else " (Absolute)"
                base_title = f"{plot_type}{time_suffix} "
            elif plot_type:
                base_title = f"{plot_type} "
            else:
                base_title = "Plot "
                
            while f"{base_title}{plot_num}" in existing_titles:
                plot_num += 1
            title = f"{base_title}{plot_num}"
        
        # Create plot widget
        plot_widget = QWidget()
        plot_layout = QVBoxLayout(plot_widget)
        
        # Create plot canvas
        canvas = PlotCanvas()
        
        # Ensure canvas has all required methods
        self._ensure_canvas_compatibility(canvas)
        
        # Store plot type and time mode in widget properties
        if plot_type:
            plot_widget.setProperty("plot_type", plot_type)
        if time_mode:
            plot_widget.setProperty("time_mode", time_mode)
        
        plot_layout.addWidget(canvas)
        
        # Create toolbar
        toolbar = NavigationToolbar(canvas, plot_widget)
        plot_layout.addWidget(toolbar)
        
        # Add to tabs
        index = self.plot_tabs.addTab(plot_widget, title)
        self.plot_tabs.setCurrentIndex(index)
        
        return canvas
    
    def get_current_tab_plot_type(self):
        """Get the plot type of the current tab"""
        current_tab = self.plot_tabs.currentWidget()
        if current_tab:
            return current_tab.property("plot_type")
        return None
    
    def get_current_tab_time_mode(self):
        """Get the time mode of the current tab"""
        current_tab = self.plot_tabs.currentWidget()
        if current_tab:
            return current_tab.property("time_mode")
        return None
    
    def set_current_tab_plot_type(self, plot_type):
        """Set the plot type of the current tab"""
        current_tab = self.plot_tabs.currentWidget()
        if current_tab:
            current_tab.setProperty("plot_type", plot_type)
    
    def set_current_tab_time_mode(self, time_mode):
        """Set the time mode of the current tab"""
        current_tab = self.plot_tabs.currentWidget()
        if current_tab:
            current_tab.setProperty("time_mode", time_mode)
        
    def _ensure_canvas_compatibility(self, canvas):
        """Ensure canvas has all required methods for compatibility"""
        # Check if plot_time_series needs to be replaced
        needs_replacement = False
        if hasattr(canvas, 'plot_time_series'):
            try:
                sig = inspect.signature(canvas.plot_time_series)
                params = list(sig.parameters.keys())
                if 'use_elapsed_time' not in params:
                    needs_replacement = True
            except:
                needs_replacement = True
        else:
            needs_replacement = True
        
        # Replace or add plot_time_series with full functionality
        if needs_replacement:
            def plot_time_series(self, timestamps, values, label="", style="-", use_elapsed_time=False, time_origin=None):
                if not timestamps or not values:
                    print("Warning: No data to plot")
                    return None
                
                if use_elapsed_time:
                    # Convert to elapsed time
                    if time_origin is None:
                        time_origin = timestamps[0]
                    times = [(t - time_origin) / 1e9 for t in timestamps]  # Convert to seconds
                    xlabel = "Elapsed Time (s)"
                else:
                    # Convert timestamps to datetime
                    times = [datetime.fromtimestamp(t / 1e9) for t in timestamps]
                    xlabel = "Time"
                
                # Plot data
                line = self.ax.plot(times, values, style, label=label)[0]
                if hasattr(self, 'plot_lines'):
                    self.plot_lines[label] = line
                else:
                    self.plot_lines = {label: line}
                
                # Format x-axis
                if not use_elapsed_time:
                    self.figure.autofmt_xdate()
                
                if label:
                    self.ax.legend()
                    
                self.ax.set_xlabel(xlabel)
                self.ax.set_ylabel("Value")
                self.ax.grid(True, alpha=0.3)
                
                self.draw()
                return line
            
            # Replace the method
            canvas.plot_time_series = plot_time_series.__get__(canvas, PlotCanvas)
        
        
        # Add other required methods
        if not hasattr(canvas, 'clear_plot'):
            def clear_plot(self):
                self.ax.clear()
                self.ax.grid(True, alpha=0.3)
                if hasattr(self, 'plot_data'):
                    self.plot_data = []
                if hasattr(self, 'plot_lines'):
                    self.plot_lines = {}
                if hasattr(self, 'custom_labels'):
                    self.custom_labels = {}
                self.draw()
            canvas.clear_plot = clear_plot.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'plot_xy'):
            def plot_xy(self, x_values, y_values, label="", style="-"):
                if not x_values or not y_values:
                    print("Warning: No data to plot")
                    return None
                    
                line = self.ax.plot(x_values, y_values, style, label=label)[0]
                if hasattr(self, 'plot_lines'):
                    self.plot_lines[label] = line
                else:
                    self.plot_lines = {label: line}
                
                if label:
                    self.ax.legend()
                    
                self.draw()
                return line
            canvas.plot_xy = plot_xy.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'get_plot_info'):
            def get_plot_info(self):
                legends = []
                if hasattr(self, 'plot_lines') and self.plot_lines:
                    legends = list(self.plot_lines.keys())
                elif self.ax.get_legend():
                    handles, labels = self.ax.get_legend_handles_labels()
                    legends = labels
                
                return {
                    'title': self.ax.get_title(),
                    'xlabel': self.ax.get_xlabel(),
                    'ylabel': self.ax.get_ylabel(),
                    'legends': legends
                }
            canvas.get_plot_info = get_plot_info.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'update_labels'):
            def update_labels(self, xlabel="", ylabel="", title=""):
                if xlabel:
                    self.ax.set_xlabel(xlabel)
                if ylabel:
                    self.ax.set_ylabel(ylabel)
                if title:
                    self.ax.set_title(title)
                self.draw()
            canvas.update_labels = update_labels.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'update_legend_labels'):
            def update_legend_labels(self, label_map):
                if hasattr(self, 'plot_lines'):
                    for old_label, new_label in label_map.items():
                        if old_label in self.plot_lines:
                            line = self.plot_lines[old_label]
                            line.set_label(new_label)
                            self.plot_lines[new_label] = self.plot_lines.pop(old_label)
                else:
                    if self.ax.get_legend():
                        handles, labels = self.ax.get_legend_handles_labels()
                        new_labels = []
                        for label in labels:
                            new_labels.append(label_map.get(label, label))
                        self.ax.legend(handles, new_labels)
                        self.draw()
                        return
                
                self.ax.legend()
                self.draw()
            canvas.update_legend_labels = update_legend_labels.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'remove_plot_line'):
            def remove_plot_line(self, label):
                if hasattr(self, 'plot_lines') and label in self.plot_lines:
                    line = self.plot_lines[label]
                    line.remove()
                    del self.plot_lines[label]
                    
                    # Update legend
                    if self.plot_lines:
                        self.ax.legend()
                    else:
                        legend = self.ax.get_legend()
                        if legend:
                            legend.remove()
                    
                    self.draw()
                    return True
                return False
            canvas.remove_plot_line = remove_plot_line.__get__(canvas, PlotCanvas)
        
        if not hasattr(canvas, 'get_plot_lines'):
            def get_plot_lines(self):
                if hasattr(self, 'plot_lines'):
                    return list(self.plot_lines.keys())
                return []
            canvas.get_plot_lines = get_plot_lines.__get__(canvas, PlotCanvas)
        
        # Ensure attributes exist
        if not hasattr(canvas, 'plot_lines'):
            canvas.plot_lines = {}
        if not hasattr(canvas, '_supports_elapsed_time'):
            canvas._supports_elapsed_time = True
        
    def add_new_plot_tab(self):
        """Add a new plot tab"""
        # Use current settings for new tab
        plot_type = self.plot_type_combo.currentText()
        time_mode = self.time_mode_combo.currentText() if plot_type == "Time Series" else None
        self.add_plot_tab(plot_type=plot_type, time_mode=time_mode)
        self.update_tab_info()
        
    def close_current_tab(self):
        """Close the current tab"""
        current_index = self.plot_tabs.currentIndex()
        if self.plot_tabs.count() > 1:
            self.close_tab(current_index)
        else:
            QMessageBox.information(self, "Info", "Cannot close the last plot tab")
            
    def close_tab(self, index: int):
        """Close a specific tab"""
        if self.plot_tabs.count() > 1:
            self.plot_tabs.removeTab(index)
            self.update_tab_info()
        else:
            QMessageBox.information(self, "Info", "Cannot close the last plot tab")
            
    def update_tab_info(self):
        """Update tab count info"""
        count = self.plot_tabs.count()
        current = self.plot_tabs.currentIndex() + 1
        self.tab_info_label.setText(f"Plot {current} of {count}")
        
    def on_tab_changed(self, index: int):
        """Handle tab change"""
        self.update_tab_info()
        
        # Ensure the current canvas is compatible
        if index >= 0:
            tab = self.plot_tabs.widget(index)
            if tab:
                canvas = tab.findChild(PlotCanvas)
                if canvas:
                    self._ensure_canvas_compatibility(canvas)
        
    def on_tab_double_clicked(self, index: int):
        """Handle tab double click - rename tab"""
        from PyQt6.QtWidgets import QInputDialog
        
        current_name = self.plot_tabs.tabText(index)
        new_name, ok = QInputDialog.getText(
            self, 
            "Rename Tab", 
            "Enter new tab name:",
            text=current_name
        )
        
        if ok and new_name:
            self.plot_tabs.setTabText(index, new_name)
            
    def on_selected_field_changed(self):
        """Handle selection change in selected fields list"""
        has_selection = len(self.selected_fields_list.selectedItems()) > 0
        has_items = self.selected_fields_list.count() > 0
        
        self.remove_selected_btn.setEnabled(has_selection)
        self.clear_all_fields_btn.setEnabled(has_items)
        
    def show_field_context_menu(self, position):
        """Show context menu for selected fields"""
        menu = QMenu()
        
        # Check if there's an item at the position
        item = self.selected_fields_list.itemAt(position)
        if item:
            remove_action = menu.addAction("Remove")
            remove_action.triggered.connect(lambda: self.remove_field_item(item))
            menu.addSeparator()
        
        # Always show clear all option if there are items
        if self.selected_fields_list.count() > 0:
            clear_all_action = menu.addAction("Clear All...")
            clear_all_action.triggered.connect(self.clear_all_fields_with_confirmation)
            clear_all_action.setToolTip("Remove all selected fields with confirmation")
        
        # Only show menu if there are actions
        if menu.actions():
            menu.exec(self.selected_fields_list.mapToGlobal(position))
            
    def remove_field_item(self, item: QListWidgetItem):
        """Remove a specific field item"""
        row = self.selected_fields_list.row(item)
        self.selected_fields_list.takeItem(row)
        
        # Update button state
        if self.selected_fields_list.count() == 0:
            self.add_plot_btn.setEnabled(False)
            self.clear_all_fields_btn.setEnabled(False)
            
    def remove_selected_fields(self):
        """Remove selected field items"""
        items = self.selected_fields_list.selectedItems()
        for item in items:
            row = self.selected_fields_list.row(item)
            self.selected_fields_list.takeItem(row)
            
        # Update button state
        if self.selected_fields_list.count() == 0:
            self.add_plot_btn.setEnabled(False)
            self.clear_all_fields_btn.setEnabled(False)
            
    def remove_all_fields(self):
        """Remove all field items"""
        self.selected_fields_list.clear()
        self.add_plot_btn.setEnabled(False)
        self.clear_all_fields_btn.setEnabled(False)
    
    def clear_all_fields_with_confirmation(self):
        """Remove all field items with confirmation dialog"""
        if self.selected_fields_list.count() == 0:
            return
        
        # Show confirmation dialog
        field_count = self.selected_fields_list.count()
        reply = QMessageBox.question(
            self,
            "Clear All Selected Fields",
            f"Are you sure you want to remove all {field_count} selected fields?\n\n"
            f"This action cannot be undone.",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No,
            QMessageBox.StandardButton.No  # Default to No for safety
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            self.remove_all_fields()
        
    def rename_current_tab(self):
        """Rename the current tab"""
        current_index = self.plot_tabs.currentIndex()
        if current_index >= 0:
            self.on_tab_double_clicked(current_index)
            
    def on_selected_field_double_clicked(self, item: QListWidgetItem):
        """Handle double click on selected field - quick add to plot"""
        # Store current selected items
        selected_items = self.selected_fields_list.selectedItems()
        
        # Set plot type to Time Series (most common use case)
        self.plot_type_combo.setCurrentText("Time Series")
        
        # Clear selection and select only the double-clicked item
        self.selected_fields_list.clearSelection()
        item.setSelected(True)
        
        # Temporarily set to absolute time for quick plot
        original_time_mode = self.time_mode_combo.currentText()
        self.time_mode_combo.setCurrentText("Absolute Time")
        
        # Add to plot
        self.add_to_plot()
        
        # Restore time mode
        self.time_mode_combo.setCurrentText(original_time_mode)
        
        # Restore previous selection if needed
        for selected_item in selected_items:
            if selected_item != item:
                selected_item.setSelected(True)
        
    def eventFilter(self, source, event):
        """Event filter for keyboard shortcuts"""
        if source == self.selected_fields_list and event.type() == event.Type.KeyPress:
            key_event = event
            if key_event.key() == Qt.Key.Key_Delete:
                self.remove_selected_fields()
                return True
        return super().eventFilter(source, event)
        
    def open_rosbag(self):
        """Open rosbag directory"""
        directory = QFileDialog.getExistingDirectory(
            self, 
            "Select Rosbag Directory",
            os.path.expanduser("~")
        )
        
        if directory:
            self.load_rosbag(directory)
            
    def load_rosbag(self, directory: str):
        """Load rosbag data in background"""
        # Check if ROS2 is available
        if not HAS_ROS2:
            QMessageBox.critical(self, "Error", 
                "ROS2 packages not found. Please install:\n"
                "- rosbag2-py\n"
                "- rclpy\n"
                "- ros-<distro>-rosbag2-storage-mcap (for MCAP support)")
            return
            
        # Check if directory contains rosbag files
        db_files = [f for f in os.listdir(directory) if f.endswith('.db3')]
        mcap_files = [f for f in os.listdir(directory) if f.endswith('.mcap')]
        
        all_files = [(f, 'sqlite3') for f in db_files] + [(f, 'mcap') for f in mcap_files]
        
        if not all_files:
            QMessageBox.warning(self, "Warning", "No rosbag files (.db3 or .mcap) found in the selected directory.")
            return
            
        # If multiple files, let user choose
        bag_file, storage_id = all_files[0]
        if len(all_files) > 1:
            from PyQt6.QtWidgets import QInputDialog
            file_names = [f[0] for f in all_files]
            chosen_file, ok = QInputDialog.getItem(
                self, "Select Bag File", 
                "Multiple bag files found. Select one:",
                file_names, 0, False
            )
            if ok:
                for fname, sid in all_files:
                    if fname == chosen_file:
                        bag_file = fname
                        storage_id = sid
                        break
            else:
                return
            
        # Clear previous data
        self.bag_data = {}
        self.topics_list.clear()
        self.selected_fields_list.clear()
        self.topic_search.clear()  # Clear search box
        
        # Show progress
        self.progress_bar.setVisible(True)
        self.progress_bar.setValue(0)
        self.status_bar.showMessage(f"Starting to load {bag_file}...")
        
        # Create loader thread
        bag_path = os.path.join(directory, bag_file)
        self.loader_thread = DataLoaderThread(bag_path, storage_id)
        self.loader_thread.progress.connect(self.on_load_progress)
        self.loader_thread.status.connect(self.on_load_status)
        self.loader_thread.finished.connect(self.on_load_finished)
        self.loader_thread.error.connect(self.on_load_error)
        self.loader_thread.start()
        
    def on_load_progress(self, progress: int):
        """Handle load progress"""
        self.progress_bar.setValue(min(progress, 100))
        
    def on_load_status(self, status: str):
        """Handle load status"""
        self.status_bar.showMessage(status)
        
    def on_load_finished(self, data: dict):
        """Handle load finished"""
        self.bag_data = data
        
        # Ensure progress bar shows 100% before hiding
        self.progress_bar.setValue(100)
        
        # Update topics list
        for topic, info in data['topic_info'].items():
            item = QListWidgetItem(f"{topic} ({info['count']} messages)")
            item.setData(Qt.ItemDataRole.UserRole, topic)
            self.topics_list.addItem(item)
        
        # Update time range limits
        self.update_time_range_limits()
        
        # Brief delay to show 100% completion, then hide
        QTimer.singleShot(200, lambda: (
            self.progress_bar.setVisible(False),
            self.status_bar.showMessage(f"Loaded {data['total_messages']} messages from {len(data['topic_info'])} topics")
        ))
        
    def on_plot_type_changed(self, plot_type: str):
        """Handle plot type change"""
        # Show/hide time options based on plot type
        self.time_options_widget.setVisible(plot_type == "Time Series")
        
    def on_time_mode_changed(self, mode: str):
        """Handle time mode change"""
        # Enable/disable origin combo based on mode
        self.time_origin_combo.setEnabled(mode == "Elapsed Time")
        
    def on_time_filter_toggled(self, checked: bool):
        """Handle time filter checkbox toggle"""
        self.start_time_spinbox.setEnabled(checked)
        self.end_time_spinbox.setEnabled(checked)
        
        # Auto-update time range when enabled if data is loaded
        if checked and hasattr(self, 'bag_data') and self.bag_data:
            self.update_time_range_limits()
    
    def update_time_range_limits(self):
        """Update time range spinbox limits based on loaded data"""
        if not hasattr(self, 'bag_data') or not self.bag_data:
            return
        
        # Find earliest and latest timestamps
        all_first_times = []
        all_last_times = []
        
        for topic_messages in self.bag_data['messages'].values():
            if topic_messages:
                all_first_times.append(topic_messages[0]['_timestamp'])
                all_last_times.append(topic_messages[-1]['_timestamp'])
        
        if all_first_times and all_last_times:
            min_time_ns = min(all_first_times)
            max_time_ns = max(all_last_times)
            
            # Convert to seconds relative to start
            duration_s = (max_time_ns - min_time_ns) / 1e9
            
            # Update spinbox ranges and values
            self.start_time_spinbox.setMaximum(duration_s)
            self.end_time_spinbox.setMaximum(duration_s)
            self.end_time_spinbox.setValue(duration_s)
        
    def edit_plot_properties(self):
        """Edit current plot properties"""
        current_tab = self.plot_tabs.currentWidget()
        if not current_tab:
            return
            
        canvas = current_tab.findChild(PlotCanvas)
        if not canvas:
            return
        
        # Ensure canvas has all required methods
        self._ensure_canvas_compatibility(canvas)
            
        # Get current plot info
        plot_info = canvas.get_plot_info()
        
        # Show dialog
        dialog = PlotPropertiesDialog(plot_info, self)
        if dialog.exec() == QDialog.DialogCode.Accepted:
            props = dialog.get_properties()
            
            # Update plot
            canvas.update_labels(
                xlabel=props['xlabel'],
                ylabel=props['ylabel'],
                title=props['title']
            )
            
            # Update legend labels
            if props['legend_map']:
                canvas.update_legend_labels(props['legend_map'])
        
    def filter_topics(self, text: str):
        """Filter topics based on search text"""
        search_text = text.lower()
        
        for i in range(self.topics_list.count()):
            item = self.topics_list.item(i)
            topic = item.data(Qt.ItemDataRole.UserRole)
            
            # Show/hide based on search
            if search_text in topic.lower():
                item.setHidden(False)
            else:
                item.setHidden(True)
        
    def on_load_error(self, error: str):
        """Handle load error"""
        self.progress_bar.setVisible(False)
        QMessageBox.critical(self, "Error", error)
        self.status_bar.showMessage("Failed to load rosbag")
        
    def on_topic_selected(self, item: QListWidgetItem):
        """Handle topic selection"""
        topic = item.data(Qt.ItemDataRole.UserRole)
        if topic in self.bag_data['messages']:
            messages = self.bag_data['messages'][topic]
            self.field_selector.set_message_data(topic, messages)
            
    def on_field_selected(self, topic: str, field_path: List[str]):
        """Handle field selection"""
        # Create display string for the field path
        field_str_parts = []
        for f in field_path:
            if isinstance(f, int):
                field_str_parts.append(f"[{f}]")
            else:
                if field_str_parts and not field_str_parts[-1].endswith(']'):
                    field_str_parts.append('.')
                field_str_parts.append(str(f))
        
        field_str = f"{topic}: {''.join(field_str_parts)}"
        
        # Check if already added
        for i in range(self.selected_fields_list.count()):
            if self.selected_fields_list.item(i).text() == field_str:
                return
                
        item = QListWidgetItem(field_str)
        item.setData(Qt.ItemDataRole.UserRole, (topic, field_path))
        self.selected_fields_list.addItem(item)
        
        # Enable buttons
        self.add_plot_btn.setEnabled(True)
        self.clear_all_fields_btn.setEnabled(True)
        
    def extract_field_data(self, messages: List[dict], field_path: List[str]) -> Tuple[List[float], List[Any]]:
        """Extract field data from messages"""
        timestamps = []
        values = []
        
        # Get time range filtering settings
        use_time_filter = self.time_filter_checkbox.isChecked()
        start_time_s = self.start_time_spinbox.value()
        end_time_s = self.end_time_spinbox.value()
        
        # Calculate time range in nanoseconds if filtering is enabled
        start_time_ns = None
        end_time_ns = None
        if use_time_filter and messages:
            # Find the earliest timestamp in the entire bag
            all_first_times = []
            for topic_messages in self.bag_data['messages'].values():
                if topic_messages:
                    all_first_times.append(topic_messages[0]['_timestamp'])
            
            if all_first_times:
                bag_start_time = min(all_first_times)
                start_time_ns = bag_start_time + (start_time_s * 1e9)
                end_time_ns = bag_start_time + (end_time_s * 1e9)
        
        for msg in messages:
            # Get timestamp
            timestamp = msg['_timestamp']
            
            # Apply time filter if enabled
            if use_time_filter and start_time_ns is not None and end_time_ns is not None:
                if timestamp < start_time_ns or timestamp > end_time_ns:
                    continue
            
            timestamps.append(timestamp)
            
            # Navigate to field
            value = msg
            for field in field_path:
                if isinstance(value, dict) and field in value:
                    value = value[field]
                elif isinstance(value, list) and isinstance(field, int) and field < len(value):
                    value = value[field]
                else:
                    value = None
                    break
            
            # Convert to numeric if possible
            if value is not None:
                try:
                    # Try to convert to float
                    if isinstance(value, (int, float)):
                        value = float(value)
                    elif isinstance(value, str):
                        value = float(value)
                    else:
                        # For non-numeric types, skip
                        value = None
                except (ValueError, TypeError):
                    value = None
                    
            values.append(value)
            
        return timestamps, values
        
    def add_to_plot(self):
        """Add selected fields to plot"""
        if self.selected_fields_list.count() == 0:
            return
            
        plot_type = self.plot_type_combo.currentText()
        time_mode = self.time_mode_combo.currentText() if plot_type == "Time Series" else None
        
        # Check if current tab has different plot configuration
        current_tab_plot_type = self.get_current_tab_plot_type()
        current_tab_time_mode = self.get_current_tab_time_mode()
        
        needs_new_tab = False
        
        # Check if plot type is different
        if current_tab_plot_type and current_tab_plot_type != plot_type:
            needs_new_tab = True
        
        # For Time Series, also check if time mode is different
        if (not needs_new_tab and plot_type == "Time Series" and 
            current_tab_time_mode and current_tab_time_mode != time_mode):
            needs_new_tab = True
        
        if needs_new_tab:
            # Create new tab for different configuration
            canvas = self.add_plot_tab(plot_type=plot_type, time_mode=time_mode)
        else:
            # Use current tab
            current_tab = self.plot_tabs.currentWidget()
            if not current_tab:
                # No tab exists, create first one
                canvas = self.add_plot_tab(plot_type=plot_type, time_mode=time_mode)
            else:
                canvas = current_tab.findChild(PlotCanvas)
                if not canvas:
                    return
                # Set plot type and time mode if not set
                if not current_tab_plot_type:
                    self.set_current_tab_plot_type(plot_type)
                if plot_type == "Time Series" and not current_tab_time_mode:
                    self.set_current_tab_time_mode(time_mode)
        
        # Ensure canvas has all required methods
        self._ensure_canvas_compatibility(canvas)
        
        try:
            if plot_type == "Time Series":
                # Get time mode settings
                use_elapsed_time = self.time_mode_combo.currentText() == "Elapsed Time"
                time_origin = None
                
                if use_elapsed_time:
                    origin_mode = self.time_origin_combo.currentText()
                    if origin_mode == "First Data Point":
                        # Will use first timestamp as origin
                        time_origin = None
                    elif origin_mode == "Bag Start Time":
                        # Find earliest timestamp efficiently
                        first_timestamps = [
                            topic_messages[0]['_timestamp'] 
                            for topic_messages in self.bag_data['messages'].values() 
                            if topic_messages
                        ]
                        time_origin = min(first_timestamps) if first_timestamps else None
                    elif origin_mode == "Custom Time":
                        # Show dialog to input custom time
                        from PyQt6.QtWidgets import QInputDialog
                        
                        # Get current time range efficiently
                        all_first_times = [
                            topic_messages[0]['_timestamp'] 
                            for topic_messages in self.bag_data['messages'].values() 
                            if topic_messages
                        ]
                        all_last_times = [
                            topic_messages[-1]['_timestamp'] 
                            for topic_messages in self.bag_data['messages'].values() 
                            if topic_messages
                        ]
                        
                        if all_first_times and all_last_times:
                            min_time = min(all_first_times)
                            max_time = max(all_last_times)
                            
                            # Convert to seconds for display
                            min_sec = min_time / 1e9
                            max_sec = max_time / 1e9
                            
                            time_str, ok = QInputDialog.getText(
                                self,
                                "Custom Time Origin",
                                f"Enter time origin in seconds since epoch\n"
                                f"(Range: {min_sec:.3f} - {max_sec:.3f}):",
                                text=str(min_sec)
                            )
                            
                            if ok:
                                try:
                                    time_origin = float(time_str) * 1e9  # Convert to nanoseconds
                                except ValueError:
                                    QMessageBox.warning(self, "Invalid Input", "Please enter a valid number")
                                    time_origin = None
                            else:
                                time_origin = None
                
                # Plot time series
                for i in range(self.selected_fields_list.count()):
                    item = self.selected_fields_list.item(i)
                    topic, field_path = item.data(Qt.ItemDataRole.UserRole)
                    
                    if topic in self.bag_data['messages']:
                        messages = self.bag_data['messages'][topic]
                        timestamps, values = self.extract_field_data(messages, field_path)
                        
                        # Filter out None values
                        valid_data = [(t, v) for t, v in zip(timestamps, values) if v is not None]
                        if valid_data:
                            timestamps, values = zip(*valid_data)
                            
                            # Create label with proper formatting
                            label_parts = [topic]
                            for f in field_path:
                                if isinstance(f, int):
                                    label_parts.append(f"[{f}]")
                                else:
                                    label_parts.append(f".{f}")
                            label = ''.join(label_parts)
                            
                            # Check if canvas supports the new elapsed time feature
                            if hasattr(canvas, '_supports_elapsed_time') and canvas._supports_elapsed_time:
                                # Use new signature with elapsed time support
                                canvas.plot_time_series(
                                    timestamps, 
                                    values, 
                                    label, 
                                    "-",  # style
                                    use_elapsed_time, 
                                    time_origin
                                )
                            else:
                                # Fallback to basic time series plot
                                print("Note: This plot canvas doesn't support elapsed time. Using absolute time.")
                                # Convert timestamps based on settings
                                if use_elapsed_time:
                                    if time_origin is None:
                                        time_origin = timestamps[0]
                                    # Convert to elapsed seconds
                                    elapsed_times = [(t - time_origin) / 1e9 for t in timestamps]
                                    # Create a simple plot
                                    line = canvas.ax.plot(elapsed_times, values, "-", label=label)[0]
                                    canvas.ax.set_xlabel("Elapsed Time (s)")
                                else:
                                    # Use original plot_time_series
                                    times = [datetime.fromtimestamp(t / 1e9) for t in timestamps]
                                    line = canvas.ax.plot(times, values, "-", label=label)[0]
                                    canvas.figure.autofmt_xdate()
                                    canvas.ax.set_xlabel("Time")
                                
                                canvas.ax.set_ylabel("Value")
                                canvas.ax.legend()
                                canvas.draw()
                        else:
                            # Create field string for warning
                            field_str_parts = []
                            for f in field_path:
                                if isinstance(f, int):
                                    field_str_parts.append(f"[{f}]")
                                else:
                                    if field_str_parts and not field_str_parts[-1].endswith(']'):
                                        field_str_parts.append('.')
                                    field_str_parts.append(str(f))
                            field_str = ''.join(field_str_parts)
                            
                            QMessageBox.warning(self, "Warning", 
                                f"No valid numeric data found for {topic}{field_str}")
                            
            elif plot_type == "X-Y Plot":
                # Need at least 2 fields
                if self.selected_fields_list.count() < 2:
                    QMessageBox.warning(self, "Warning", "Please select at least 2 fields for X-Y plot")
                    return
                    
                # Get first two fields
                item1 = self.selected_fields_list.item(0)
                item2 = self.selected_fields_list.item(1)
                
                topic1, field_path1 = item1.data(Qt.ItemDataRole.UserRole)
                topic2, field_path2 = item2.data(Qt.ItemDataRole.UserRole)
                
                # Extract data
                values1 = []
                values2 = []
                
                if topic1 in self.bag_data['messages']:
                    messages1 = self.bag_data['messages'][topic1]
                    _, values1 = self.extract_field_data(messages1, field_path1)
                    
                if topic2 in self.bag_data['messages']:
                    messages2 = self.bag_data['messages'][topic2]
                    _, values2 = self.extract_field_data(messages2, field_path2)
                    
                # Align data
                min_len = min(len(values1), len(values2))
                values1 = values1[:min_len]
                values2 = values2[:min_len]
                
                # Filter out None values
                valid_data = [(v1, v2) for v1, v2 in zip(values1, values2) if v1 is not None and v2 is not None]
                if valid_data:
                    values1, values2 = zip(*valid_data)
                    
                    # Create labels with proper formatting
                    def format_field_path(topic, field_path):
                        parts = [topic]
                        for f in field_path:
                            if isinstance(f, int):
                                parts.append(f"[{f}]")
                            else:
                                parts.append(f".{f}")
                        return ''.join(parts)
                    
                    label1 = format_field_path(topic1, field_path1)
                    label2 = format_field_path(topic2, field_path2)
                    
                    canvas.plot_xy(values1, values2, f"{label1} vs {label2}")
                    canvas.update_labels(xlabel=label1, ylabel=label2)
                else:
                    QMessageBox.warning(self, "Warning", "No valid numeric data found for X-Y plot")
                    
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to plot data: {str(e)}")
            
        # Don't clear selected fields to allow easy re-plotting
        
    def clear_current_plot(self):
        """Clear the current plot"""
        current_tab = self.plot_tabs.currentWidget()
        if current_tab:
            canvas = current_tab.findChild(PlotCanvas)
            if canvas:
                # Ensure canvas has all required methods
                self._ensure_canvas_compatibility(canvas)
                canvas.clear_plot()
                
    def clear_plots(self):
        """Clear all plots"""
        reply = QMessageBox.question(
            self, 
            "Clear All Plots", 
            "Are you sure you want to clear all plots?",
            QMessageBox.StandardButton.Yes | QMessageBox.StandardButton.No
        )
        
        if reply == QMessageBox.StandardButton.Yes:
            for i in range(self.plot_tabs.count()):
                tab = self.plot_tabs.widget(i)
                if tab:
                    canvas = tab.findChild(PlotCanvas)
                    if canvas:
                        # Ensure canvas has all required methods
                        self._ensure_canvas_compatibility(canvas)
                        canvas.clear_plot()
    
    def remove_plot_line(self):
        """Remove a specific plot line"""
        current_tab = self.plot_tabs.currentWidget()
        if not current_tab:
            return
            
        canvas = current_tab.findChild(PlotCanvas)
        if not canvas:
            return
        
        # Ensure canvas has all required methods
        self._ensure_canvas_compatibility(canvas)
        
        # Get available plot lines
        plot_lines = canvas.get_plot_lines()
        if not plot_lines:
            QMessageBox.information(self, "Info", "No plot lines to remove")
            return
        
        # Show selection dialog
        from PyQt6.QtWidgets import QInputDialog
        
        selected_line, ok = QInputDialog.getItem(
            self,
            "Remove Plot Line",
            "Select a plot line to remove:",
            plot_lines,
            0,
            False
        )
        
        if ok and selected_line:
            success = canvas.remove_plot_line(selected_line)
            if success:
                self.status_bar.showMessage(f"Removed plot line: {selected_line}")
            else:
                QMessageBox.warning(self, "Warning", f"Failed to remove plot line: {selected_line}")


def main():
    """Main function"""
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create and show main window
    window = RosbagVisualizer()
    window.show()
    
    sys.exit(app.exec())


if __name__ == '__main__':
    main()