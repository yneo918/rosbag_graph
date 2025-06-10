# rosbag_graph

A ROS2 bag file visualization tool - A PyQt6-based GUI application for loading and visualizing data from ROS2 bag files (.db3 and .mcap).

## Features

- **Load ROS2 bag files**: Supports both SQLite3 (.db3) and MCAP (.mcap) formats
- **Topic exploration**: Display all topics in the bag file with search/filter functionality
- **Message field selection**: Tree view to explore message structure and select fields to plot
- **Array data support**: Handles ROS2 array types like Float32MultiArray, Int32MultiArray with individual element selection and range specification
- **Multiple plot types**:
  - Time series plots: Display data changes over time (absolute or elapsed time)
  - X-Y plots: Show correlation between two fields
- **Intelligent tab management**: Automatically creates new tabs when switching between different plot types or time modes
- **Time range filtering**: Filter data by specific time ranges for focused analysis
- **Plot customization**: Edit plot properties including title, axis labels, and legend entries
- **Interactive plots**: Zoom, pan, save, and other standard Matplotlib features
- **Field management**: Easy field selection with bulk operations and confirmation dialogs
- **Keyboard shortcuts**: Quick actions for common operations

## Requirements

### ROS2 Environment
- ROS2 (Humble, Iron, Jazzy, etc.) must be installed
- ROS2 environment must be sourced (`source /opt/ros/<distro>/setup.bash`)

### Python Packages
```bash
# Required packages
pip install PyQt6 matplotlib

# ROS2 packages (provided by ROS2 environment)
# Install via apt (example for Humble)
sudo apt install ros-humble-rosbag2-py ros-humble-rclpy

# MCAP format support (optional)
sudo apt install ros-humble-rosbag2-storage-mcap
```

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yneo918/rosbag_graph.git
cd rosbag_graph
```

2. Source ROS2 environment:
```bash
source /opt/ros/humble/setup.bash  # or your ROS2 distribution
```

## Usage

### Basic Usage

1. **Launch the application**:
```bash
python3 src/ros2bag_visualizer.py
```

2. **Load a bag file**:
   - Click the "Load Rosbag Directory" button
   - Select a directory containing bag files (.db3 or .mcap)
   - If multiple bag files exist, choose which one to load

3. **Select a topic**:
   - Click on a topic from the "Topics" list in the left panel
   - Use the search box to filter topics by name

4. **Select fields**:
   - After selecting a topic, the "Fields" panel shows the message structure in a tree view
   - Click on fields you want to plot
   - For arrays, you can select individual elements (e.g., data[0], data[1])
   - For large arrays, double-click to specify the range to display

5. **Create plots**:
   - Select plot type (Time Series or X-Y Plot)
   - Choose time display mode for time series (Absolute Time or Elapsed Time)
   - Configure time range filtering if needed
   - Click "Add to Plot" button
   - Double-click selected fields for instant plotting

### Advanced Features

#### Time Series Options
- **Absolute Time**: Display actual timestamps from the bag file
- **Elapsed Time**: Show time relative to a reference point
  - First Data Point: Time starts from the first plotted data
  - Bag Start Time: Time starts from the earliest message in the bag
  - Custom Time: Set a specific reference time

#### Time Range Filtering
- Enable "Filter by Time Range" checkbox
- Set start and end times in seconds from bag beginning
- Only data within the specified range will be plotted
- Automatically adjusts range limits based on loaded bag duration

#### Intelligent Tab Management
- Tabs are automatically created when switching between:
  - Different plot types (Time Series ↔ X-Y Plot)
  - Different time modes (Absolute ↔ Elapsed Time)
- Tab names reflect the plot configuration:
  - "Time Series (Absolute) 1"
  - "Time Series (Elapsed) 1" 
  - "X-Y Plot 1"
- Selected fields are preserved for easy re-plotting with different settings

#### Compare Multiple Fields
- Select multiple fields before clicking "Add to Plot" to overlay them on the same graph
- Fields remain selected after plotting for easy experimentation

#### X-Y Plots
- Select at least 2 fields
- Set Plot Type to X-Y Plot
- The first two fields will be used as X and Y axes

#### Multi-tab Plotting
- Click "New Plot Tab" button to create a new tab
- Double-click tab names to rename them
- Drag tabs to reorder
- Close tabs with the × button or Ctrl+W

#### Plot Customization
- Click "Edit Plot Properties" or press Ctrl+E
- Modify plot title, axis labels, and legend entries
- Changes are applied immediately

#### Field Management
- **Individual removal**: Select fields and press Delete or use "Remove Selected"
- **Bulk removal**: Use "Clear All" button with confirmation dialog
- **Context menu**: Right-click for additional options
- Selected fields persist after plotting for easy re-use

#### Keyboard Shortcuts
- `Ctrl+T`: Create new plot tab
- `Ctrl+W`: Close current tab
- `Ctrl+E`: Edit plot properties
- `Ctrl+D`: Remove plot line
- `F2`: Rename current tab
- `Delete`: Remove selected fields from list

### Plot Controls

Each plot includes Matplotlib's standard toolbar:
- **Pan**: Drag to move the graph
- **Zoom**: Rectangle select to zoom in
- **Home**: Return to original view
- **Back/Forward**: Navigate view history
- **Save**: Save plot as image

## Workflow Examples

### Comparing Time Display Modes
1. Load bag file and select fields
2. Create Time Series plot with Absolute Time → "Time Series (Absolute) 1" tab
3. Change to Elapsed Time and plot again → "Time Series (Elapsed) 1" tab created automatically
4. Switch between tabs to compare different time representations

### Analyzing Specific Time Periods
1. Load bag file
2. Enable "Filter by Time Range"
3. Set start/end times to focus on events of interest
4. Create plots with filtered data
5. Adjust time range and re-plot to compare different periods

### Multi-perspective Analysis
1. Select sensor data fields
2. Create Time Series plot to see temporal behavior
3. Switch to X-Y Plot to see correlations
4. Use different tabs to compare various field combinations

## Troubleshooting

### ROS2 packages not found
```
Error: ROS2 packages not found. Please install:
- rosbag2-py
- rclpy
```
→ Make sure your ROS2 environment is sourced

### Cannot open MCAP files
```
Failed to open bag file: ...
Make sure you have the required rosbag2 storage plugin installed.
```
→ Install the `ros-<distro>-rosbag2-storage-mcap` package

### High memory usage
Large bag files load all messages into memory, which can result in high memory usage. Consider:
- Using time range filtering to load only needed data
- Using smaller bag files or splitting large recordings
- Closing unused tabs to free memory

### Performance with large arrays
For messages with large arrays:
- Use array range selection to display only relevant elements
- Consider plotting individual array elements rather than entire arrays
- Large arrays may impact loading and plotting performance

## Supported Message Types

Supports standard ROS2 message types including:
- `std_msgs/Float32MultiArray`, `std_msgs/Int32MultiArray`, etc.
- `sensor_msgs/PointCloud2`, `sensor_msgs/Image`
- Various `geometry_msgs` package messages
- Custom message types (if properly built and sourced)

The tool automatically handles:
- Python `array.array` types commonly used in ROS2 messages
- Nested message structures
- Various numeric and string data types

## License

[Add license information]