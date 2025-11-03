import sys
import serial
import numpy as np
import time
import re
from collections import deque
from threading import Thread, Lock, Event
from scipy.signal import savgol_filter
import logging
import os
from enum import Enum, auto

from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                            QHBoxLayout, QGridLayout, QLabel, QPushButton, 
                            QLineEdit, QRadioButton, QButtonGroup, QFrame,
                            QComboBox, QSlider, QGroupBox, QMessageBox, QSplashScreen, QProgressBar)
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QRect
from PyQt5.QtGui import QPainter, QPen, QColor, QFont, QBrush, QPalette, QPixmap

import pyqtgraph as pg

# Configure logging
logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.FileHandler("temperature_controller.log"),
        logging.StreamHandler()
    ]
)

logger = logging.getLogger("TemperatureController")

class SystemState(Enum):
    """Enum representing system states for state machine implementation"""
    INITIALIZING = auto()
    CONNECTING = auto()
    CONNECTED = auto()
    DISCONNECTED = auto()
    ERROR = auto()
    RUNNING = auto()
    STOPPED = auto()
    RECONNECTING = auto()

class TemperatureControllerGUI(QMainWindow):
    """PyQt-based GUI for Temperature Controller System with error handling"""
    
    # Custom signals for thread-safe operations
    connection_status_signal = pyqtSignal(bool, str)
    error_signal = pyqtSignal(str)
    system_state_changed_signal = pyqtSignal(SystemState)
    update_ui_signal = pyqtSignal(dict)  # Add this line here
    
    def __init__(self, port=None, baudrate=115200, defer_port_discovery=False):
        super().__init__()
    
        # Communication Setup
        self.serial_port = None
        self.baudrate = baudrate
        self.ser = None
        self.serial_lock = Lock()
        self.reconnect_event = Event()
        self.max_reconnect_attempts = 10
        self.reconnect_delay = 2  # seconds
        self.connection_alive = False
        
        # Store the defer_port_discovery flag
        self._defer_port_discovery = defer_port_discovery
        
        # System State
        self.system_state = SystemState.INITIALIZING
        
        
        # Data Storage
        self.max_points = 3000
        self.display_points = 600  # Only display last 10 minutes in plot
        self.time_data = deque(maxlen=self.max_points)
        self.temp_data = deque(maxlen=self.max_points)
        self.setpoint_data = deque(maxlen=self.max_points)
        self.overshoot_data = deque(maxlen=self.max_points)
        self.start_time = time.time()
        
        # Control Parameters
        self.hotgun_temp = 0.0
        self.current_setpoint = 20.0
        self.max_setpoint = 180.0  # Add this line to define max setpoint
        self.current_fan_speed = 30  # Initialize to minimum allowed value when running
        self.max_overshoot = 0
        self.settling_time = 0
        self.kp = 0
        self.ki = 0
        self.kd = 0
        self.temp_threshold = 200.0  # Default threshold
        self.feedback_source = "PT1000"  # Default feedback source
        
        # System State
        self.running = True
        self.system_running = False
        self.status_message = "Please select a COM port and click Connect"  # Changed status message
        self.initialization_complete = False
        
        # Protection state tracking
        self.protection_active = False  # Flag to track if a protection alarm/warning is active
        self.current_protection_message = ""  # Store the current protection message
        
        # Communication health monitoring
        self.last_data_received_time = 0
        self.communication_timeout = 15  # seconds
        
        # Raw temp data for smoothing
        self.raw_temp_data = deque(maxlen=self.max_points)
        self.smoothing_window = 5  # Window size for moving average
        
        # Setup UI
        self.initUI()
        
        # Populate port list initially - but only if not deferred
        if not self._defer_port_discovery:
            self.refresh_port_list()
        
        # Setup data update timer (200ms)
        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_plot)
        self.update_timer.start(200)  # 200ms matches original implementation
        
        # Setup connection health check timer
        self.health_check_timer = QTimer()
        self.health_check_timer.timeout.connect(self.check_connection_health)
        self.health_check_timer.start(1000)  # Check every second
        
        # Connect signals to slots
        self.connection_status_signal.connect(self.on_connection_status_change)
        self.error_signal.connect(self.on_error)
        self.system_state_changed_signal.connect(self.on_system_state_change)
        self.update_ui_signal.connect(self.update_ui_from_data)

        # Start Serial Thread with proper initialization but don't try to connect automatically
        self.serial_thread = Thread(target=self.serial_thread_function)
        self.serial_thread.daemon = True
        self.serial_thread.start()
    
    def normalize_fan_speed(self, value):
        """Normalize fan speed to the nearest valid step (30, 40, 50, 60, 70, 80, 90, 100)"""
        valid_speeds = [30, 40, 50, 60, 70, 80, 90, 100]
        # Find closest value in valid_speeds
        return min(valid_speeds, key=lambda x: abs(x - value))
        
    def serial_thread_function(self):
        """Main function for the serial communication thread with improved reconnection logic"""
        while self.running:
            try:
                # Only attempt connection if we have a port and are in a reconnection state
                if (self.serial_port is not None) and (
                    self.system_state == SystemState.DISCONNECTED or
                    self.system_state == SystemState.RECONNECTING):
                    
                    # Small delay before reconnection attempt
                    if self.system_state == SystemState.RECONNECTING:
                        # Gradually increase reconnect delay with more failures
                        if hasattr(self, '_reconnection_attempts'):
                            delay = min(5.0, 1.0 + (self._reconnection_attempts * 0.5))
                            time.sleep(delay)
                        else:
                            time.sleep(1)
                    
                    # Attempt connection
                    success = self.connect_serial()
                    
                    # Dynamically adjust reconnection delay based on success/failure
                    if not success:
                        # Increment reconnection counter if not already done
                        if not hasattr(self, '_reconnection_attempts'):
                            self._reconnection_attempts = 1
                        else:
                            self._reconnection_attempts += 1
                        
                        # Calculate appropriate delay based on number of attempts
                        self.reconnect_delay = min(10.0, 2.0 + (self._reconnection_attempts * 0.5))
                    else:
                        # Reset counters on successful connection
                        self._reconnection_attempts = 0
                        self.reconnect_delay = 2.0  # Reset to default
                        
                # Normal operation while connected
                while self.connection_alive and self.running:
                    self.read_serial()
                    time.sleep(0.01)  # Small sleep to prevent CPU hogging
                    
                # Handle error state
                if self.system_state == SystemState.ERROR:
                    # Wait until reconnection is requested or timeout expires
                    self.reconnect_event.wait(self.reconnect_delay)
                    self.reconnect_event.clear()
                    
            except Exception as e:
                logger.error(f"Serial thread error: {e}")
                self.error_signal.emit(f"Communication error: {e}")
                self.connection_alive = False
                self.set_system_state(SystemState.ERROR)
                time.sleep(2.0)  # Avoid fast reconnection attempts
                
    def set_system_state(self, new_state):
        """Thread-safe system state change with signal emission"""
        if self.system_state != new_state:
            self.system_state = new_state
            self.system_state_changed_signal.emit(new_state)
    
    # Replace the existing connect_serial method with this improved version
    def connect_serial(self):
        """Establish serial connection to Arduino with robust error handling and port discovery"""
        if self.ser is not None and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                logger.warning(f"Error closing previous connection: {e}")
        
        # Release resources from previous connection attempts
        self._release_com_port_resources()
        
        try:
            logger.info(f"Attempting to connect to {self.serial_port} at {self.baudrate} baud")
            self.set_system_state(SystemState.CONNECTING)
            
            # Track if we should try port discovery
            try_discovery = hasattr(self, '_enable_port_discovery') and self._enable_port_discovery
            
            # Start with configured port unless discovery is enabled
            ports_to_try = [self.serial_port]
            
            # If discovery is enabled or this is a reconnection attempt, discover all available ports
            if try_discovery:
                available_ports = self._discover_available_ports()
                if available_ports:
                    # Put the original port first in the list if it exists
                    if self.serial_port in available_ports:
                        available_ports.remove(self.serial_port)
                        ports_to_try = [self.serial_port] + available_ports
                    else:
                        ports_to_try = available_ports
            
            # Try each port in the list
            connection_success = False
            attempted_port = None
            
            for port_to_try in ports_to_try:
                logger.info(f"Attempting connection to port {port_to_try}")
                attempted_port = port_to_try
                
                try:
                    # Create a new serial object for each attempt
                    with self.serial_lock:
                        # Close previous connection if it exists
                        if self.ser is not None and self.ser.is_open:
                            self.ser.close()
                            self.ser = None
                        
                        # Create a new serial object
                        self.ser = serial.Serial(
                            port=None,  # Don't open immediately
                            baudrate=self.baudrate,
                            timeout=1,
                            write_timeout=1
                        )
                        self.ser.port = port_to_try
                        
                        # Try to open the port
                        self.ser.open()
                        
                        # If we get here, the port opened successfully
                        if self.ser.is_open:
                            # Set DTR after opening to properly reset Arduino
                            self.ser.setDTR(False)  # Set DTR low
                            time.sleep(0.1)         # Short delay
                            self.ser.setDTR(True)   # Set DTR high to reset Arduino
                            time.sleep(0.5)         # Allow Arduino time to boot
                            
                            # Clear buffers
                            self.ser.reset_input_buffer()
                            self.ser.reset_output_buffer()
                            
                            # Update serial port if it changed
                            if port_to_try != self.serial_port:
                                logger.info(f"Switching from port {self.serial_port} to {port_to_try}")
                                self.serial_port = port_to_try
                            
                            connection_success = True
                            break  # Exit the port loop since we found a working one
                
                except (serial.SerialException, OSError) as e:
                    logger.warning(f"Failed to connect to {port_to_try}: {e}")
                    # Continue to the next port
                    continue
            
            # After trying all ports, check if connection was successful
            if connection_success:
                logger.info(f"Successfully connected to {self.serial_port}")
                self.connection_alive = True
                self.set_system_state(SystemState.CONNECTED)
                self.connection_status_signal.emit(True, f"Connected to {self.serial_port}")
                self.last_data_received_time = time.time()
                
                # Reset reconnection attempt counter if we have one
                if hasattr(self, '_reconnection_attempts'):
                    self._reconnection_attempts = 0
                
                # Disable port discovery for next connection since we found a working port
                self._enable_port_discovery = False
                return True
            else:
                # All connection attempts failed
                error_msg = f"Failed to connect to any available port including {self.serial_port}"
                logger.error(error_msg)
                self.connection_status_signal.emit(False, error_msg)
                self.set_system_state(SystemState.ERROR)
                
                # Enable port discovery for next connection attempt
                self._enable_port_discovery = True
                
                # Track connection attempts
                if not hasattr(self, '_reconnection_attempts'):
                    self._reconnection_attempts = 1
                else:
                    self._reconnection_attempts += 1
                
                return False
                    
        except Exception as e:
            error_msg = f"Connection error: {e}"
            logger.error(error_msg)
            self.connection_status_signal.emit(False, error_msg)
            self.set_system_state(SystemState.ERROR)
            
            # Enable port discovery for next attempt
            self._enable_port_discovery = True
            return False
    
    # Replace the existing attempt_reconnection method with this improved version
    def attempt_reconnection(self):
        """Attempt to reconnect to Arduino after connection failure"""
        logger.info("Attempting to reconnect...")
        self.status_message = "Attempting to reconnect..."
        self.status_label.setText(self.status_message)
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #fff3cd; color: #856404;")
        
        # Close existing connection and release resources
        with self.serial_lock:
            if self.ser is not None:
                try:
                    if self.ser.is_open:
                        self.ser.close()
                    self.ser = None
                except Exception as e:
                    logger.warning(f"Error closing serial port during reconnect: {e}")
        
        # More aggressive cleanup if multiple reconnection attempts
        if hasattr(self, '_reconnection_attempts') and self._reconnection_attempts > 2:
            logger.info(f"Multiple reconnection attempts ({self._reconnection_attempts}), performing aggressive cleanup")
            self._release_com_port_resources()
            time.sleep(1.5)  # Longer delay for stubborn ports
        else:
            # Brief delay for first few reconnection attempts
            time.sleep(0.5)
        
        # Reset connection state and system state tracking
        self.connection_alive = False
        self.system_running = False
        
        # Enable port discovery for reconnection
        self._enable_port_discovery = True
        
        # Trigger reconnection in serial thread
        self.set_system_state(SystemState.RECONNECTING)
        self.reconnect_event.set()
        
    def check_connection_health(self):
        """Monitor connection health and attempt reconnection if needed"""
        # Skip check if system is initializing
        if self.system_state == SystemState.INITIALIZING or not hasattr(self, 'last_data_received_time'):
            return
            
        current_time = time.time()
        # Check if we've received data within the timeout period
        if self.connection_alive and (current_time - self.last_data_received_time) > self.communication_timeout:
            logger.warning(f"Communication timeout detected ({self.communication_timeout}s without data)")
            self.connection_alive = False
            self.set_system_state(SystemState.DISCONNECTED)
            self.connection_status_signal.emit(False, "Communication timeout - No data received")
            
            # Trigger reconnection attempt
            self.attempt_reconnection()   
            
    def _release_com_port_resources(self):
        """Attempt to forcibly release COM port resources"""
        try:
            # First ensure our serial object is properly closed
            if self.ser is not None:
                if self.ser.is_open:
                    try:
                        self.ser.close()
                    except Exception as e:
                        logger.warning(f"Error closing serial port: {e}")
                
                # Set to None to ensure proper garbage collection
                self.ser = None
            
            # On Windows, try to force COM port release
            if sys.platform.startswith('win'):
                import subprocess
                import gc
                
                # Force garbage collection to release any lingering resources
                gc.collect()
                time.sleep(0.2)  # Brief delay to let system process
                
                # Try to detect if the port is still in use by another process
                try:
                    # Check if the COM port still exists in system
                    if self.serial_port.startswith('COM'):
                        cmd = f'powershell -Command "[System.IO.Ports.SerialPort]::GetPortNames() -contains \'{self.serial_port}\'"'
                        result = subprocess.run(cmd, capture_output=True, text=True, shell=True)
                        
                        if "True" in result.stdout:
                            logger.info(f"{self.serial_port} exists in system, attempting resource release")
                            # Additional garbage collection and delay
                            gc.collect()
                            time.sleep(0.5)
                except Exception as e:
                    logger.warning(f"Error checking COM port status: {e}")
        except Exception as e:
            logger.error(f"Failed to release COM port resources: {e}")

    def _discover_available_ports(self):
        """Discover available COM ports that can be used for connection"""
        try:
            import serial.tools.list_ports
            ports = list(serial.tools.list_ports.comports())
            available_ports = []
            
            for port in ports:
                # Add detailed port information to logs to help with debugging
                logger.debug(f"Found port: {port.device}, desc: {port.description}, hwid: {port.hwid}")
                available_ports.append(port.device)
                
            logger.info(f"Discovered available ports: {available_ports}")
            return available_ports
        except Exception as e:
            logger.error(f"Error discovering available ports: {e}")
            return []
        
    # Add these new methods to the TemperatureControllerGUI class

    # Optimize the refresh_port_list method

    def refresh_port_list(self):
        """Refresh the list of available COM ports in the dropdown with progress indication"""
        try:
            # Clear the current list
            self.port_combo.clear()
            
            # Add a "Searching..." item while ports are being discovered
            self.port_combo.addItem("Searching for ports...")
            self.port_combo.setEnabled(False)
            
            # Force UI update to show "Searching..." immediately
            QApplication.processEvents()
            
            # Get available ports
            available_ports = self._discover_available_ports()
            
            # Clear the searching item
            self.port_combo.clear()
            
            if available_ports:
                # Add all available ports to the combo box
                self.port_combo.addItems(available_ports)
                
                # Select the current serial port if it's in the list
                if self.serial_port in available_ports:
                    index = self.port_combo.findText(self.serial_port)
                    if index >= 0:
                        self.port_combo.setCurrentIndex(index)
            else:
                self.port_combo.addItem("No ports found")
            
            # Re-enable the combo box
            self.port_combo.setEnabled(True)
            
            # Update status to indicate completion
            self.status_message = "Ready - Select a port and connect"
            self.status_label.setText(self.status_message)
            
        except Exception as e:
            logger.error(f"Error refreshing port list: {e}")
            self.port_combo.clear()
            self.port_combo.addItem("Error listing ports")
            self.port_combo.setEnabled(True)

    def connect_selected_port(self):
        """Connect to the selected COM port"""
        selected_port = self.port_combo.currentText()
        
        # Check if a valid port is selected
        if not selected_port or selected_port == "No ports found" or selected_port == "Error listing ports":
            QMessageBox.warning(self, "Port Selection Error", "Please select a valid COM port.")
            return
        
        # If we're already connected to this port, no need to reconnect
        if self.connection_alive and self.serial_port == selected_port:
            QMessageBox.information(self, "Already Connected", f"Already connected to {selected_port}.")
            return
        
        # Close existing connection if any
        if self.connection_alive:
            with self.serial_lock:
                if self.ser is not None and self.ser.is_open:
                    try:
                        self.ser.close()
                    except Exception as e:
                        logger.warning(f"Error closing previous connection: {e}")
            
            # Reset connection state
            self.connection_alive = False
            self._release_com_port_resources()
        
        # Update the serial port
        self.serial_port = selected_port
        logger.info(f"Manually selected port: {self.serial_port}")
        
        # Update status message
        self.status_message = f"Connecting to {selected_port}..."
        self.status_label.setText(self.status_message)
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #fff3cd; color: #856404;")
        
        # Disable port discovery for next connection attempt since user manually selected a port
        self._enable_port_discovery = False
        
        # Attempt to connect
        self.set_system_state(SystemState.RECONNECTING)
        self.reconnect_event.set()
    
    def initUI(self):
        """Initialize the User Interface"""
        # (Same as your original initUI implementation)
        # Entire UI setup code here...
        self.setWindowTitle('Temperature Controller')
        self.setGeometry(100, 100, 1200, 700)
        self.setStyleSheet("""
            QMainWindow {
                background-color: #f0f0f0;
            }
            QGroupBox {
                font-weight: bold;
                border: 1px solid #cccccc;
                border-radius: 6px;
                margin-top: 12px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
            }
            /* System Information specific styling */
            QGroupBox#systemInfoPanel {
                font-size: 16px;  /* Larger title font */
            }
            QGroupBox#systemInfoPanel::title {
                font-size: 16px;  /* Larger title font */
            }
            QPushButton {
                border-radius: 4px;
                padding: 6px;
                font-weight: bold;
            }
            QPushButton#startButton {
                background-color: #4caf50;
                color: white;
            }
            QPushButton#startButton:hover {
                background-color: #45a049;
            }
            QPushButton#stopButton {
                background-color: #f44336;
                color: white;
            }
            QPushButton#stopButton:hover {
                background-color: #d32f2f;
            }
            QLabel#statusLabel {
                font-weight: bold;
                color: #2c3e50;
                padding: 8px;
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                background-color: #ecf0f1;
            }
            QLabel#infoLabel {
                background-color: #ffffff;
                border: 1px solid #dddddd;
                border-radius: 4px;
                padding: 12px;
                font-size: 16px;  /* Increased from 14px to 16px */
                line-height: 1.8;  /* Increased from 1.6 to 1.8 */
            }
        """)
        
        # Create central widget and layout
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # Top section with graph and info panel
        top_layout = QHBoxLayout()
        
        # First create the info panel (before trying to use it)
        info_panel = QGroupBox("System Information")
        info_panel.setObjectName("systemInfoPanel")
        info_layout = QVBoxLayout(info_panel)
        self.info_label = QLabel()
        self.info_label.setObjectName("infoLabel")
        self.info_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.info_label.setWordWrap(True)
        self.info_label.setMinimumWidth(250)
        info_panel.setMinimumHeight(250)  # Slightly decrease minimum height
        self.update_info_text()  # Initialize with default values
        info_layout.addWidget(self.info_label)
        
        # COM Port Selection Panel beneath info panel
        port_selection_panel = QGroupBox("COM Port Selection")
        port_selection_panel.setObjectName("portSelectionPanel")
        port_selection_panel.setStyleSheet("QGroupBox { font-size: 12pt; font-weight: bold; }")
        port_selection_layout = QVBoxLayout(port_selection_panel)
        
        # Horizontal layout for port selection controls
        port_controls_layout = QHBoxLayout()
        
        # Create a vertical layout to stack info panel and port selection
        right_side_layout = QVBoxLayout()
        right_side_layout.addWidget(info_panel, 4)  # Info panel takes 4 parts
        right_side_layout.addWidget(port_selection_panel, 1)  # Port selection takes 1 part
        
        # Create ComboBox for port selection
        self.port_combo = QComboBox()
        self.port_combo.setStyleSheet("font-size: 12pt;")
        self.port_combo.setMinimumWidth(150)
        port_controls_layout.addWidget(self.port_combo, 3)
        
        # Create refresh button for port list
        refresh_ports_button = QPushButton("Refresh")
        refresh_ports_button.setStyleSheet("font-size: 11pt;")
        refresh_ports_button.clicked.connect(self.refresh_port_list)
        port_controls_layout.addWidget(refresh_ports_button, 1)
        
        # Create connect button
        connect_port_button = QPushButton("Connect")
        connect_port_button.setStyleSheet("font-size: 11pt; background-color: #4caf50; color: white;")
        connect_port_button.clicked.connect(self.connect_selected_port)
        port_controls_layout.addWidget(connect_port_button, 1)
        
        # Add horizontal layout to panel
        port_selection_layout.addLayout(port_controls_layout)
        
        # Add helper text with increased size and custom styling
        port_help_label = QLabel("Select the COM port connected to your Heating Unit")
        port_help_label.setStyleSheet("font-size: 11pt; font-weight: bold; color: #2c3e50;")
        port_selection_layout.addWidget(port_help_label)
        
        
        # Graph Widget (using pyqtgraph)
        self.graph_widget = pg.PlotWidget()
        self.graph_widget.setBackground('w')  # white background
        self.graph_widget.setTitle("Real-time Temperature Monitoring", color="#333333", size="14pt",bold=True)
        self.graph_widget.setLabel('left', 'Temperature (°C)', color="#333333", font="bold 11pt")
        self.graph_widget.setLabel('bottom', 'Time (s)', color="#333333", font="bold 11pt")
        self.graph_widget.showGrid(x=True, y=True, alpha=0.3)
        self.graph_widget.setYRange(0, 220)
        
        # Create plot lines
        # self.temp_line = self.graph_widget.plot([], [], pen=pg.mkPen(color='b', width=3), name='Actual Temp')
                # Create a gradient pen for a more professional look
        blue_pen = pg.mkPen({
            'color': (0, 0, 180),
            'width': 3, 
            'cosmetic': True
        })
        self.setpoint_line = self.graph_widget.plot([], [], pen=pg.mkPen(color='r', width=3, style=Qt.DashLine), name='Setpoint')
        self.overshoot_line = pg.InfiniteLine(angle=0, movable=False, pen=pg.mkPen(color='g', width=1, style=Qt.DotLine))
        self.overshoot_line.setVisible(False)
        self.graph_widget.addItem(self.overshoot_line)
        # Create plot lines with anti-aliasing
        self.temp_line = self.graph_widget.plot([], [], pen=blue_pen, name='Actual Temp', antialias=True)
        
        
        top_layout.addWidget(self.graph_widget, 8)
        top_layout.addLayout(right_side_layout, 2)  # Add the right side layout with both panels
        
        # Control panel section
        control_panel = QWidget()
        control_layout = QHBoxLayout(control_panel)
        
        # Start/Stop buttons
        button_group = QGroupBox("System Control")
        button_group.setStyleSheet("QGroupBox { font-size: 13pt; font-weight: bold; }")
        button_layout = QVBoxLayout(button_group)
        
        self.start_button = QPushButton("START SYSTEM")
        self.start_button.setObjectName("startButton")
        self.start_button.setEnabled(False)  # Disabled until initialization complete
        self.start_button.clicked.connect(self.start_system)
        self.start_button.setMinimumHeight(50)
        self.start_button.setStyleSheet("font-size: 12pt; font-weight: bold;")
        
        self.stop_button = QPushButton("STOP SYSTEM")
        self.stop_button.setObjectName("stopButton")
        self.stop_button.setEnabled(False)  # Disabled until initialization complete
        self.stop_button.clicked.connect(self.stop_system)
        self.stop_button.setMinimumHeight(50)
        self.stop_button.setStyleSheet("font-size: 12pt; font-weight: bold;")
        
        # Add reconnect button
        self.reconnect_button = QPushButton("RECONNECT")
        self.reconnect_button.setEnabled(False)
        self.reconnect_button.clicked.connect(self.attempt_reconnection)
        self.reconnect_button.setMinimumHeight(50)
        self.reconnect_button.setStyleSheet("font-size: 12pt; font-weight: bold; background-color: #3498db; color: white;")
        
        button_layout.addWidget(self.start_button)
        button_layout.addWidget(self.stop_button)
        button_layout.addWidget(self.reconnect_button)
        
        # Temperature and Fan controls
        temp_control_group = QGroupBox("Temperature and  Fan Controls")
        temp_control_group.setStyleSheet("QGroupBox { font-size: 13pt; font-weight: bold; }")
        temp_control_layout = QGridLayout(temp_control_group)
        
        # Increased font size for Target Temperature label
        target_temp_label = QLabel("Target Temperature (°C):")
        target_temp_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        temp_control_layout.addWidget(target_temp_label, 0, 0)
        self.setpoint_input = QLineEdit(str(self.current_setpoint))
        self.setpoint_input.setMaximumWidth(120)  # Increased from 100
        self.setpoint_input.setStyleSheet("font-size: 13pt;")
        self.setpoint_input.returnPressed.connect(self.update_setpoint)
        temp_control_layout.addWidget(self.setpoint_input, 0, 1)
        
        # Increased font size for Fan Speed label
        fan_speed_label = QLabel("Fan Speed (%):")
        fan_speed_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        temp_control_layout.addWidget(fan_speed_label, 1, 0)
        self.fan_speed_input = QLineEdit(str(self.current_fan_speed))
        self.fan_speed_input.setMaximumWidth(120)  # Increased from 100
        self.fan_speed_input.setStyleSheet("font-size: 13pt;")
        self.fan_speed_input.returnPressed.connect(self.update_fan_speed)
        temp_control_layout.addWidget(self.fan_speed_input, 1, 1)
        
        # Add a slider for fan speed with discrete steps (30, 40, 50, 60, 70, 80, 90, 100)
        self.fan_slider = QSlider(Qt.Horizontal)
        self.fan_slider.setMinimum(30)
        self.fan_slider.setMaximum(100)
        self.fan_slider.setSingleStep(10)  # Set step size to 10
        self.fan_slider.setPageStep(10)    # Set page step to 10
        self.fan_slider.setTickPosition(QSlider.TicksBelow)  # Show ticks below slider
        self.fan_slider.setTickInterval(10)  # Place ticks every 10 units
        # Normalize initial value to nearest discrete step
        normalized_value = self.normalize_fan_speed(self.current_fan_speed)
        self.fan_slider.setValue(normalized_value)
        self.fan_slider.valueChanged.connect(self.slider_fan_update)
        temp_control_layout.addWidget(self.fan_slider, 1, 2)
        
        # Feedback source and protection threshold
        sensor_group = QGroupBox("Sensor and  Protection Settings")
        sensor_group.setStyleSheet("QGroupBox { font-size: 13pt; font-weight: bold; }")
        sensor_layout = QGridLayout(sensor_group)
        
        # Increased font size for Feedback Source label
        feedback_label = QLabel("Feedback Source:")
        feedback_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        sensor_layout.addWidget(feedback_label, 0, 0)
        self.feedback_combo = QComboBox()
        self.feedback_combo.addItems(["PT1000", "K-Type Thermocouple"])
        self.feedback_combo.setStyleSheet("font-size: 13pt;")
        self.feedback_combo.currentTextChanged.connect(self.update_feedback_source)
        sensor_layout.addWidget(self.feedback_combo, 0, 1)
        
        # Increased font size for Protection Threshold label
        threshold_label = QLabel("Protection Threshold (°C):")
        threshold_label.setStyleSheet("font-size: 14pt; font-weight: bold;")
        sensor_layout.addWidget(threshold_label, 1, 0)
        self.threshold_input = QLineEdit(str(self.temp_threshold))
        self.threshold_input.setMaximumWidth(120)  # Increased from 100
        self.threshold_input.setStyleSheet("font-size: 13pt;")
        self.threshold_input.returnPressed.connect(self.update_threshold)
        sensor_layout.addWidget(self.threshold_input, 1, 1)
        
        # Add all control sections to control layout
        control_layout.addWidget(button_group, 1)
        control_layout.addWidget(temp_control_group, 2)
        control_layout.addWidget(sensor_group, 2)
        
        # Status bar at bottom
        self.status_label = QLabel(self.status_message)
        self.status_label.setObjectName("statusLabel")
        self.status_label.setAlignment(Qt.AlignCenter)
        self.status_label.setMinimumHeight(40)
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; padding: 8px; border: 1px solid #bdc3c7; border-radius: 4px; background-color: #ecf0f1; color: #2c3e50;")
        
        # Add all sections to main layout
        main_layout.addLayout(top_layout, 7)
        main_layout.addWidget(control_panel, 2)
        main_layout.addWidget(self.status_label, 1)
        
        # Populate port list initially
        self.refresh_port_list()
        
        # We don't send the initial feedback source selection here anymore
        # It will be sent once connection is established
    
    # Modify the on_connection_status_change method

    def on_connection_status_change(self, is_connected, message):
        """Handle connection status changes"""
        if is_connected:
            # Only update UI if there's no active protection event
            if not self.protection_active:
                self.status_message = message
                self.status_label.setText(self.status_message)
                self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
            self.reconnect_button.setEnabled(False)
            
            # Update port dropdown to show the connected port
            if hasattr(self, 'port_combo'):
                index = self.port_combo.findText(self.serial_port)
                if index >= 0:
                    self.port_combo.setCurrentIndex(index)
            
            # Send initial feedback source selection to Arduino only during first connection
            # not when reconnecting or receiving connection status updates
            if not self.initialization_complete:
                QTimer.singleShot(500, lambda: self.send_command("B1"))  # Default to PT1000
        else:
            # Disconnection is considered a critical event, so override protection messages
            self.protection_active = False  # Reset protection state on disconnect
            self.current_protection_message = ""
            
            self.status_message = message
            self.status_label.setText(self.status_message)
            self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #f8d7da; color: #721c24;")
            self.reconnect_button.setEnabled(True)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
            
    def update_ui_from_data(self, params):
        """Thread-safe UI update from signal"""
        # Skip UI updates if app is shutting down
        if hasattr(self, 'ui_shutting_down') and self.ui_shutting_down:
            return
            
        try:
            # Check if widgets still exist
            if not hasattr(self, 'info_label') or not self.info_label:
                logger.warning("UI elements not available for update")
                return
                
            # Update status message first, but only if no protection event is active
            current_temp = self.temp_data[-1] if self.temp_data else 0
            if not self.protection_active:
                self._update_status_message(current_temp)
            else:
                # If protection is active, ensure protection message remains visible
                self.status_label.setText(self.current_protection_message)
            
            # Update info panel
            self.update_info_text()
            
            # Update fan speed UI
            if hasattr(self, 'fan_speed_input'):
                self.fan_speed_input.setText(str(self.current_fan_speed))
            
            if hasattr(self, 'fan_slider'):
                self.fan_slider.setValue(self.current_fan_speed)
                
            # Update feedback source UI
            if 'FB' in params and hasattr(self, 'feedback_combo'):
                self._ignore_feedback_source_change = True
                index = 0 if self.feedback_source == "PT1000" else 1
                self.feedback_combo.setCurrentIndex(index)
                self._ignore_feedback_source_change = False
                
            # Update overshoot line in graph
            if self.max_overshoot > 0 and hasattr(self, 'overshoot_line'):
                self.overshoot_line.setValue(self.current_setpoint + self.max_overshoot)
                self.overshoot_line.setVisible(True)
                
        except Exception as e:
            logger.error(f"Error updating UI from data: {e}")        
    
    def on_error(self, error_message):
        """Handle error messages"""
        logger.error(error_message)
        
        # System errors are critical, they should override protection messages
        self.protection_active = True
        self.current_protection_message = error_message
        
        self.status_message = error_message
        self.status_label.setText(self.status_message)
        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #f8d7da; color: #721c24;")
        
    def on_system_state_change(self, new_state):
        """Handle system state changes"""
        logger.info(f"System state changed to {new_state.name}")
        
        # Update UI elements based on state
        if new_state == SystemState.CONNECTED:
            self.reconnect_button.setEnabled(False)
        elif new_state in [SystemState.DISCONNECTED, SystemState.ERROR]:
            self.reconnect_button.setEnabled(True)
            self.start_button.setEnabled(False)
            self.stop_button.setEnabled(False)
    
    def update_plot(self):
        """Update the temperature plot with current data"""
        if len(self.time_data) > 0:
            # Convert deques to numpy arrays for plotting
            times = np.array(self.time_data)
            temps = np.array(self.temp_data)
            setpoints = np.array(self.setpoint_data)
            
            # Update the plot lines
            self.temp_line.setData(times, temps)
            self.setpoint_line.setData(times, setpoints)
            
            # Auto-scroll the x-axis to show the last 60 seconds of data
            if len(times) > 1:
                view_min = max(0, times[-1] - 60)
                view_max = times[-1] + 5
                self.graph_widget.setXRange(view_min, view_max)
            
            # Update overshoot line if needed
            if self.max_overshoot > 0:
                self.overshoot_line.setValue(self.current_setpoint + self.max_overshoot)
                self.overshoot_line.setVisible(True)
    
    def read_serial(self):
        """Read serial data from Arduino with improved error handling"""
        try:
            with self.serial_lock:
                if not self.ser or not self.ser.is_open:
                    return
                    
                if self.ser.in_waiting:
                    line = self.ser.readline().decode('utf-8', errors='replace').strip()
                    if line:  # Only process non-empty lines
                        self.process_serial_data(line)
                        self.last_data_received_time = time.time()  # Update the timestamp
                    
        except serial.SerialException as e:
            logger.error(f"Serial read error: {e}")
            self.connection_alive = False
            self.set_system_state(SystemState.DISCONNECTED)
            
        except UnicodeDecodeError as e:
            logger.warning(f"Unicode decode error (non-critical): {e}")
            # This is non-critical, just ignore the current line
            
        except Exception as e:
            logger.error(f"Unexpected error during serial read: {e}")
            self.error_signal.emit(f"Communication error: {e}")
    
    def process_serial_data(self, line):
        """Process incoming serial data from Arduino with improved error handling"""
        try:
            if line.startswith("DATA:"):
                self.process_status_data(line[5:])
                
            elif line.startswith("ACK:"):
                logger.debug(f"Command acknowledged: {line[4:]}")
                # Check if this is the feedback source acknowledgment message
                if line.startswith("ACK:FEEDBACK_SOURCE="):
                    self.feedback_source = line[21:]
                    # Mark initialization as complete so start button can be enabled
                    self.initialization_complete = True
                    # Update the UI to show initialization is complete
                    self.start_button.setEnabled(True)
                    if not self.protection_active:  # Only update status if no protection active
                        self.status_message = "System ready - Press START button"
                        self.status_label.setText(self.status_message)
                # Update fan input if fan command was acknowledged
                elif line.startswith("ACK:FAN="):
                    try:
                        fan_value = int(line[8:])
                        # Only update internal state and UI, don't send commands
                        self.current_fan_speed = fan_value
                        
                        # Block UI signals while updating to prevent feedback loops
                        old_state = self.fan_speed_input.blockSignals(True)
                        self.fan_speed_input.setText(str(fan_value))
                        self.fan_speed_input.blockSignals(old_state)
                        
                        old_state = self.fan_slider.blockSignals(True)
                        self.fan_slider.setValue(fan_value)
                        self.fan_slider.blockSignals(old_state)
                    except ValueError:
                        pass  # Ignore invalid fan value
                # System started acknowledgment
                elif line.startswith("ACK:SYSTEM_STARTED"):
                    self.system_running = True
                    self.set_system_state(SystemState.RUNNING)
                    if not self.protection_active:  # Only update status if no protection active
                        self.status_message = "System RUNNING"
                        self.status_label.setText(self.status_message)
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
                    self.start_button.setEnabled(False)
                    self.stop_button.setEnabled(True)
                # System stopped acknowledgment    
                elif line.startswith("ACK:SYSTEM_STOPPED"):
                    self.system_running = False
                    self.set_system_state(SystemState.STOPPED)
                    if not self.protection_active:  # Only update status if no protection active
                        self.status_message = "System STOPPED"
                        self.status_label.setText(self.status_message)
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ecf0f1; color: #2c3e50;")
                    self.start_button.setEnabled(True)
                    self.stop_button.setEnabled(False)
                    
            elif line.startswith("ERR:"):
                logger.warning(f"Error from Arduino: {line[4:]}")
                
                # Set protection active flag for errors
                self.protection_active = True
                
                if line.startswith("ERR:THERMOCOUPLE_NOT_FOUND"):
                    self.current_protection_message = "ERROR: Thermocouple not found!"
                elif line.startswith("ERR:INITIALIZATION_NOT_COMPLETE"):
                    self.current_protection_message = "ERROR: System initialization not complete"
                elif "OVERTEMP" in line:
                    # Standardize overtemp error messages
                    self.current_protection_message = "ALARM: Over-Tempereature Protection Activated!"
                elif line.startswith("ERR:SYSTEM_NOT_RUNNING"):
                    # Handle state mismatch by updating GUI state to match Arduino
                    logger.info("State mismatch detected: Arduino reports system not running")
                    if self.system_running:
                        # Update our system state to match Arduino reality
                        self.system_running = False
                        self.set_system_state(SystemState.STOPPED)
                        self.current_protection_message = "System STOPPED (Reset after reconnection)"
                        
                        # Don't change the protection state for state mismatch
                        self.protection_active = False
                        self.status_message = self.current_protection_message
                        self.status_label.setText(self.status_message)
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ecf0f1; color: #2c3e50;")
                        self.start_button.setEnabled(True)
                        self.stop_button.setEnabled(False)
                        # Return early for this special case
                        return
                else:
                    self.current_protection_message = f"ERROR: {line[4:]}"
                
                # Update status message and UI with the error message
                self.status_message = self.current_protection_message
                self.status_label.setText(self.current_protection_message)
                self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #f8d7da; color: #721c24;")
                
            elif line.startswith("ALARM:"):
                logger.warning(f"Alarm from Arduino: {line[6:]}")
                
                # Set protection active flag for alarms
                self.protection_active = True
                
                # Standardize all overtemp alarm messages to a simple format
                if "OVERTEMP" in line:
                    self.current_protection_message = "ALARM: Over-Tempereature Protection Activated!"
                elif "UNDERTEMP" in line:
                    self.current_protection_message = "ALARM: Under-Temperature Protection Activated!"
                elif "HOTGUN_OVERTEMP" in line:
                    self.current_protection_message = "ALARM: HotGun Over-Temperature Protection Activated!"
                else:
                    self.current_protection_message = f"ALARM: {line[6:]}"
                    
                # Update status display with alarm message
                self.status_message = self.current_protection_message
                self.status_label.setText(self.current_protection_message)
                # Make status label red for alarms
                self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ffeeee; color: #cc0000;")
                
            elif line.startswith("WARNING:"):
                logger.warning(f"Warning from Arduino: {line[8:]}")
                
                # Set protection active flag for warnings
                self.protection_active = True
                
                # Standardize all overtemp warning messages to a simple format
                if "OVERTEMP" in line:
                    self.current_protection_message = "ALARM: Over-Tempereature Protection Activated!"
                else:
                    # For other warnings, keep a generic format
                    self.current_protection_message = f"WARNING: {line[8:]}"
                    
                # Update status display with warning message
                self.status_message = self.current_protection_message
                self.status_label.setText(self.current_protection_message)
                # Orange/yellow styling for warnings
                self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #fff3cd; color: #856404;")
                
            elif line.startswith("SYSTEM:"):
                if line.startswith("SYSTEM:READY"):
                    logger.info("Arduino system reports ready state")
                    # Don't automatically send B1 command again here
                    # Only send feedback source during initial connection
                    
                elif line.startswith("SYSTEM:RESET"):
                    # Arduino has reset - update our state to match
                    logger.info("Arduino system reports reset state")
                    self.system_running = False
                    self.set_system_state(SystemState.STOPPED)
                    self.protection_active = False  # Clear protection state on reset
                    self.current_protection_message = ""
                    self.status_message = "System RESET - Ready to start"
                    self.status_label.setText(self.status_message)
                    self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ecf0f1; color: #2c3e50;")
                    self.start_button.setEnabled(True)
                    self.stop_button.setEnabled(False)
                    
                elif line.startswith("SYSTEM:STOPPED_PROTECTION"):
                    logger.info("System stopped due to protection mechanism")
                    self.system_running = False
                    self.set_system_state(SystemState.STOPPED)
                    # Don't clear protection flag yet - keep showing the alarm
                    # Make sure we're still showing the protection message
                    if self.current_protection_message:
                        self.status_label.setText(self.current_protection_message)
                    self.start_button.setEnabled(True)
                    self.stop_button.setEnabled(False)
                    
                elif line.startswith("SYSTEM:PROTECTION_CLEARED"):
                    logger.info("Protection condition cleared, system can be restarted")
                    # Reset protection flags when explicitly cleared
                    self.protection_active = False
                    self.current_protection_message = ""
                    
                    self.status_message = "Protection cleared - Press START to resume"
                    self.status_label.setText(self.status_message)
                    self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
                    self.start_button.setEnabled(True)
                    self.stop_button.setEnabled(False)
                    
            elif line.startswith("STATUS:"):
                # Only update the status message if no protection is active
                if not self.protection_active:
                    self.status_message = line[7:]  # Remove STATUS: prefix
                    self.status_label.setText(self.status_message)
                    # Style remains unchanged unless explicitly set
                else:
                    # If protection is active, log the status message but don't show it
                    logger.debug(f"Ignored status message during protection: {line[7:]}")
                    
            elif line.startswith("DEBUG:"):
                # Handle debug messages - these don't affect the UI status display
                logger.debug(f"Arduino debug: {line[6:]}")
                
                # Special handling for protection condition cleared debug message
                if "OVERTEMP_CONDITION_CLEARED" in line or "PROTECTION_CLEARED" in line:
                    logger.info("Protection condition cleared via debug message")
                    self.protection_active = False
                    self.current_protection_message = ""
                    # Only update the UI if the debug message indicates protection was cleared
                    if not self.system_running:
                        self.status_message = "Protection cleared - Press START to resume"
                        self.status_label.setText(self.status_message)
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
                    
        except Exception as e:
            logger.error(f"Data processing error: {e}")
            # Don't propagate this error further, just log it
    
    def process_status_data(self, data):
        """Process status data received from Arduino with improved error handling"""
        try:
            # Use safer regex pattern matching that's more tolerant of malformed data
            params = {}
            matches = re.findall(r'([A-Z]+)=([^,]+)', data)
            for key, value in matches:
                params[key] = value
                
            # Extract current temperature from data first
            try:
                current_temp = float(params.get('CT', 0))  # Get current temperature with default 0
            except (ValueError, TypeError):
                logger.warning(f"Invalid temperature value in data: {params.get('CT')}")
                current_temp = 0  # Set a safe default
                    
            # Update current values with safe defaults
            try:
                # Get the setpoint value from data
                new_setpoint = float(params.get('ST', self.current_setpoint))
                # Apply the new setpoint limit range (0 to 180)
                if 0 <= new_setpoint <= 180:
                    self.current_setpoint = new_setpoint
                else:
                    logger.warning(f"Received setpoint {new_setpoint} outside valid range (0-180°C)")
            except (ValueError, TypeError):
                logger.warning(f"Invalid setpoint value in data: {params.get('ST')}")
                
            # Extract hotgun temperature (TT)
            try:
                self.hotgun_temp = float(params.get('TT', 0))
            except (ValueError, TypeError):
                logger.warning(f"Invalid hotgun temperature value in data: {params.get('TT')}")    
                
            # Update feedback source if provided
            if 'FB' in params:
                self.feedback_source = params['FB']
                # Update the UI dropdown to match but prevent sending commands
                self._ignore_feedback_source_change = True
                index = 0 if self.feedback_source == "PT1000" else 1
                self.feedback_combo.setCurrentIndex(index)
                self._ignore_feedback_source_change = False
                
            # Update PID values
            try:
                if 'KP' in params:
                    self.kp = float(params['KP'])
                if 'KI' in params:
                    self.ki = float(params['KI'])
                if 'KD' in params:
                    self.kd = float(params['KD'])
            except (ValueError, TypeError):
                logger.warning(f"Invalid PID value in data")
                
            # Update data series
            current_time = time.time() - self.start_time
            self.time_data.append(current_time)
            self.temp_data.append(current_temp)
            self.setpoint_data.append(self.current_setpoint)
            
            # Update raw temperature data for possible smoothing
            self.raw_temp_data.append(current_temp)
            
            # Update overshoot tracking
            try:
                self.max_overshoot = float(params.get('OV', self.max_overshoot))
                if self.max_overshoot > 0:
                    self.overshoot_line.setValue(self.current_setpoint + self.max_overshoot)
                    self.overshoot_line.setVisible(True)
            except (ValueError, TypeError):
                logger.warning(f"Invalid overshoot value in data: {params.get('OV')}")
                
            # Update settling time
            try:
                if 'STL' in params:
                    self.settling_time = float(params['STL'])
            except (ValueError, TypeError):
                logger.warning(f"Invalid settling time value in data: {params.get('STL')}")
                
            # Update fan speed
            try:
                self.current_fan_speed = int(params.get('FS', self.current_fan_speed))
                self.fan_speed_input.setText(str(self.current_fan_speed))
                self.fan_slider.setValue(self.current_fan_speed)
            except (ValueError, TypeError):
                logger.warning(f"Invalid fan speed value in data: {params.get('FS')}")
                
            # Update status message only if no protection event is active
            if not self.protection_active:
                self._update_status_message(current_temp)
            else:
                # If protection is active, ensure the protection message remains visible
                self.status_label.setText(self.current_protection_message)
            
            # Update info display
            self.update_info_text()
            
            # Apply smoothing only for thermocouple data
            if self.feedback_source == "THERMOCOUPLE" and len(self.raw_temp_data) >= 7:
                try:
                    # Convert deque to list for SavGol filter
                    temp_list = list(self.raw_temp_data)
                    if len(temp_list) >= 7:  # Minimum length for savgol with these parameters
                        # Apply Savitzky-Golay filter for visual smoothing
                        smoothed_temps = savgol_filter(temp_list, 18, 1)
                        current_temp = smoothed_temps[-1]  # Use the last smoothed value
                except Exception as e:
                    logger.warning(f"Error applying smoothing filter: {e}")
                    
        except Exception as e:
            logger.error(f"Error processing status data: {e}")
            # Don't propagate this error up further
            
    def _update_status_message(self, current_temp):
        """Update status message based on system state with feedback-specific tolerance"""
        # Skip updating status if protection is active
        if self.protection_active:
            return
            
        # Original status update logic
        if not self.initialization_complete:
            self.status_message = "System initializing - Please wait"
            self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ecf0f1; color: #2c3e50;")
        elif not self.system_running:
            self.status_message = "System STOPPED - Press Start"
            self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #ecf0f1; color: #2c3e50;")
        else:
            # System is running - check temperature stability with feedback-specific tolerance
            try:
                temp_diff = abs(current_temp - self.current_setpoint)
                
                # Apply different tolerance based on feedback source
                if self.feedback_source == "PT1000":
                    # Standard tolerance for PT1000 sensor
                    tolerance = 5  # ±5 degrees
                elif self.feedback_source == "THERMOCOUPLE":
                    # Wider tolerance for thermocouple sensor
                    tolerance = 10  # ±10 degrees
                else:
                    # Default to standard tolerance if feedback source is unknown
                    tolerance = 5
                
                # Check if temperature is within the appropriate tolerance
                is_within_tolerance = temp_diff <= tolerance
                
                if is_within_tolerance:
                    # Temperature is within tolerance for the current feedback source
                    if self.status_message != "System RUNNING - Ready for measurement":
                        self.status_message = "System RUNNING - Ready for measurement"
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
                        self.status_label.setText(self.status_message)
                else:
                    # Temperature out of tolerance for the current feedback source
                    if self.status_message != "Please wait till the temperature settles":
                        self.status_message = "Please wait till the temperature settles"
                        self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #fff3cd; color: #856404;")
                        self.status_label.setText(self.status_message)
                    
            except Exception as e:
                logger.warning(f"Error updating temperature status: {e}")
                self.status_message = "System RUNNING"
                self.status_label.setStyleSheet("font-size: 14pt; font-weight: bold; background-color: #d4edda; color: #155724;")
                self.status_label.setText(self.status_message)
        
    def update_info_text(self):
        """Update the information panel with current system values"""
        # Skip UI updates if app is shutting down
        if hasattr(self, 'ui_shutting_down') and self.ui_shutting_down:
            return
            
        try:
            # First check if UI elements exist
            if not hasattr(self, 'info_label') or self.info_label is None:
                logger.warning("Info label not available for update")
                return
                
            # Define a style for parameter values (larger, bold)
            value_style = "style='font-size: 16px; font-weight: bold;'"
            # Define style for labels (blue color)
            label_style = "style='color: #1a5fb4; font-weight: bold;'"
            
            # Add connection status to info panel
            connection_status = "CONNECTED" if self.connection_alive else "DISCONNECTED"
            connection_style = "color: green; font-weight: bold;" if self.connection_alive else "color: red; font-weight: bold;"
            
            info = [
                f"<span {label_style}>Connection:</span> <span style='{connection_style}'>{connection_status}</span>",
                f"<span {label_style}>Setpoint (Max 180.0°C):</span> <span {value_style}>{self.current_setpoint:.1f}°C</span>",
                f"<span {label_style}>Actual Temp on DUT:</span> <span {value_style}>{self.temp_data[-1] if self.temp_data else 0:.1f}°C</span>",
                f"<span {label_style}>Hotgun Temp:</span> <span {value_style}>{self.hotgun_temp:.1f}°C</span>",  
                f"<span {label_style}>Feedback Source:</span> <span {value_style}>{self.feedback_source}</span>",
                f"<span {label_style}>Max Overshoot:</span> <span {value_style}>{self.max_overshoot:.1f}°C</span>",
                f"<span {label_style}>Protection Threshold:</span> <span {value_style}>{self.temp_threshold:.1f}°C</span>",
                f"<span {label_style}>Fan Speed:</span> <span {value_style}>{self.current_fan_speed}%</span>",
                f"<span {label_style}>PID Values:</span> <span {value_style}>Kp={self.kp:.1f} Ki={self.ki:.1f} Kd={self.kd:.1f}</span>",
                f"<span {label_style}>Status:</span> <span {value_style}>{'RUNNING' if self.system_running else 'STOPPED'}</span>",
                f"<span {label_style}>Initialization:</span> <span {value_style}>{'COMPLETE' if self.initialization_complete else 'IN PROGRESS'}</span>"
            ]
            # Join with <br><br> for increased line spacing
            self.info_label.setText('<br><br>'.join(info))
        except Exception as e:
            logger.error(f"Error updating info text: {e}")
    
    def send_command(self, command):
        """Send a command to the Arduino controller with robust error handling"""
        if not self.connection_alive:
            logger.warning(f"Cannot send command: {command} - not connected")
            return False
            
        try:
            with self.serial_lock:
                if self.ser and self.ser.is_open:
                    logger.debug(f"Sending command: {command}")
                    self.ser.write((command + '\n').encode('utf-8'))
                    return True
                else:
                    logger.warning("Serial port not open, cannot send command")
                    return False
        except serial.SerialTimeoutException:
            logger.error(f"Write timeout when sending command: {command}")
            self.error_signal.emit("Command failed: write timeout")
            return False
        except serial.SerialException as e:
            logger.error(f"Serial error when sending command: {e}")
            self.error_signal.emit(f"Command failed: {e}")
            self.connection_alive = False
            self.set_system_state(SystemState.ERROR)
            return False
        except Exception as e:
            logger.error(f"Unexpected error when sending command: {e}")
            self.error_signal.emit(f"Command failed: {e}")
            return False
    
    def update_setpoint(self):
        """Handle setpoint changes from UI with validation against max limit and protection threshold."""
        try:
            sp = float(self.setpoint_input.text())
            
            # First check against maximum allowed setpoint of 180°C
            if sp > self.max_setpoint:
                QMessageBox.warning(self, "Invalid Setpoint", 
                                f"Setpoint cannot exceed {self.max_setpoint}°C.")
                sp = self.max_setpoint  # Adjust setpoint to maximum allowed value
                self.setpoint_input.setText(str(sp))
            # Then check against protection threshold
            elif sp > self.temp_threshold:
                QMessageBox.warning(self, "Invalid Setpoint", 
                                "Setpoint cannot exceed the protection threshold.")
                sp = self.temp_threshold  # Adjust setpoint to match the threshold
                self.setpoint_input.setText(str(sp))
            
            # Send the validated setpoint to the Arduino
            success = self.send_command(f"S{sp}")
            if not success:
                self.setpoint_input.setText(str(self.current_setpoint))
        except ValueError:
            QMessageBox.warning(self, "Invalid Value", "Setpoint must be a number.")
            self.setpoint_input.setText(str(self.current_setpoint))
    
    def update_feedback_source(self, source):
        """Handle feedback source selection changes"""
        # Use a flag to prevent command feedback loops
        if hasattr(self, '_ignore_feedback_source_change') and self._ignore_feedback_source_change:
            return
        
        # Store the last sent feedback command time to prevent rapid changes
        current_time = time.time()
        last_sent_time = getattr(self, '_last_feedback_command_time', 0)
        
        # Only send if more than 2 seconds since last command or if it's the first command
        if current_time - last_sent_time > 2.0 or not hasattr(self, '_last_feedback_command_time'):
            if source == 'PT1000':
                self.send_command("B1")
            elif source == 'K-Type Thermocouple':
                self.send_command("B2")
        
        # Update the timestamp
        self._last_feedback_command_time = current_time
    
    def update_fan_speed(self):
        """Handle fan speed changes from UI with improved validation and discrete steps"""
        # Skip if we're in startup fan setting period
        if hasattr(self, '_startup_fan_set') and self._startup_fan_set:
            return
            
        if self.system_running:
            try:
                fan_speed = int(self.fan_speed_input.text())
                # Ensure fan speed is between 30% and 100%
                if 30 <= fan_speed <= 100:
                    # Normalize to valid discrete steps (30, 40, 50, 60, 70, 80, 90, 100)
                    normalized_fan_speed = self.normalize_fan_speed(fan_speed)
                    
                    # If the user entered a non-standard value, show feedback
                    if normalized_fan_speed != fan_speed:
                        logger.info(f"Normalizing fan speed from {fan_speed} to {normalized_fan_speed}")
                        QMessageBox.information(self, "Discrete Fan Speed", 
                                            f"Fan speed adjusted to {normalized_fan_speed}% (valid values: 30, 40, 50, 60, 70, 80, 90, 100)")
                    
                    # Send the normalized value
                    success = self.send_command(f"F{normalized_fan_speed}")
                    if success:
                        self.fan_slider.setValue(normalized_fan_speed)
                        self.fan_speed_input.setText(str(normalized_fan_speed))
                    else:
                        self.fan_speed_input.setText(str(self.current_fan_speed))
                else:
                    logger.warning(f"Invalid fan speed value: {fan_speed} - must be 30-100%")
                    self.fan_speed_input.setText(str(self.current_fan_speed))
                    QMessageBox.warning(self, "Invalid Value", "Fan speed must be between 30% and 100%.")
            except ValueError:
                logger.warning(f"Non-numeric fan speed value entered: {self.fan_speed_input.text()}")
                self.fan_speed_input.setText(str(self.current_fan_speed))
                QMessageBox.warning(self, "Invalid Value", "Fan speed must be a number.")
        else:
            # Don't allow fan control when system is stopped
            logger.info("Cannot change fan speed when system is stopped")
            self.fan_speed_input.setText(str(self.current_fan_speed))
            QMessageBox.information(self, "System Stopped", "Cannot change fan speed when system is stopped.")

    def slider_fan_update(self, value):
        """Handle fan speed changes from slider with error handling and discrete steps"""
        # Skip if we're in startup fan setting period
        if hasattr(self, '_startup_fan_set') and self._startup_fan_set:
            return
        
        # Normalize to discrete values (30, 40, 50, 60, 70, 80, 90, 100)
        normalized_value = self.normalize_fan_speed(value)
        
        # If value isn't already at a valid step, update the slider position
        if value != normalized_value:
            self.fan_slider.blockSignals(True)
            self.fan_slider.setValue(normalized_value)
            self.fan_slider.blockSignals(False)
            
        if self.system_running and normalized_value != self.current_fan_speed:
            self.fan_speed_input.setText(str(normalized_value))
            self.send_command(f"F{normalized_value}")
    
    def update_threshold(self):
        """Handle protection threshold changes from UI with improved validation"""
        try:
            thresh = float(self.threshold_input.text())
            if 1 <= thresh <= 300:
                self.temp_threshold = thresh
                # Subtract 5 degrees from threshold before sending to Arduino
                adjusted_threshold = thresh
                success = self.send_command(f"T{adjusted_threshold}")
                if not success:
                    self.threshold_input.setText(str(self.temp_threshold))
            else:
                logger.warning(f"Invalid threshold value: {thresh} - must be 1-300°C")
                self.threshold_input.setText(str(self.temp_threshold))
                QMessageBox.warning(self, "Invalid Value", "Threshold must be between 1°C and 300°C.")
        except ValueError:
            logger.warning(f"Non-numeric threshold value entered: {self.threshold_input.text()}")
            self.threshold_input.setText(str(self.temp_threshold))
            QMessageBox.warning(self, "Invalid Value", "Threshold must be a number.")
        
    
    def start_system(self):
        """Start the heating system with error handling"""
        # Clear protection status when manually starting (if user confirms)
        if self.protection_active:
            reply = QMessageBox.question(self, "Protection Active", 
                                         "A protection condition was previously detected. Are you sure you want to start the system?",
                                         QMessageBox.Yes | QMessageBox.No, QMessageBox.No)
            if reply == QMessageBox.No:
                return
            # If user confirms, continue with start and clear protection flag
            self.protection_active = False
            self.current_protection_message = ""
            
        # Original start_system code follows
        if not self.system_running:
            if self.connection_alive and self.initialization_complete:
                # Use the last temperature from temp_data instead of current_temperature
                current_temp = self.temp_data[-1] if self.temp_data else 0
                # Check if relay_activated exists, otherwise assume it's False
                relay_activated = getattr(self, 'relay_activated', False)
                
                if current_temp < self.temp_threshold or not relay_activated:
                    success = self.send_command("RSTART")
                    if success:
                        self.system_running = True
                        self.set_system_state(SystemState.RUNNING)
                        self.status_message = "System STARTING..."
                        self.status_label.setText(self.status_message)
                        self.start_button.setEnabled(False)
                        self.stop_button.setEnabled(True)
                        
                        # Add a flag to prevent duplicate fan commands during startup
                        self._startup_fan_set = True
                        
                        # Set fan speed to 100% when system starts
                        self.current_fan_speed = 100  # Update internal state first
                        self.fan_speed_input.setText("100")
                        self.fan_slider.setValue(100)
                        self.send_command("F100")
                        
                        # Schedule flag removal after startup is complete
                        QTimer.singleShot(2000, lambda: setattr(self, '_startup_fan_set', False))
                    else:
                        QMessageBox.warning(self, "Communication Error", 
                                        "Failed to send start command. Please check connection.")
                else:
                    QMessageBox.warning(self, "Overtemperature", 
                                    "Cannot start system. Temperature still exceeds threshold.")
            else:
                QMessageBox.warning(self, "System Not Ready", 
                                "System is not connected or initialization is not complete.")

    def set_initial_fan_speed(self):
        """Set the initial fan speed after system start"""
        if self.system_running:
            # Set the fan speed in the UI
            self.current_fan_speed = 100
            self.fan_speed_input.setText("100")
            self.fan_slider.setValue(100)
            
            # Send the fan command
            self.send_command("F100")
    
    def stop_system(self):
        """Stop the heating system with error handling"""
        if self.system_running:
            success = self.send_command("RSTOP")
            if success:
                self.system_running = False
                self.set_system_state(SystemState.STOPPED)
                self.status_message = "System STOPPING..."
                self.status_label.setText(self.status_message)
                self.start_button.setEnabled(True)
                self.stop_button.setEnabled(False)
                # Set fan speed to minimum value when stopped
                self.fan_speed_input.setText("30")
                self.fan_slider.setValue(30)
                
                # Clear any protection status when manually stopping
                self.protection_active = False
                self.current_protection_message = ""
            else:
                QMessageBox.warning(self, "Communication Error", 
                                   "Failed to send stop command. Please check connection.")
    
    def closeEvent(self, event):
        """Clean up resources when window is closed"""
        self.running = False
        # Stop system if running
        if self.system_running:
            self.send_command("RSTOP")
            
        if self.ser and self.ser.is_open:
            try:
                self.ser.close()
            except Exception as e:
                logger.error(f"Error closing serial port: {e}")
                
        logger.info("Application closing")
        event.accept()


# Changes to the main function at the bottom of the file

if __name__ == "__main__":
    # Create the application
    app = QApplication(sys.argv)
    
    # Set application style
    app.setStyle('Fusion')
    
    # Create a splash screen for feedback during initialization
    splash_pix = QPixmap(400, 200)
    splash_pix.fill(QColor('#f0f0f0'))
    
    splash = QSplashScreen(splash_pix)
    splash_layout = QVBoxLayout()
    splash_label = QLabel("Initializing Temperature Controller...")
    splash_label.setStyleSheet("font-size: 16pt; font-weight: bold; color: #2c3e50;")
    splash_label.setAlignment(Qt.AlignCenter)
    splash_layout.addWidget(splash_label)
    
    # Add a progress bar
    progress = QProgressBar()
    progress.setRange(0, 0)  # Indeterminate progress
    progress.setTextVisible(False)
    progress.setFixedHeight(10)
    splash_layout.addWidget(progress)
    
    # Set the layout on a widget
    splash_widget = QWidget()
    splash_widget.setLayout(splash_layout)
    
    # Create a QVBoxLayout instance and set it on splash
    splash.setLayout(splash_layout)
    
    # Show the splash screen
    splash.show()
    app.processEvents()
    
    # Create and show the main window
    try:
        # Show main window with port discovery deferred
        controller = TemperatureControllerGUI(baudrate=115200, defer_port_discovery=True)
        controller.show()
        
        # Hide the splash screen once the main window is shown
        splash.finish(controller)
        
        # Now that the GUI is shown, perform port discovery
        QTimer.singleShot(100, controller.refresh_port_list)
        
        # Start the event loop
        sys.exit(app.exec_())
    except Exception as e:
        logger.critical(f"Fatal error starting application: {e}")
        QMessageBox.critical(None, "Fatal Error", f"Failed to start application: {str(e)}")
        sys.exit(1)