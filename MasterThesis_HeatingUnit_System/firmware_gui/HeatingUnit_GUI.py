"""
Heating Unit GUI

This is a graphical user interface for controlling the heating unit
of the RS PRO 858D SMD rework station modification.

Author: Basil Jacob
License: MIT
"""

import tkinter as tk
from tkinter import ttk

class HeatingUnitGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Heating Unit Control")
        self.root.geometry("400x300")
        
        # Create main frame
        main_frame = ttk.Frame(root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Title
        title_label = ttk.Label(main_frame, text="Heating Unit Control System", 
                                font=("Arial", 16, "bold"))
        title_label.grid(row=0, column=0, columnspan=2, pady=10)
        
        # Current temperature display
        ttk.Label(main_frame, text="Current Temperature:").grid(row=1, column=0, sticky=tk.W, pady=5)
        self.current_temp_label = ttk.Label(main_frame, text="0.0 °C", font=("Arial", 12))
        self.current_temp_label.grid(row=1, column=1, sticky=tk.W, pady=5)
        
        # Target temperature input
        ttk.Label(main_frame, text="Target Temperature:").grid(row=2, column=0, sticky=tk.W, pady=5)
        self.target_temp_entry = ttk.Entry(main_frame, width=10)
        self.target_temp_entry.grid(row=2, column=1, sticky=tk.W, pady=5)
        
        # Control buttons
        button_frame = ttk.Frame(main_frame)
        button_frame.grid(row=3, column=0, columnspan=2, pady=20)
        
        self.start_button = ttk.Button(button_frame, text="Start", command=self.start_heating)
        self.start_button.pack(side=tk.LEFT, padx=5)
        
        self.stop_button = ttk.Button(button_frame, text="Stop", command=self.stop_heating)
        self.stop_button.pack(side=tk.LEFT, padx=5)
        
        # Status label
        self.status_label = ttk.Label(main_frame, text="Status: Idle", foreground="blue")
        self.status_label.grid(row=4, column=0, columnspan=2, pady=10)
        
    def start_heating(self):
        """Start the heating process"""
        target_temp = self.target_temp_entry.get()
        self.status_label.config(text=f"Status: Heating to {target_temp}°C", foreground="green")
        # TODO: Implement serial communication with Arduino
        
    def stop_heating(self):
        """Stop the heating process"""
        self.status_label.config(text="Status: Stopped", foreground="red")
        # TODO: Implement serial communication with Arduino

def main():
    root = tk.Tk()
    app = HeatingUnitGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()
