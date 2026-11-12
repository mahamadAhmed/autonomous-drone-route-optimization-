"""
Interactive Tkinter GUI Visualizer for Drone Route Planning.
"""

import tkinter as tk
from tkinter import ttk
from typing import Optional, List
from .router import DroneRoutePlanner


class DroneDeliveryGUI:
    """Tkinter-based spatial graph viewer and route execution interface."""

    def __init__(self, planner: Optional[DroneRoutePlanner] = None):
        self.planner = planner if planner is not None else DroneRoutePlanner()
        self.graph = self.planner.graph
        self.coordinates = self.planner.coordinates

        self.root = tk.Tk()
        self.root.title("Autonomous Drone Route Planning & Optimization")
        self.root.geometry("640x420")
        self.root.resizable(False, False)

        self._init_ui()

    def _init_ui(self):
        # Control Panel (Left)
        control_frame = ttk.Frame(self.root, padding="15")
        control_frame.pack(side=tk.LEFT, fill=tk.Y)

        ttk.Label(
            control_frame,
            text="Drone Navigation",
            font=("Helvetica", 14, "bold")
        ).pack(anchor=tk.W, pady=(0, 15))

        nodes = list(self.graph.keys())

        # Start Node
        ttk.Label(control_frame, text="Start Waypoint (Origin):").pack(anchor=tk.W)
        self.start_var = tk.StringVar(value=nodes[0] if nodes else "W")
        start_cb = ttk.Combobox(control_frame, textvariable=self.start_var, values=nodes, state="readonly")
        start_cb.pack(anchor=tk.W, fill=tk.X, pady=(2, 10))

        # Goal Node
        ttk.Label(control_frame, text="Target Delivery Goal:").pack(anchor=tk.W)
        self.goal_var = tk.StringVar(value="E" if "E" in nodes else nodes[-1])
        goal_cb = ttk.Combobox(control_frame, textvariable=self.goal_var, values=nodes, state="readonly")
        goal_cb.pack(anchor=tk.W, fill=tk.X, pady=(2, 15))

        # Action Button
        run_btn = ttk.Button(control_frame, text="Calculate Optimal Route", command=self.on_run)
        run_btn.pack(anchor=tk.W, fill=tk.X, pady=(0, 15))

        # Metrics Panel
        metrics_frame = ttk.LabelFrame(control_frame, text="Route Diagnostics", padding="10")
        metrics_frame.pack(anchor=tk.W, fill=tk.X)

        ttk.Label(metrics_frame, text="Optimal Path:", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        self.path_label = ttk.Label(metrics_frame, text="--", foreground="#0d6efd", wraplength=180)
        self.path_label.pack(anchor=tk.W, pady=(2, 8))

        ttk.Label(metrics_frame, text="Total Energy / Cost:", font=("Helvetica", 9, "bold")).pack(anchor=tk.W)
        self.cost_label = ttk.Label(metrics_frame, text="--", foreground="#198754")
        self.cost_label.pack(anchor=tk.W, pady=(2, 4))

        # Canvas Visualizer (Right)
        canvas_frame = ttk.Frame(self.root, padding="15")
        canvas_frame.pack(side=tk.RIGHT, expand=True, fill=tk.BOTH)

        self.canvas = tk.Canvas(canvas_frame, width=380, height=380, bg="#F8F9FA", highlightthickness=1, highlightbackground="#DEE2E6")
        self.canvas.pack()

        self.redraw_graph()

    def redraw_graph(self, path: Optional[List[str]] = None):
        self.canvas.delete("all")

        # Draw all graph edges
        for u, neighbors in self.graph.items():
            if u not in self.coordinates:
                continue
            x1, y1 = self.coordinates[u]
            for v, weight in neighbors.items():
                if v not in self.coordinates:
                    continue
                x2, y2 = self.coordinates[v]
                self.canvas.create_line(x1, y1, x2, y2, fill="#ADB5BD", width=2)
                # Label edge cost
                mid_x = (x1 + x2) / 2
                mid_y = (y1 + y2) / 2
                self.canvas.create_text(mid_x, mid_y, text=str(int(weight)), fill="#6C757D", font=("Helvetica", 8))

        # Highlight optimal path if present
        if path and len(path) > 1:
            for i in range(len(path) - 1):
                u, v = path[i], path[i + 1]
                if u in self.coordinates and v in self.coordinates:
                    x1, y1 = self.coordinates[u]
                    x2, y2 = self.coordinates[v]
                    self.canvas.create_line(x1, y1, x2, y2, fill="#0D6EFD", width=4)

        # Draw waypoints / nodes
        for node, (x, y) in self.coordinates.items():
            is_in_path = path and node in path
            fill_color = "#0D6EFD" if is_in_path else "#E9ECEF"
            text_color = "#FFFFFF" if is_in_path else "#212529"
            outline_color = "#0A58CA" if is_in_path else "#6C757D"

            self.canvas.create_oval(x - 14, y - 14, x + 14, y + 14, fill=fill_color, outline=outline_color, width=2)
            self.canvas.create_text(x, y, text=node, fill=text_color, font=("Helvetica", 10, "bold"))

    def on_run(self):
        start = self.start_var.get()
        goal = self.goal_var.get()

        path, cost = self.planner.find_shortest_path(start, goal)
        if path:
            self.path_label.config(text=" → ".join(path))
            self.cost_label.config(text=f"{cost:.1f} units")
            self.redraw_graph(path)
        else:
            self.path_label.config(text="No valid path found")
            self.cost_label.config(text="N/A")
            self.redraw_graph(None)

    def run(self):
        self.root.mainloop()

