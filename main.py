"""
Main entry point for Drone Route Optimization.
Supports both interactive Tkinter visualization and headless CLI path queries.
"""

import argparse
import sys
from src.router import DroneRoutePlanner


def run_cli(start: str, goal: str):
    planner = DroneRoutePlanner()
    print(f"\n========================================================")
    print(f" Autonomous Drone Route Optimization - Path Evaluation")
    print(f"========================================================")
    print(f"Origin (Warehouse / Waypoint): {start}")
    print(f"Target Destination:            {goal}\n")

    path, cost = planner.find_shortest_path(start, goal)

    if path:
        print(f" Status:      OPTIMAL PATH FOUND")
        print(f" Waypoints:   {' -> '.join(path)}")
        print(f" Total Cost:  {cost:.2f} distance units")
        print(f" Step Count:  {len(path) - 1} transitions")
    else:
        print(f" Status:      NO PATH FOUND between {start} and {goal}")
    print(f"========================================================\n")


def main():
    parser = argparse.ArgumentParser(description="Autonomous Drone Delivery Route Optimizer (A* Search)")
    parser.add_argument("--cli", action="store_true", help="Run in headless CLI mode without GUI")
    parser.add_argument("--start", type=str, default="W", help="Start node key (default: W)")
    parser.add_argument("--goal", type=str, default="E", help="Goal node key (default: E)")

    args = parser.parse_args()

    if args.cli:
        run_cli(args.start, args.goal)
    else:
        try:
            from src.gui import DroneDeliveryGUI
            app = DroneDeliveryGUI()
            app.run()
        except Exception as e:
            print(f"[Warning] GUI initialization failed ({e}), falling back to CLI execution:")
            run_cli(args.start, args.goal)


if __name__ == "__main__":
    main()

