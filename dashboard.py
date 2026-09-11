"""
Edge-AI AMR Fleet Coordination Demo (Python/Tkinter) — Presentation & Control Room Layer.

The simulation and decentralized coordination engine is decoupled into the `backend/` package:
- Space-Time A* MAPF planner (4D search preventing vertex & swap conflicts)
- Decentralized Reservation Table
- Multi-factor Dynamic Priority Engine (anti-starvation aging & battery protection)
- Distributed Conflict Resolution (junction yield, space-time detours, sidestep)
- Decentralized P2P Mesh Network with comms-loss simulation
- Contract Net Protocol Task Auctions
"""

from __future__ import annotations

import math
import random
import time
import tkinter as tk
from typing import Dict, List, Optional, Tuple

from backend import (
    BG,
    CELL_SIZE as CELL,
    COLS,
    DEFAULT_SIM_SPEED,
    PANEL,
    ROBOT_COLORS,
    ROWS,
    SHELVES,
    SLOW_TICKS,
    STATIONS,
    TICK_MS,
    ConflictType,
    FleetModel,
    Point,
    ResolutionAction,
    Robot,
    TaskPriority,
)

CANVAS_W, CANVAS_H = 900, 600


class FleetDashboard(tk.Tk):
    """Presentation-focused monitoring UI. FleetModel remains the coordination engine."""

    def __init__(self):
        super().__init__()
        self.title("PiTuPi Control Room — Decentralized Coordination Demo")
        self.configure(bg="#0A0F16")
        self.geometry("1440x900")
        self.minsize(1180, 760)

        self.view_mode = "2d"
        self.azimuth = 0.65
        self.elevation = 0.72
        self.zoom = 1.0
        self.drag_last = None
        self.comms_restore_job = None
        self.log_entries = []
        self.speed_levels = [0.25, 0.5, 1.0, 1.5, 2.0, 3.0, 4.0]

        self.ui = {
            "bg": "#0A0F16",
            "surface": "#101722",
            "surface2": "#151E2A",
            "surface3": "#1A2533",
            "border": "#253244",
            "text": "#EAF1F8",
            "muted": "#8291A5",
            "cyan": "#42C8E8",
            "green": "#4ED089",
            "amber": "#F2B84B",
            "red": "#F1686A",
            "violet": "#A98BF5",
        }

        self._build_ui()
        self.model = FleetModel(self.push_log, self.render_metrics)
        self.obstacle_mode = False
        self.render_all()
        self.after(TICK_MS, self.loop)

    def _label(self, parent, text="", **kwargs):
        return tk.Label(
            parent,
            text=text,
            bg=kwargs.pop("bg", self.ui["surface"]),
            fg=kwargs.pop("fg", self.ui["text"]),
            font=kwargs.pop("font", ("Segoe UI", 10)),
            **kwargs,
        )

    def _card(self, parent, bg=None):
        return tk.Frame(
            parent,
            bg=bg or self.ui["surface"],
            highlightbackground=self.ui["border"],
            highlightthickness=1,
            bd=0,
        )

    def _build_ui(self):
        # ---------- Header ----------
        header = tk.Frame(self, bg=self.ui["surface"], height=78)
        header.pack(fill="x", side="top")
        header.pack_propagate(False)

        brand = tk.Frame(header, bg=self.ui["surface"])
        brand.pack(side="left", padx=22, pady=13)
        tk.Label(brand, text="PiTuPi", bg=self.ui["surface"], fg=self.ui["cyan"],
                 font=("Segoe UI", 9, "bold")).pack(anchor="w")
        tk.Label(brand, text="Decentralized Coordination Control Room", bg=self.ui["surface"],
                 fg=self.ui["text"], font=("Segoe UI", 17, "bold")).pack(anchor="w")

        status_wrap = tk.Frame(header, bg=self.ui["surface"])
        status_wrap.pack(side="right", padx=20)
        self.net_dot = tk.Label(status_wrap, text="●", bg=self.ui["surface"], fg=self.ui["green"],
                                font=("Segoe UI", 13, "bold"))
        self.net_dot.pack(side="left", padx=(0, 7))
        self.net_var = tk.StringVar(value="P2P MESH HEALTHY")
        self.net_label = tk.Label(status_wrap, textvariable=self.net_var, bg=self.ui["surface"],
                                  fg=self.ui["green"], font=("Segoe UI", 9, "bold"))
        self.net_label.pack(side="left")

        # ---------- KPI strip ----------
        kpi_strip = tk.Frame(self, bg=self.ui["bg"])
        kpi_strip.pack(fill="x", padx=16, pady=(14, 10))
        self.metric_vars = {k: tk.StringVar(value="0") for k in ("collisions", "completed", "speedup", "reroutes")}
        self.metric_vars["speedup"].set("—")

        kpis = [
            ("collisions", "Safety events", self.ui["green"]),
            ("completed", "Tasks completed", self.ui["cyan"]),
            ("reroutes", "Dynamic reroutes", self.ui["violet"]),
            ("speedup", "Vs. stop/wait baseline", self.ui["green"]),
        ]
        for i, (key, subtitle, accent) in enumerate(kpis):
            card = self._card(kpi_strip, self.ui["surface"])
            card.grid(row=0, column=i, sticky="nsew", padx=(0 if i == 0 else 5, 0 if i == len(kpis)-1 else 5))
            kpi_strip.grid_columnconfigure(i, weight=1)
            inner = tk.Frame(card, bg=self.ui["surface"])
            inner.pack(fill="both", expand=True, padx=14, pady=10)
            tk.Label(inner, textvariable=self.metric_vars[key], bg=self.ui["surface"], fg=accent,
                     font=("Consolas", 20, "bold")).pack(anchor="w")
            tk.Label(inner, text=subtitle, bg=self.ui["surface"], fg=self.ui["muted"],
                     font=("Segoe UI", 8)).pack(anchor="w")

        # ---------- Main content ----------
        main = tk.Frame(self, bg=self.ui["bg"])
        main.pack(fill="both", expand=True, padx=16, pady=(0, 10))
        main.grid_columnconfigure(0, weight=1)
        main.grid_columnconfigure(1, weight=0)
        main.grid_rowconfigure(0, weight=1)

        stage_card = self._card(main, self.ui["surface"])
        stage_card.grid(row=0, column=0, sticky="nsew", padx=(0, 10))
        # Scrollable right rail: ROBOT HEALTH + SCENARIO CONTROLS always fit,
        # even on smaller screens / Windows DPI scaling.
        side_host = tk.Frame(main, bg=self.ui["bg"], width=340)
        side_host.grid(row=0, column=1, sticky="nsew")
        side_host.grid_propagate(False)

        side_canvas = tk.Canvas(
            side_host, bg=self.ui["bg"], highlightthickness=0, bd=0
        )
        side_scroll = tk.Scrollbar(
            side_host, orient="vertical", command=side_canvas.yview,
            width=9, relief="flat", bd=0
        )
        side = tk.Frame(side_canvas, bg=self.ui["bg"], bd=0, highlightthickness=0)
        side_window = side_canvas.create_window((0, 0), window=side, anchor="nw")
        side_canvas.configure(yscrollcommand=side_scroll.set)

        side_canvas.pack(side="left", fill="both", expand=True)
        side_scroll.pack(side="right", fill="y")

        def _side_configure(_event=None):
            side_canvas.configure(scrollregion=side_canvas.bbox("all"))

        def _side_width(event):
            side_canvas.itemconfigure(side_window, width=event.width)

        side.bind("<Configure>", _side_configure)
        side_canvas.bind("<Configure>", _side_width)

        def _side_wheel(event):
            if event.delta:
                side_canvas.yview_scroll(int(-event.delta / 120), "units")
            return "break"

        side_canvas.bind("<MouseWheel>", _side_wheel)
        side.bind("<MouseWheel>", _side_wheel)

        # Give the right rail enough room to remain stable while the main map
        # receives the remaining width.
        main.grid_columnconfigure(1, minsize=340)

        # stage toolbar
        stage_top = tk.Frame(stage_card, bg=self.ui["surface"])
        stage_top.pack(fill="x", padx=14, pady=(12, 8))
        title_box = tk.Frame(stage_top, bg=self.ui["surface"])
        title_box.pack(side="left")
        tk.Label(title_box, text="LIVE WAREHOUSE MAP", bg=self.ui["surface"], fg=self.ui["text"],
                 font=("Segoe UI", 10, "bold")).pack(anchor="w")
        tk.Label(title_box, text="11 × 8 grid • Space-Time MAPF • Decentralized P2P Conflict Resolution", bg=self.ui["surface"],
                 fg=self.ui["muted"], font=("Segoe UI", 8)).pack(anchor="w")

        # Simulation speed controls
        speed_wrap = tk.Frame(stage_top, bg=self.ui["surface2"], highlightbackground=self.ui["border"], highlightthickness=1)
        speed_wrap.pack(side="right", padx=(0, 10))
        tk.Button(speed_wrap, text="−", command=lambda: self.change_speed(-1), bg=self.ui["surface2"], fg=self.ui["muted"],
                  activebackground=self.ui["surface3"], activeforeground=self.ui["text"], bd=0, relief="flat",
                  font=("Segoe UI", 11, "bold"), padx=9, pady=3, cursor="hand2").pack(side="left")
        self.speed_var = tk.StringVar(value="1.0×")
        tk.Label(speed_wrap, textvariable=self.speed_var, bg=self.ui["surface2"], fg=self.ui["cyan"],
                 font=("Consolas", 9, "bold"), width=5).pack(side="left")
        tk.Button(speed_wrap, text="+", command=lambda: self.change_speed(1), bg=self.ui["surface2"], fg=self.ui["muted"],
                  activebackground=self.ui["surface3"], activeforeground=self.ui["text"], bd=0, relief="flat",
                  font=("Segoe UI", 11, "bold"), padx=9, pady=3, cursor="hand2").pack(side="left")

        view_switch = tk.Frame(stage_top, bg=self.ui["surface2"], highlightbackground=self.ui["border"], highlightthickness=1)
        view_switch.pack(side="right")
        self.btn_2d = self._pill(view_switch, "2D", lambda: self.set_view("2d"), active=True)
        self.btn_3d = self._pill(view_switch, "3D", lambda: self.set_view("3d"), active=False)

        self.canvas = tk.Canvas(stage_card, width=CANVAS_W, height=CANVAS_H,
                                bg="#0D141D", highlightthickness=0)
        self.canvas.pack(fill="both", expand=True, padx=10, pady=(0, 8))
        self.canvas.bind("<ButtonPress-1>", self.on_canvas_press)
        self.canvas.bind("<ButtonRelease-1>", self.on_canvas_release)
        self.canvas.bind("<Button-3>", self.on_right_click)
        self.canvas.bind("<B1-Motion>", self.on_drag)
        self.canvas.bind("<MouseWheel>", self.on_wheel)

        legend = tk.Frame(stage_card, bg=self.ui["surface"])
        legend.pack(fill="x", padx=13, pady=(0, 11))
        tk.Label(legend, text="STATUS", bg=self.ui["surface"], fg=self.ui["muted"], font=("Segoe UI", 8, "bold")).pack(side="left")
        for dot, txt in [(self.ui["cyan"], "Moving"), (self.ui["amber"], "Yielding"),
                         (self.ui["violet"], "Rerouting"), (self.ui["red"], "Blocked / offline")]:
            tk.Label(legend, text="●", bg=self.ui["surface"], fg=dot, font=("Segoe UI", 9)).pack(side="left", padx=(14, 3))
            tk.Label(legend, text=txt, bg=self.ui["surface"], fg=self.ui["muted"], font=("Segoe UI", 8)).pack(side="left")

        # ---------- right column ----------
        fleet_card = self._card(side, self.ui["surface"])
        fleet_card.pack(fill="both", expand=True, pady=(0, 10))
        fleet_head = tk.Frame(fleet_card, bg=self.ui["surface"])
        fleet_head.pack(fill="x", padx=13, pady=(12, 6))
        tk.Label(fleet_head, text="ROBOT HEALTH", bg=self.ui["surface"], fg=self.ui["text"],
                 font=("Segoe UI", 10, "bold")).pack(side="left")
        self.robot_count_var = tk.StringVar(value="3 online")
        tk.Label(fleet_head, textvariable=self.robot_count_var, bg=self.ui["surface"], fg=self.ui["green"],
                 font=("Segoe UI", 8, "bold")).pack(side="right")
        self.cards_frame = tk.Frame(fleet_card, bg=self.ui["surface"])
        self.cards_frame.pack(fill="both", expand=True, padx=10, pady=(0, 8))

        action_card = self._card(side, self.ui["surface"])
        action_card.pack(fill="x")
        tk.Label(action_card, text="SCENARIO CONTROLS", bg=self.ui["surface"], fg=self.ui["text"],
                 font=("Segoe UI", 10, "bold")).pack(anchor="w", padx=13, pady=(12, 7))
        tk.Label(action_card, text="Inject failures live during the demo", bg=self.ui["surface"], fg=self.ui["muted"],
                 font=("Segoe UI", 8)).pack(anchor="w", padx=13, pady=(0, 8))
        actions = tk.Frame(action_card, bg=self.ui["surface"])
        actions.pack(fill="x", padx=10, pady=(0, 10))
        self._action_button(actions, "Place obstacle", "＋", self.toggle_obstacle_mode, self.ui["amber"]).grid(row=0, column=0, sticky="ew", padx=(0,4), pady=4)
        self._action_button(actions, "Obstacle", "▰", self.inject_obstacle, self.ui["amber"]).grid(row=0, column=1, sticky="ew", padx=(4,0), pady=4)
        self._action_button(actions, "Robot fail", "×", self.fail_robot, self.ui["red"]).grid(row=1, column=0, sticky="ew", padx=(0,4), pady=4)
        self._action_button(actions, "Drop comms", "⌁", self.drop_comms, self.ui["violet"]).grid(row=1, column=1, sticky="ew", padx=(4,0), pady=4)
        self._action_button(actions, "Reset", "↻", self.reset_sim, self.ui["cyan"]).grid(row=2, column=0, columnspan=2, sticky="ew", padx=(0,0), pady=4)
        actions.grid_columnconfigure(0, weight=1)
        actions.grid_columnconfigure(1, weight=1)

        # ---------- event console ----------
        console = self._card(self, self.ui["surface"])
        console.pack(fill="x", padx=16, pady=(0, 14))
        console_head = tk.Frame(console, bg=self.ui["surface"])
        console_head.pack(fill="x", padx=13, pady=(9, 6))

        c_left = tk.Frame(console_head, bg=self.ui["surface"])
        c_left.pack(side="left")
        tk.Label(c_left, text="DECENTRALIZED EVENT CONSOLE", bg=self.ui["surface"], fg=self.ui["cyan"],
                 font=("Segoe UI", 9, "bold")).pack(side="left")
        self.log_count_var = tk.StringVar(value="0 events")
        tk.Label(c_left, textvariable=self.log_count_var, bg=self.ui["surface2"], fg=self.ui["muted"],
                 font=("Segoe UI", 7, "bold"), padx=6, pady=1).pack(side="left", padx=(8, 0))

        tk.Button(console_head, text="Clear Console", command=self.clear_console, bg=self.ui["surface2"], fg=self.ui["muted"],
                  activebackground=self.ui["surface3"], activeforeground=self.ui["text"], bd=0, font=("Segoe UI", 8),
                  padx=8, pady=2, cursor="hand2").pack(side="right")
        tk.Label(console_head, text="live peer decisions • Space-Time MAPF • no central controller", bg=self.ui["surface"],
                 fg=self.ui["muted"], font=("Segoe UI", 8)).pack(side="right", padx=(0, 12))

        log_container = tk.Frame(console, bg="#0B1119", bd=0)
        log_container.pack(fill="x", padx=9, pady=(0, 9))

        log_scroll = tk.Scrollbar(log_container, orient="vertical", width=9, relief="flat", bd=0)
        self.log_text = tk.Text(log_container, height=8, bg="#0B1119", fg=self.ui["text"], insertbackground=self.ui["text"],
                                yscrollcommand=log_scroll.set, bd=0, font=("Consolas", 9), wrap="word", padx=10, pady=8)
        log_scroll.configure(command=self.log_text.yview)
        log_scroll.pack(side="right", fill="y")
        self.log_text.pack(side="left", fill="both", expand=True)

        self.log_text.configure(state="disabled")
        for tag, color in [("comm", "#52D1C4"), ("conflict", self.ui["amber"]),
                           ("reroute", self.ui["violet"]), ("fail", self.ui["red"]),
                           ("task", self.ui["green"]), ("mapf", self.ui["cyan"]),
                           ("time", "#66758A")]:
            self.log_text.tag_configure(tag, foreground=color)

    def _pill(self, parent, text, cmd, active=False):
        fg = self.ui["bg"] if active else self.ui["muted"]
        bg = self.ui["cyan"] if active else self.ui["surface2"]
        b = tk.Button(parent, text=text, command=cmd, bg=bg, fg=fg, activebackground=self.ui["cyan"],
                      activeforeground=self.ui["bg"], bd=0, relief="flat", font=("Segoe UI", 8, "bold"),
                      padx=12, pady=5, cursor="hand2")
        b.pack(side="left")
        return b

    def _action_button(self, parent, text, icon, cmd, accent):
        b = tk.Button(parent, text=f"{icon}  {text}", command=cmd, bg=self.ui["surface2"], fg=accent,
                      activebackground=self.ui["surface3"], activeforeground=accent,
                      bd=0, relief="flat", font=("Segoe UI", 9, "bold"), padx=9, pady=9, cursor="hand2")
        b.bind("<Enter>", lambda e, btn=b: btn.configure(bg=self.ui["surface3"]))
        b.bind("<Leave>", lambda e, btn=b: btn.configure(bg=self.ui["surface2"]))
        return b

    def change_speed(self, direction):
        current = self.model.sim_speed
        # Find the closest supported speed, then move one step up/down.
        idx = min(range(len(self.speed_levels)), key=lambda i: abs(self.speed_levels[i] - current))
        idx = max(0, min(len(self.speed_levels) - 1, idx + direction))
        self.model.sim_speed = self.speed_levels[idx]
        self.speed_var.set(f"{self.model.sim_speed:g}×")

    def set_view(self, mode):
        if mode == self.view_mode:
            return
        self.view_mode = mode
        if mode == "2d":
            self.btn_2d.configure(bg=self.ui["cyan"], fg=self.ui["bg"])
            self.btn_3d.configure(bg=self.ui["surface2"], fg=self.ui["muted"])
        else:
            self.btn_3d.configure(bg=self.ui["cyan"], fg=self.ui["bg"])
            self.btn_2d.configure(bg=self.ui["surface2"], fg=self.ui["muted"])
        self.render_stage()

    def clear_console(self):
        self.log_entries.clear()
        if hasattr(self, "log_count_var"):
            self.log_count_var.set("0 events")
        if hasattr(self, "log_text"):
            self.log_text.configure(state="normal")
            self.log_text.delete("1.0", "end")
            self.log_text.configure(state="disabled")

    def push_log(self, tag, who, msg):
        ts = time.strftime("%H:%M:%S")
        self.log_entries.append((ts, tag, who, msg))
        self.log_entries = self.log_entries[-300:]
        if hasattr(self, "log_count_var"):
            self.log_count_var.set(f"{len(self.log_entries)} events")
        if not hasattr(self, "log_text"):
            return
        self.log_text.configure(state="normal")
        self.log_text.insert("end", ts + "  ", "time")
        self.log_text.insert("end", f"[{who:<8}]  ", tag)
        self.log_text.insert("end", msg + "\n")
        self.log_text.see("end")
        self.log_text.configure(state="disabled")

    def render_metrics(self, metrics):
        if not hasattr(self, "metric_vars"):
            return
        self.metric_vars["collisions"].set(str(metrics["collisions"]))
        self.metric_vars["completed"].set(str(metrics["completed"]))
        self.metric_vars["reroutes"].set(str(metrics["reroute_events"]))
        if metrics["actual_time_accum"] > 0 and metrics["baseline_time_estimate"] > 0:
            speedup = round((metrics["baseline_time_estimate"] - metrics["actual_time_accum"]) / metrics["baseline_time_estimate"] * 100)
            self.metric_vars["speedup"].set(f"{speedup}%")

    def reset_sim(self):
        if self.comms_restore_job:
            try:
                self.after_cancel(self.comms_restore_job)
            except Exception:
                pass
            self.comms_restore_job = None
        self.log_text.configure(state="normal")
        self.log_text.delete("1.0", "end")
        self.log_text.configure(state="disabled")
        self.log_entries.clear()
        self.net_var.set("P2P MESH HEALTHY")
        self.net_label.configure(fg=self.ui["green"])
        self.net_dot.configure(fg=self.ui["green"])
        self.model.reset()

    def toggle_obstacle_mode(self):
        self.obstacle_mode = not self.obstacle_mode
        if self.obstacle_mode:
            self.canvas.configure(cursor="crosshair")
            self.push_log("comm", "SYSTEM", "Live obstacle mode ON — click any free 2D grid cell to place an obstacle; right-click removes one.")
        else:
            self.canvas.configure(cursor="")
            self.push_log("comm", "SYSTEM", "Live obstacle mode OFF.")

    def grid_cell_from_event(self, event):
        if self.view_mode != "2d":
            return None
        _, _, cell = self.cell_to_px(0, 0)
        x0, y0, _ = self.cell_to_px(0, 0)
        x0 -= cell / 2
        y0 -= cell / 2
        c = int((event.x - x0) // cell)
        r = int((event.y - y0) // cell)
        if self.model.in_bounds(c, r):
            return (c, r)
        return None

    def on_canvas_press(self, event):
        if self.obstacle_mode and self.view_mode == "2d":
            cell = self.grid_cell_from_event(event)
            if cell is not None:
                if self.model.place_manual_obstacle(cell):
                    self.render_stage()
            return
        self.on_drag_start(event)

    def on_canvas_release(self, event):
        if not self.obstacle_mode:
            self.drag_last = None

    def on_right_click(self, event):
        if self.view_mode != "2d":
            return
        cell = self.grid_cell_from_event(event)
        if cell is not None and self.model.remove_manual_obstacle(cell):
            self.render_stage()

    def inject_obstacle(self):
        self.model.inject_obstacle()

    def fail_robot(self):
        self.model.fail_robot()

    def drop_comms(self):
        robot = next((r for r in self.model.robots if r.id == "A"), None)
        if not robot or not robot.alive or not robot.comms:
            return
        self.model.set_robot_a_comms(False)
        self.net_var.set("ROBOT A LINK LOST")
        self.net_label.configure(fg=self.ui["red"])
        self.net_dot.configure(fg=self.ui["red"])
        self.comms_restore_job = self.after(4000, self.restore_comms)

    def restore_comms(self):
        robot = next((r for r in self.model.robots if r.id == "A"), None)
        if robot and robot.alive:
            self.model.set_robot_a_comms(True)
            self.net_var.set("P2P MESH HEALTHY")
            self.net_label.configure(fg=self.ui["green"])
            self.net_dot.configure(fg=self.ui["green"])
        self.comms_restore_job = None

    def on_drag_start(self, event):
        if self.view_mode == "3d":
            self.drag_last = (event.x, event.y)

    def on_drag(self, event):
        if self.view_mode != "3d" or not self.drag_last:
            return
        dx, dy = event.x - self.drag_last[0], event.y - self.drag_last[1]
        self.drag_last = (event.x, event.y)
        self.azimuth -= dx * 0.008
        self.elevation = max(0.25, min(1.25, self.elevation - dy * 0.005))
        self.render_stage()

    def on_wheel(self, event):
        if self.view_mode != "3d":
            return
        self.zoom = max(0.55, min(1.8, self.zoom * (0.92 if event.delta < 0 else 1.08)))
        self.render_stage()

    def canvas_dims(self):
        return max(self.canvas.winfo_width(), 600), max(self.canvas.winfo_height(), 400)

    def cell_to_px(self, c, r):
        w, h = self.canvas_dims()
        cell = min(CELL, (w - 90) / COLS, (h - 75) / ROWS)
        ox = (w - COLS * cell) / 2 + cell / 2
        oy = (h - ROWS * cell) / 2 + cell / 2
        return ox + c * cell, oy + r * cell, cell

    def interp_grid_pos(self, robot: Robot):
        if not robot.path or robot.path_idx >= len(robot.path):
            return robot.c, robot.r
        cur = robot.path[robot.path_idx]
        nxt = robot.path[robot.path_idx + 1] if robot.path_idx + 1 < len(robot.path) else cur
        return (cur[0] + (nxt[0] - cur[0]) * robot.progress,
                cur[1] + (nxt[1] - cur[1]) * robot.progress)

    def render_stage(self):
        self.canvas.delete("all")
        if self.view_mode == "2d":
            self.draw_2d()
        else:
            self.draw_3d()

    def draw_2d(self):
        _, _, cell = self.cell_to_px(0, 0)
        x0, y0, _ = self.cell_to_px(0, 0)
        x0 -= cell / 2
        y0 -= cell / 2

        # floor shadow and border
        self.canvas.create_rectangle(x0-9, y0-9, x0+COLS*cell+9, y0+ROWS*cell+9,
                                     fill="#0A1017", outline="#1B2837", width=1)
        self.canvas.create_rectangle(x0, y0, x0+COLS*cell, y0+ROWS*cell,
                                     fill="#0D151F", outline="#2A3B4F", width=1)

        for c in range(COLS + 1):
            x = x0 + c * cell
            self.canvas.create_line(x, y0, x, y0 + ROWS * cell, fill="#1B2837")
        for r in range(ROWS + 1):
            y = y0 + r * cell
            self.canvas.create_line(x0, y, x0 + COLS * cell, y, fill="#1B2837")

        # coordinates improve judge readability
        for c in range(COLS):
            self.canvas.create_text(x0 + c*cell + cell/2, y0-13, text=str(c), fill="#52647A", font=("Consolas", 7))
        for r in range(ROWS):
            self.canvas.create_text(x0-14, y0 + r*cell + cell/2, text=str(r), fill="#52647A", font=("Consolas", 7))

        for sc, sr, sw, sh in SHELVES:
            x, y, _ = self.cell_to_px(sc, sr)
            left, top = x-cell/2+7, y-cell/2+7
            right, bottom = left+sw*cell-14, top+sh*cell-14
            self.canvas.create_rectangle(left+4, top+5, right+4, bottom+5, fill="#080C11", outline="")
            self.canvas.create_rectangle(left, top, right, bottom, fill="#182332", outline="#30435A", width=1)
            for i in range(1, 4):
                yy = top + (bottom-top)*i/4
                self.canvas.create_line(left+5, yy, right-5, yy, fill="#26364A")
            self.canvas.create_text((left+right)/2, (top+bottom)/2, text="RACK", fill="#50647B", font=("Segoe UI", 7, "bold"))

        for name, c, r in STATIONS:
            x, y, _ = self.cell_to_px(c, r)
            self.canvas.create_oval(x-18, y-18, x+18, y+18, fill="#101C27", outline="#36506A", width=2)
            self.canvas.create_text(x, y, text=name, fill="#9FB0C3", font=("Consolas", 8, "bold"))

        shown_obs = set()
        for robot in self.model.robots:
            if robot.temp_blocked and robot.temp_blocked not in shown_obs:
                shown_obs.add(robot.temp_blocked)
                x, y, _ = self.cell_to_px(*robot.temp_blocked)
                self.canvas.create_rectangle(x-cell/2+7, y-cell/2+7, x+cell/2-7, y+cell/2-7,
                                             fill="#33181F", outline=self.ui["red"], dash=(4,3), width=2)
                self.canvas.create_text(x, y-6, text="OBSTACLE", fill=self.ui["red"], font=("Segoe UI", 7, "bold"))
                self.canvas.create_text(x, y+8, text="×", fill=self.ui["red"], font=("Segoe UI", 15, "bold"))

        # User-placed live obstacles remain visible until removed or reset.
        for c, r in sorted(self.model.manual_obstacles):
            x, y, _ = self.cell_to_px(c, r)
            self.canvas.create_rectangle(x-cell/2+7, y-cell/2+7, x+cell/2-7, y+cell/2-7,
                                         fill="#352316", outline=self.ui["amber"], width=2)
            self.canvas.create_text(x, y-6, text="LIVE", fill=self.ui["amber"], font=("Segoe UI", 7, "bold"))
            self.canvas.create_text(x, y+8, text="×", fill=self.ui["amber"], font=("Segoe UI", 15, "bold"))

        for robot in self.model.robots:
            if robot.alive and robot.path:
                pts=[]
                for c,r in robot.path[robot.path_idx:]:
                    x,y,_=self.cell_to_px(c,r)
                    pts.extend([x,y])
                if len(pts)>=4:
                    self.canvas.create_line(*pts, fill=ROBOT_COLORS[robot.id], width=3, dash=(7,5), smooth=True)

        for robot in self.model.robots:
            gc, gr = self.interp_grid_pos(robot) if robot.alive else (robot.c, robot.r)
            x, y, _ = self.cell_to_px(gc, gr)
            col = ROBOT_COLORS[robot.id] if robot.alive else "#58616C"
            ring = None
            if robot.state == "slowing": ring = self.ui["amber"]
            if robot.state == "rerouting": ring = self.ui["violet"]
            if robot.state in ("blocked", "offline"): ring = self.ui["red"]
            if ring:
                self.canvas.create_oval(x-26, y-26, x+26, y+26, outline=ring, dash=(3,3), width=2)
            self.canvas.create_oval(x-17, y-17, x+17, y+17, fill="#0B1118", outline=col, width=3)
            self.canvas.create_text(x, y-1, text=robot.id, fill=col, font=("Consolas", 11, "bold"))
            self.canvas.create_text(x, y+29, text=robot.state.upper(), fill=ring or "#66788E", font=("Segoe UI", 6, "bold"))
            if robot.alive and not robot.comms:
                self.canvas.create_text(x, y-31, text="LINK LOST", fill=self.ui["red"], font=("Segoe UI", 7, "bold"))

    def project3d(self, gx, gy, z=0.0):
        w, h = self.canvas_dims()
        x = (gx - (COLS-1)/2) * 60
        y = (gy - (ROWS-1)/2) * 60
        ca, sa = math.cos(self.azimuth), math.sin(self.azimuth)
        xr = x*ca - y*sa
        yr = x*sa + y*ca
        sy, cy = math.sin(self.elevation), math.cos(self.elevation)
        return w/2 + xr*self.zoom, h/2 + (yr*sy - z*25*cy)*self.zoom

    def poly(self, points, fill, outline=""):
        self.canvas.create_polygon(*[v for p in points for v in p], fill=fill, outline=outline)

    def draw_box3d(self, c, r, cw, rh, height, fill):
        x0,y0=c-0.45,r-0.45; x1,y1=c+cw-0.55,r+rh-0.55
        b=[self.project3d(x0,y0,0),self.project3d(x1,y0,0),self.project3d(x1,y1,0),self.project3d(x0,y1,0)]
        t=[self.project3d(x0,y0,height),self.project3d(x1,y0,height),self.project3d(x1,y1,height),self.project3d(x0,y1,height)]
        self.poly([b[0],b[1],t[1],t[0]],"#17212D","#30435A")
        self.poly([b[1],b[2],t[2],t[1]],"#1B2735","#30435A")
        self.poly(t,fill,"#3B516B")

        # Rack texture/details: upright posts + multiple horizontal shelf beams.
        post_w = 0.10
        beam_h = 0.10
        for px in (x0, x1):
            self.poly([
                self.project3d(px-post_w, y0, 0), self.project3d(px+post_w, y0, 0),
                self.project3d(px+post_w, y0, height), self.project3d(px-post_w, y0, height)
            ], "#101923", "#40556B")
            self.poly([
                self.project3d(px-post_w, y1, 0), self.project3d(px+post_w, y1, 0),
                self.project3d(px+post_w, y1, height), self.project3d(px-post_w, y1, height)
            ], "#101923", "#40556B")
        for frac in (0.25, 0.50, 0.75):
            z = height * frac
            self.canvas.create_line(
                *self.project3d(x0-0.03, y0-0.03, z),
                *self.project3d(x1+0.03, y0-0.03, z),
                fill="#5A7188", width=3
            )
            self.canvas.create_line(
                *self.project3d(x0-0.03, y1+0.03, z),
                *self.project3d(x1+0.03, y1+0.03, z),
                fill="#40556B", width=2
            )

    def draw_3d(self):
        floor=[self.project3d(-0.6,-0.6),self.project3d(COLS-0.4,-0.6),self.project3d(COLS-0.4,ROWS-0.4),self.project3d(-0.6,ROWS-0.4)]
        self.poly(floor,"#0D151F","#30435A")
        for c in range(COLS+1):
            self.canvas.create_line(*self.project3d(c-0.5,-0.5),*self.project3d(c-0.5,ROWS-0.5),fill="#203044")
        for r in range(ROWS+1):
            self.canvas.create_line(*self.project3d(-0.5,r-0.5),*self.project3d(COLS-0.5,r-0.5),fill="#203044")
        for sc,sr,sw,sh in SHELVES:
            self.draw_box3d(sc,sr,sw,sh,2.2,"#1D2A39")
        for name,c,r in STATIONS:
            x,y=self.project3d(c,r,0.05)
            self.canvas.create_oval(x-11,y-7,x+11,y+7,fill="#101C27",outline="#36506A")
            self.canvas.create_text(x,y-13,text=name,fill="#9FB0C3",font=("Consolas",8,"bold"))
        shown=set()
        for robot in self.model.robots:
            if robot.temp_blocked and robot.temp_blocked not in shown:
                shown.add(robot.temp_blocked)
                self.draw_box3d(*robot.temp_blocked,1,1,0.45,"#4A2027")
        for c, r in sorted(self.model.manual_obstacles):
            self.draw_box3d(c, r, 1, 1, 0.55, "#5A3820")

        for robot in self.model.robots:
            if robot.alive and robot.path:
                pts=[]
                for c,r in robot.path[robot.path_idx:]: pts+=list(self.project3d(c,r,0.08))
                if len(pts)>=4: self.canvas.create_line(*pts,fill=ROBOT_COLORS[robot.id],width=3,dash=(7,5))
        ordered=[]
        for robot in self.model.robots:
            gc,gr=self.interp_grid_pos(robot) if robot.alive else (robot.c,robot.r)
            _,py=self.project3d(gc,gr,0); ordered.append((py,robot,gc,gr))
        for _,robot,gc,gr in sorted(ordered):
            x,y=self.project3d(gc,gr,0.6)
            col=ROBOT_COLORS[robot.id] if robot.alive else "#58616C"
            self.canvas.create_oval(x-18*self.zoom,y-9*self.zoom,x+18*self.zoom,y+9*self.zoom,fill=col,outline="#080C11",width=2)
            self.canvas.create_oval(x-10*self.zoom,y-15*self.zoom,x+10*self.zoom,y-3*self.zoom,fill="#0B1118",outline=col)
            self.canvas.create_text(x,y-25*self.zoom,text=f"R{robot.id}",fill=self.ui["text"],font=("Consolas",9,"bold"))
            if robot.state in ("slowing","rerouting","blocked","offline"):
                ring=self.ui["amber"] if robot.state=="slowing" else self.ui["violet"] if robot.state=="rerouting" else self.ui["red"]
                self.canvas.create_oval(x-26,y-14,x+26,y+14,outline=ring,dash=(3,3),width=2)
            if robot.alive and not robot.comms:
                self.canvas.create_text(x,y-40,text="LINK LOST",fill=self.ui["red"],font=("Segoe UI",7,"bold"))
        self.canvas.create_text(14,self.canvas_dims()[1]-13,anchor="sw",text="drag to orbit  •  mouse wheel to zoom",fill="#66788E",font=("Consolas",8))

    def render_cards(self):
        """Update existing health widgets in-place; never rebuild the sidebar per tick."""
        if not hasattr(self, "_robot_ui"):
            self._robot_ui = {}

        online = sum(1 for r in self.model.robots if r.alive)
        self.robot_count_var.set(f"{online}/{len(self.model.robots)} online")

        for robot in self.model.robots:
            ui = self._robot_ui.get(robot.id)

            if ui is None:
                card = self._card(self.cards_frame, self.ui["surface2"])
                card.pack(fill="x", pady=5)

                row = tk.Frame(card, bg=self.ui["surface2"])
                row.pack(fill="x", padx=10, pady=(9, 5))

                rid = tk.Label(
                    row, text=f"R{robot.id}", bg=self.ui["surface2"],
                    fg=ROBOT_COLORS[robot.id],
                    font=("Consolas", 13, "bold")
                )
                rid.pack(side="left")

                state = tk.Label(
                    row, text="", bg=self.ui["surface2"],
                    font=("Segoe UI", 7, "bold"), padx=7, pady=3
                )
                state.pack(side="right")
                # Click the robot status to open a dedicated detail panel.
                state.configure(cursor="hand2")
                state.bind("<Button-1>", lambda e, rid=robot.id: self.show_robot_details(rid))

                info = tk.Frame(card, bg=self.ui["surface2"])
                info.pack(fill="x", padx=10, pady=(0, 7))

                task_label = tk.Label(
                    info, text="", bg=self.ui["surface2"],
                    fg=self.ui["text"], font=("Consolas", 8)
                )
                task_label.pack(anchor="w")

                link_label = tk.Label(
                    info, text="", bg=self.ui["surface2"],
                    fg=self.ui["muted"], font=("Segoe UI", 7)
                )
                link_label.pack(anchor="w", pady=(2, 0))

                battery = self._make_bar(card, "BATTERY")
                mission = self._make_bar(card, "MISSION")

                ui = self._robot_ui[robot.id] = {
                    "card": card, "state": state,
                    "task": task_label, "link": link_label,
                    "battery": battery, "mission": mission,
                }

            state_col = (
                self.ui["red"] if robot.state in ("blocked", "offline")
                else self.ui["violet"] if robot.state == "rerouting"
                else self.ui["amber"] if robot.state == "slowing"
                else self.ui["cyan"] if robot.state == "moving"
                else self.ui["muted"]
            )

            task = f"{robot.task_id}  →  {robot.goal[0]}" if robot.alive else "No active task"
            link = (
                "mesh linked" if robot.comms and robot.alive
                else "link lost" if robot.alive
                else "offline"
            )

            # Only configure text when it actually changed.
            if ui["state"].cget("text") != robot.state.upper():
                ui["state"].configure(text=robot.state.upper(), fg=state_col)
            elif ui["state"].cget("fg") != state_col:
                ui["state"].configure(fg=state_col)

            if ui["task"].cget("text") != task:
                ui["task"].configure(text=task)

            link_text = f"{link}  •  cell ({round(robot.c)},{round(robot.r)})"
            if ui["link"].cget("text") != link_text:
                ui["link"].configure(text=link_text)

            battery_col = (
                self.ui["green"] if robot.battery > 40
                else self.ui["amber"] if robot.battery > 15
                else self.ui["red"]
            )
            self._update_bar(ui["battery"], robot.battery, battery_col)

            prog = 0
            if robot.path and len(robot.path) > 1:
                prog = 100 * robot.path_idx / (len(robot.path) - 1)
            self._update_bar(ui["mission"], prog, ROBOT_COLORS[robot.id])

    def show_robot_details(self, robot_id):
        robot = next((r for r in self.model.robots if r.id == robot_id), None)
        if robot is None:
            return

        # Reuse the existing detail window if it is already open.
        if getattr(self, "_robot_detail", None) is not None and self._robot_detail.winfo_exists():
            win = self._robot_detail
            for child in win.winfo_children():
                child.destroy()
        else:
            win = tk.Toplevel(self)
            self._robot_detail = win
            win.protocol("WM_DELETE_WINDOW", win.destroy)

        win.title(f"Robot {robot.id} — Full Status")
        win.geometry("430x560")
        win.minsize(390, 500)
        win.configure(bg=self.ui["bg"])

        header = tk.Frame(win, bg=self.ui["surface"], padx=18, pady=14)
        header.pack(fill="x")
        tk.Label(header, text=f"ROBOT {robot.id}", bg=self.ui["surface"],
                 fg=ROBOT_COLORS[robot.id], font=("Segoe UI", 16, "bold")).pack(side="left")
        status_col = self.ui["green"] if robot.alive else self.ui["red"]
        tk.Label(header, text=robot.state.upper(), bg=self.ui["surface"], fg=status_col,
                 font=("Segoe UI", 9, "bold")).pack(side="right")

        body = tk.Frame(win, bg=self.ui["bg"], padx=16, pady=12)
        body.pack(fill="both", expand=True)

        details = [
            ("Status", robot.state.upper()),
            ("Operational", "ONLINE / ALIVE" if robot.alive else "FAILED / OFFLINE"),
            ("Communication", "MESH LINKED" if robot.comms and robot.alive else "LINK LOST" if robot.alive else "OFFLINE"),
            ("Task ID", robot.task_id),
            ("Task Priority", getattr(robot, "task_priority", TaskPriority.NORMAL).name),
            ("Priority Score", f"{getattr(robot, 'priority_score', 0.0):.1f}"),
            ("Goal", f"{robot.goal[0]}  ({robot.goal[1]}, {robot.goal[2]})"),
            ("Current position", f"({robot.c:.2f}, {robot.r:.2f})"),
            ("Grid cell", f"({round(robot.c)}, {round(robot.r)})"),
            ("Battery", f"{robot.battery:.1f}%"),
            ("Robot speed", f"{robot.speed:.4f}"),
            ("Mission progress", f"{robot.progress:.1f}%"),
            ("Path index", f"{robot.path_idx}"),
            ("Path length", f"{len(robot.path) if robot.path else 0} cells"),
            ("Steps taken", f"{robot.steps_taken}"),
            ("Blocked ticks", f"{robot.blocked_ticks}"),
            ("Conflict ticks", f"{robot.conflict_ticks}"),
            ("Waiting ticks", f"{getattr(robot, 'waiting_ticks', 0)}"),
            ("Temporary block", str(robot.temp_blocked) if robot.temp_blocked else "None"),
            ("Idle until", f"{robot.idle_until:.2f}"),
            ("Failed obstacle", "YES" if robot_id in self.model.failed_obstacles else "NO"),
        ]

        for label, value in details:
            row = tk.Frame(body, bg=self.ui["surface2"], padx=10, pady=7)
            row.pack(fill="x", pady=2)
            tk.Label(row, text=label, width=19, anchor="w", bg=self.ui["surface2"],
                     fg=self.ui["muted"], font=("Segoe UI", 8, "bold")).pack(side="left")
            tk.Label(row, text=value, anchor="e", bg=self.ui["surface2"],
                     fg=self.ui["text"], font=("Consolas", 8)).pack(side="right")

        tk.Label(body, text="Click the robot status again after conditions change to refresh this panel.",
                 bg=self.ui["bg"], fg=self.ui["muted"], font=("Segoe UI", 8),
                 wraplength=380, justify="left").pack(anchor="w", pady=(12, 0))
        win.lift()
        win.focus_force()

    def _make_bar(self, parent, label):
        row = tk.Frame(parent, bg=self.ui["surface2"])
        row.pack(fill="x", padx=10, pady=(0, 6))

        tk.Label(
            row, text=label, bg=self.ui["surface2"], fg=self.ui["muted"],
            font=("Segoe UI", 7, "bold"), width=8, anchor="w"
        ).pack(side="left")

        cv = tk.Canvas(
            row, width=130, height=6, bg="#091018",
            highlightthickness=0, bd=0
        )
        cv.pack(side="left", fill="x", expand=True, padx=6)

        fill_id = cv.create_rectangle(0, 0, 0, 6, fill=self.ui["green"], outline="")
        value = tk.Label(
            row, text="0%", bg=self.ui["surface2"],
            fg=self.ui["muted"], font=("Consolas", 7)
        )
        value.pack(side="right")

        return {"canvas": cv, "fill": fill_id, "value": value, "last": None, "last_color": None}

    def _update_bar(self, bar, pct, color):
        pct = max(0.0, min(100.0, float(pct)))
        rounded = round(pct)
        # Do not redraw the rectangle unless the visible value changed.
        if bar["last"] != rounded or bar["last_color"] != color:
            bar["canvas"].coords(bar["fill"], 0, 0, 130 * pct / 100.0, 6)
            bar["canvas"].itemconfigure(bar["fill"], fill=color)
            bar["value"].configure(text=f"{rounded:>3}%")
            bar["last"] = rounded
            bar["last_color"] = color

    def render_all(self):
        self.render_stage(); self.render_cards(); self.render_metrics(self.model.metrics)

    def loop(self):
        # Simulation advances continuously, but expensive Canvas rebuilding and
        # sidebar widget configuration are deliberately decoupled.
        self.model.step()

        # Map animation: stable ~20 FPS.
        self.render_stage()

        # Health/KPI panel: ~5 FPS, with persistent widgets and no destruction.
        if self.model.tick % 4 == 0:
            self.render_cards()
            self.render_metrics(self.model.metrics)

        self.after(50, self.loop)


if __name__ == "__main__":
    random.seed()
    app = FleetDashboard()
    app.mainloop()
