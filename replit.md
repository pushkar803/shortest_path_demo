# MechDog Route Planner Demo

## Overview
A Flask-based web application that computes multi-stop open routes on a predefined graph and simulates a robot ("MechDog") visiting locations and taking photos. Uses Held-Karp algorithm with turn/U-turn penalties for optimal pathfinding.

## Project Architecture
- **Language**: Python 3.12
- **Framework**: Flask
- **Frontend**: Inline HTML/CSS/JS served via `render_template_string` in `app.py`
- **Structure**: Single-file application (`app.py`) containing graph logic, route planning algorithms, Flask API endpoints, and embedded frontend

### Key Components
- Graph configuration (nodes, edges, weights)
- Dijkstra shortest path implementation
- Held-Karp open-path solver with U-turn penalties
- SVG-based route visualization with animation
- REST API (`/api/plan`, `/api/execute`)

## Running
- **Dev**: `python app.py` — runs Flask on `0.0.0.0:5000`
- **Production**: Same command via autoscale deployment

## Recent Changes
- 2026-02-10: Initial Replit setup — installed Python 3.12, Flask, configured workflow and deployment
