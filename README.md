# MechDog — Short Route Planner Demo

Small demo app that computes multi-stop open routes on a predefined graph and simulates a robot ("MechDog") visiting locations and taking photos.

Features
- Visual route planner UI (Flask + client-side SVG)
- Held–Karp open-path planner with turn/UTurn penalties
- Simulated execution (robot stubs) that returns photos and updates start node

Quickstart

1. Create a virtual environment and install deps:

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

2. Run the app:

```bash
export FLASK_APP=app.py
python app.py
# or: flask run --host=0.0.0.0 --port=5000
```

3. Open http://localhost:5000 in your browser.

Notes
- The UI shows a `Start` selector. The app now keeps the selected default start node rendered as the last option and, after planning or execution, updates the Start selector so the last destination becomes the new start.
- Execution is simulated — replace `robot_go_to` and `robot_take_photo` in `app.py` with real MechDog motion/camera code when integrating hardware.

License
- MIT

