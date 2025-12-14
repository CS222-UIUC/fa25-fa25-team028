Physics Simulation Sandbox

Introduction:
Project Description: An open-source, Python-based physics engine designed to provide a visual sandbox environment for users to intuitively grasp fundamental Newtonian mechanics and elastic systems. We utilize Pygame for frontend rendering and NumPy and SciPy for core physics computation.
Core Features: Object collision simulation, spring system simulation, and modifiable world parameters (gravity, drag).


Technical Architecture:
Core Engine (world.py, integrator.py, body.py, spring.py, collisions.py): They are responsible for all physics calculations (force analysis, integration, collision detection/response). They are the "brain" of the system.
User Interface (main.py, ul_controls.py): These are responsible for rendering physical objects and handling user input (keyboard/mouse events). They are the "eyes and hands" of the system.
Testing Module (tests/): We use the Pytest framework to conduct regression and unit tests on the Core Engine's physics calculations, verifying conservation laws for energy and momentum.

Major Dependencies: pygame, numpy, scipy, pytest.

Installation Instructions:
Requirements: Python 3.11
Clone Repository: git clone https://github.com/CS222-UIUC/fa25-fa25-team028.git
cd [your-repo-name]
pip install -r requirements.txt
pip install numpy
python3 src/engine/demos/physics_sandbox_showcase.py (demo1)



Group members and responsibilities:
Christian Bourquin (cb77): Spring system + chain
Joey Wang (joeyw3): UI + Collision system
Zonghang Wu (zwu63): Body + Integrator
All members contributed to the demo
