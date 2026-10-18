# NPRN Sim: Neuro-Plastic Reflex Network on a Wheeled Inverted Pendulum

A C++ / [Dear ImGui](https://github.com/ocornut/imgui) simulator of the **Neuro-Plastic Reflex Network (NPRN)**: a variable-size bank of tiny recurrent neurons that balances a wheeled inverted pendulum whose leg length and body mass change without notice.

Each neuron holds a fixed two-unit reservoir, a learned one-step-ahead predictor and a learned three-weight control readout. A hard winner-take-all picks the neuron that currently predicts the observation best, and only that neuron's readout produces the control torque. The network sees only a noisy two-dimensional tracking-error signal; it never observes leg length, mass, position, velocity or the gust force.

<img width="1443" height="903" alt="Screenshot from 2026-10-17 04-18-56" src="https://github.com/user-attachments/assets/abcce9cf-1925-4541-ab6e-e1dab5833850" />

## How it works (short version)

- **One state, two outputs.** The winner's hidden state both predicts the next observation and, through a separate readout, gives the control torque. There is no separate policy network.
- **Prediction error as regime estimate.** The winner index acts as an online regime estimate, and the winner's readout acts as a regime-specific gain set (multiple-model adaptive control with a learned model bank).
- **Learning.** Predictors are trained with normalised LMS under a gating rule that protects well-fitted neurons from being overwritten. Control readouts are trained with a sign-prior NLMS rule that needs no plant model.
- **Structural plasticity.** Neurons are spawned on sustained prediction failure (with a warm-started controller), and pruned or merged when unused or redundant. At most 40 neurons.
- **Plant.** A wheeled inverted pendulum with an Ornstein-Uhlenbeck gust force, a torque limit, an optional PD safety fallback, and four hidden morphologies visited cyclically in the order 0, 1, 2, 0, 1, 3, 2, 0, 3, 1.

## Repository layout

| File | Contents |
|---|---|
| `sim.hpp` | The simulation: plant physics, neuron bank, learning rules, structural plasticity, visit bookkeeping, text report. No GUI dependencies. |
| `main.cpp` | The Dear ImGui front end (GLFW + OpenGL3): robot view, phase plot, error plot, neuron bank view, controls and logs. |
| `report.pdf` | Goes through the maths in depth |

## Building

Requirements:

- A C++17 compiler
- [GLFW 3](https://www.glfw.org/)
- OpenGL
- Dear ImGui (recent enough for `ImGui::SeparatorText` and `ImGuiChildFlags_Borders`, i.e. 1.91 or newer), with its GLFW and OpenGL3 backends

`main.cpp` includes `imgui.h`, `imgui_impl_glfw.h` and `imgui_impl_opengl3.h`, so the ImGui sources and those two backends must be compiled and on the include path. A typical Linux command, assuming ImGui is checked out in `imgui/`:

```bash
g++ -std=c++17 -O2 main.cpp \
    imgui/imgui*.cpp \
    imgui/backends/imgui_impl_glfw.cpp \
    imgui/backends/imgui_impl_opengl3.cpp \
    -Iimgui -Iimgui/backends \
    $(pkg-config --cflags --libs glfw3) -lGL \
    -o nprn_sim
./nprn_sim
```

Adjust the OpenGL link flag for your platform (for example `-framework OpenGL` on macOS or `-lopengl32` on Windows).

## Using the simulator

**Buttons**

| Control | Effect |
|---|---|
| Reset | Re-initialise the simulation |
| Pause / Resume | Pause the simulation |
| Force next morphology | Jump immediately to the next morphology in the schedule |
| Freeze / Unfreeze structure | Disable (or re-enable) spawn, prune and merge; learning continues |
| Push robot | Apply a random-sign angular-velocity impulse |
| PD fallback ON/OFF | Toggle the fixed PD safety fallback on the true pitch error |
| Network ON/OFF | Stop or resume the network. Stopping it also drops the PD fallback |
| Rocking ON/OFF | Add a sinusoidal term to the lean reference |

**Drive pad.** Drag horizontally to drive left or right; the pad springs back to centre on release. Dragging vertically switches the leg height to manual mode, overriding the morphology schedule's height target.

**Sliders**

| Parameter | Range | Default |
|---|---|---|
| spawn / learn threshold | 0.02 to 0.3 | 0.1 |
| morphology length (steps) | 300 to 3000 | 1200 |
| steps per frame | 1 to 8 | 2 |
| control learning rate | 0.02 to 1.5 | 0.3 |

The Viewport window shows the robot, a phase plot, the prediction-error history, the neuron bank, and a table of morphology visits (adaptation error, steady-state error, spawns, dominant neuron, and whether a neuron was reused). The Logs window lists spawn, prune, merge, fall and push events.

## Report

`Sim::report()` in `sim.hpp` produces a plain-text report of the run: parameters, morphologies, per-visit statistics, per-neuron statistics and the event log.

## Citation

If you refer to this work, please cite the report in this repository (`nprn.pdf`).
