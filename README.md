# PAMI

This repository aims at creating a robot control code for a differential wheeled robot. It will also contain a visualisation part using SDL2. The robot to be controled will be used for the [2025 French Robotics Cup](https://www.coupederobotique.fr/edition-2025/).

# Dependencies

You will need to download the SDL2 dependencies : 
```bash
apt install libsdl2-image-dev libsdl2-dev
```

# Build & Launch

```bash
cmake -S . -B build -DENABLE_DISPLAY_DEBUGGER=ON
cmake --build build
./build/pami # Main binary
./build/pami_display # Main binary with visualizer
./build/visual_debugger # Displayer debugger
```