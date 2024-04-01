# Inverted Double Pendulum
The project aims to ...

## Simulation
### Setup

## MjBot Implementation
### Requirements
- 2x moteus r4.11 developer kit
- 1x Raspberry Pi 4B (Running Preempt-RT Kernel (Real-Time))
- 2x Custom Arm Links
- 1x Estop (Optional but Strongly Recommended)

### Setup
#### One Time Setup
- Install [robotpkg/pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)
- Patch RPI kernel using RT patch found [here](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/6.6/patch-6.6.23-rt28.patch.gz) by loosely following this [guide](https://www.instructables.com/64bit-RT-Kernel-Compilation-for-Raspberry-Pi-4B-/)

#### Compile and Build Project
Please run the following in the ~/git/Inverted-Double-Pendulum directory:
```
mkdir build
cd build
cmake ..
make
```

### Using
Please run the following in the ~/git/Inverted-Double-Pendulum directory:
```
./build/EXECUTATBLE-NAME # E.g. ./build/IDPDriver
```

## Useful Commands
Activate moteus Virtual Environment
``` 
source moteus-venv/bin/activate
```

View/Configure both mjbot motors at once (Can only be used in moteus venv unless installed globally)
```
tview -t 1,2
```

Reset Motor Position (Recommended use before running script)
```
moteus_tool -t 1,2 --zero-offset
```
