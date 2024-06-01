# Inverted Double Pendulum
The project aims to create a functioning Inverted Double Pendulum that employ the use of 2x Moteus r4.11 motors to actuate the pendulum.
A Simulink Simulation is developed to ensure the functionality and possibility of the Inverted Double pendulum can be run. 


## Simulation
### Setup

- Install MATLAB Add Ons [Simulink](https://au.mathworks.com/help/install/install-products.html)
- Install MATLAB Add Ons [Simscape Multibody](https://au.mathworks.com/campaigns/offerings/download_smlink.html)

#### Install Real-Time Kernal
Please run the following in the MATLAB Command Window (Restart required after executing):
```
sldrtkernel -install
```
Open Simulink file DoublePendulum.slx.
Run the Simulink file and launch visualiser in Matlab

## MjBot Implementation
### Requirements
- 2x moteus r4.11 developer kit
- 1x Raspberry Pi 4B (Running Preempt-RT Kernel (Real-Time))
- 2x Custom Arm Links
- 1x Estop (Optional but Strongly Recommended)

### Setup
#### One Time Setup
- Install [robotpkg](http://robotpkg.openrobots.org/install.html)
- Install [robotpkg/pinocchio](https://stack-of-tasks.github.io/pinocchio/download.html)
- Patch RPI kernel using RT patch found [here](https://mirrors.edge.kernel.org/pub/linux/kernel/projects/rt/6.6//older/patch-6.6.23-rt28.patch.gz) by loosely following this [guide](https://www.instructables.com/64bit-RT-Kernel-Compilation-for-Raspberry-Pi-4B-/)
- Install the following cactus_rt dependancies 
```
sudo apt install build-essential cmake protobuf-compiler libprotobuf-dev libgtest-dev libbenchmark-dev
```

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

## Latency Test of Standard Kernel compared to Preempt-RT Kernel
### Standard Kernel 6.6.20+rpt-rpi-v8
![alt text](https://github.com/Orochi13479/Inverted-Double-Pendulum/blob/main/Misc/Standard%20Latency.png?raw=true)

### Preempt-RT patched Kernel 6.6.23-rt28-v8+
![alt text](https://github.com/Orochi13479/Inverted-Double-Pendulum/blob/main/Misc/RT%20Latency.png?raw=true)


## Moteus Tview Position Control command
Use following command to simulate motion using position control instead of torque control.
```
1>d pos 0 0 nan a1 && 2>d pos 0 0 nan a1 && ? && 
1>d pos -0.2 -0.05 nan a2 && 2>d pos -0.05 -0.05 nan a2 && ? && 
1>d pos 0.1 0.25 nan a3 && 2>d pos 0.1 -0.05 nan a3 && ? && 
1>d pos 0.5 0 nan a1.5 && 2>d pos 0 0 nan a1.5
```