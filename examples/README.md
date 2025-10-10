# Dyros Robot Controller examples in  C++ and Python

This directory contains minimal examples demonstrating how to use **Dyros Robot Controller**, either directly in **C++** or through its **Python bindings**.
Some examples require the installation of the **MuJoCo** simulator.

Installing **MuJoCo**:
```bash
git clone https://github.com/deepmind/mujoco.git
mkdir build && cd build
cmake ..
cmake --build .
cmake --install .
```

Installing **MuJoCo-py**:
```bash
pip3 install mujoco
```

## Build C++ examples
```bash
cd C++
mkdir build && cd build
cmake ..
make -j10
```

## Run the Examples
The example codes include two representative robot types:
- 7-DoF torque control Manipulator: [Franka FR3](https://franka.de/franka-research-3)
- Mecanum Wheel based Mobile Robot: [Summit XL Steel](https://robotnik.eu/products/mobile-robots/rb-kairos-2/)

Run Python Example:
```bash
cd python
python3 dyros_robot_controller_example.py --robot_name fr3 # or xls
```

Run C++ Example:
```bash
cd C++
./dyros_robot_controller_example fr3 # or xls
```
