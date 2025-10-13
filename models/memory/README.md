# GVSoC Phase-Change Memory (PCM) Model with Analog In-Memory Computing (AIMC)

*Last Updated: 2025-08-29*
*Author: [domenikali](https://github.com/domenikali)*

---

## 1. About The Project

This project provides a sophisticated C++ simulation model of **Phase-Change Memory (PCM)** designed for the **GVSoC**. The model's primary feature is its support for **Analog In-Memory Computing (AIMC)**, a revolutionary technique that performs computations directly within the memory array.

The core objective is to accelerate Artificial Intelligence (AI) and Machine Learning (ML) workloads by overcoming the traditional von Neumann bottleneck, where data must constantly move between memory and processing units. By simulating Matrix-Vector Multiplication (MVM), a fundamental operation in neural networks, this model serves as a powerful tool for research and development in next-generation AI hardware accelerators.

## 2. Key Features

*   **AIMC Simulation**: Faithfully models Matrix-Vector Multiplication (MVM) directly within the PCM array.
*   **GVSoC Integration**: Designed as a standard GVSoC component, allowing for seamless integration into PULP-based architectures.
*   **High-Performance Algorithm**:
    *   **Optimized Memory Layout**: Utilizes a flattened 1D buffer to represent weight matrices, significantly reducing pointer overhead and improving memory locality compared to naive multi-dimensional array implementations.
    *   **Improved Performance**: Leverages multithreading to fasten MVM operations. The model is configured for 16 threads based on empirical analysis for modern multi-core CPUs aviable in consumer computers [Tests](https://github.com/domenikali/TesiTest/tree/main/matrix_comp_tests/flat_weight_core_20).
*   **Configurability**: The Python bindings allow for easy modification of model parameters, such as memory size and timing characteristics, directly from the simulation script.

## 3. Project Structure

The model is contained within this folder and follows standard GVSoC component design patterns:

*    `pcm_mem.hpp`: The C++ header file defining the `Pcm` class, its interfaces (I/O ports), and internal state variables. The majority of the documentations is provided in the header.
*    `pcm_mem.cpp`: The C++ implementation file containing the core logic for the PCM model, including memory access handlers and the AIMC algorithm.
*    `pcm_mem.py`: The Python bindings for the C++ model. This file makes the component available to the GVSoC configuration system and defines how it can be instantiated and configured in a virtual platform.

## 4. Technical Implementation Details

### MVM Algorithm
The core of the AIMC functionality is a highly optimized Matrix-Vector Multiplication algorithm. It simulates the analog properties of a PCM crossbar array to perform multiplications and accumulations.

### Memory Layout Optimization
To achieve maximum simulation speed, the neural network weights are not stored in a complex, multi-dimensional structure. Instead, they are flattened into a contiguous 1D array. This approach offers two key advantages:
1.  **Reduced Overhead**: Eliminates the need for multiple pointer dereferences to access a single weight, reducing cache misses.
2.  **Improved Locality**: Ensures that weights are stored contiguously in memory, allowing for faster, sequential access patterns during the MVM computation.

## 5. Getting Started

### OS Requirements Installation 

To install the required packages, run:

```bash
sudo apt-get install -y build-essential git doxygen python3-pip libsdl2-dev curl cmake gtkwave libsndfile1-dev rsync autoconf automake texinfo libtool pkg-config libsdl2-ttf-dev
```

### Toolchain and Shell Requirements 

GVSoC requires the following tools and versions:

- **g++** and **gcc** versions >= 11.2.0
- **cmake** version >= 3.18.1
- **Python** version >= 3.11.3

### To run the tests from scratch

1. **Clone the repository** and navigate into the project directory:

   ```bash
   git clone https://github.com/domenikali/gvsoc.git -b feature/SHR_PCM_module SHR_PCM_module
   cd SHR_PCM_module
   ```

2. **Initialize the simulator environment** by running:

   ```bash
   source sourceme.sh
   ```

3. **Make sure the branches are correct**
    ```bash
    $ git branch 
    > feature/SHR_PCM_module
    $ cd core
    /core $ git branch
          > SHR_PCM_module
    ```
4. **Compiler** 

    Make sure to have the `gcc-riscv64-unknown-elf` cross-compiler:
    ```bash
    sudo apt-get install gcc-riscv64-unknown-elf
    ```
5. **Run** the tests

    Some simple tests are written within the `/docs/developer/tutorials/1_how_to_write_a_component_from_scratch`

    To run them simply move to the tests direcctory then:
    ```bash
    make gvsoc
    make all run runner_args="--trace=pcm"
    ```

    The tests are written in the `main.c` file.

### Integration
To use this model, it must be integrated into a GVSoC simulation script (e.g., `mysystem.py` in the developer tutorials).

1.  **Instantiate the Model**: Add the PCM component to your system.
    ```python
    # In your system configuration script
    import memory.pcm_mem

    # ... inside your system class
    pcm = memory.pcm_mem.Pcm(self, 'pcm')
    ```

2.  **Map the Memory**: Connect the model to the system's main interconnect and map it to an address range.
    ```python
    # Map the PCM component into the system's address space
    ico.o_MAP(pcm.i_INPUT(), 'pcm', base=0x20001000, size=0x00001000, rm_base=True)
    ```

## 6. Current Status & Future Work

This model is an active project. Current and future efforts are focused on:
*   **Driver Development**: An ongoing effort to create robust drivers within the PULP-SDK to provide a clean software API for the PCM's AIMC capabilities.
*   **Comprehensive Testing**: Developing a full suite of unit and integration tests to validate the model's functionality and performance against theoretical results.

The testing and driver work can be tracked in the `tutorial` section within GVSoC and on the associated **PULP-SDK repository**.