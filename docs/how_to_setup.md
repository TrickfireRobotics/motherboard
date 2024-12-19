# How to Set Up Your Workspace

The motherboard code uses two submodules in order to be able to write code. The first one is the `pico-sdk` which includes all of the interfaces needed to interact with the Pi Pico, and the `FreeRTOS-Kernel` which allows us to use FreeRTOS within the Pi Pico.

## Step 1
Clone the repository in a Linux-based computer. If on Windows, use WSL.

`git clone https://github.com/TrickfireRobotics/motherboard.git`

## Step 2
Execute the following command while in the project folder

`git submodule update --init --progress`

## Step 3
Go into the `pico-sdk` directory and execute the following command.

`git submodule update --init --progress`

Note: Steps 2 and 3 will take a while. Grab a cup of tea.

## Step 4
Now you need to set up your CMake. Execute the following command in the project folder.

`./setup_cmake.sh`

Note: If you cannot execute this shell file, mostly likely it does not have executable permissions. Execute `chmod 777 ./setup_cmake.sh` and try again.

## Step 5
Compile the code by running the following shell file. You have to add executable permissions as seen in step 4. 

`./build.sh`