# Template: ee483-psu-ex-base

This template provides a boilerplate repository for developing ROS-based software in Duckietown. Use it for PSU EE 483.

## How to use it
Create a new repository on github.com while specifying the newly forked template repository as a template for your new repository.

## Setting up your environment
Ex0-Setup has the necessary information for you to set up your environment, i.e., create a VM with Ubuntu, install docker, install the duckietown-shell, and clone this template repo.
**Ex0-Setup** instructions must be followed before starting the next exercises.

## Instruction for each exercise
Each exercise has a set of Jupyter notebooks with detailed instructions about the tasks to be completed.
The notebooks are inside the Exercise folder `ex<number>/notebooks`
Open the notebooks using in the vs code container.
To open the container, you need to be inside an exercise folder.
``` bash
dts code editor --recipe ../ex<number>-recipe
```

## Place your code
Place your code in the correct Exercise you are developing inside the directory `ex<number>/ex_workspace/src` of your new repository.
