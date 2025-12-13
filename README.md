# AIFinalProject

### Team Members

Excel Olatunji
Eric Van Zant

## Files

### Agent

Contains the implementation for our pathfinding agents.

### Area

Contains the implementation for collision capable objects in the environment.

### Benchmarker

Ran to benchmark the Agent types, does not render.

### FlowField

Contains Flow Field implementation.

### Group

Contains Group Agent Resevervation System.

### HAstar

Contains implementation of Hierarchical A star search.

### Main

Ran to test the Agents and Environments, renders through pygame.

### Pathfinder

Simple queue for Deffered Agents.

### Problem

Based on provided Problem Class, Contains ContinuousNavigation Problem.

### Search

Provided search file from informed search lab, contains a* implementation.

### Util

Contains common functions for working with vectors as tuples.

## Running

### Benchmarker

Run python file, manually change values in main function
values in main function are a list of testing parameters
agent_types will work with any in ["Simple", "Flow", "HAstar", "Group", "Astar"]
agent_counts will work with any int greater or equal to 0
dists will probably break with values below 100
densities will cause overlapping at 25


### Main

Run python file, manual change values in gameloop function or use command line arguments to control settings.

Settings

Agent type: Type of agent valid values are "Deffered", "Flow", "HAstar", and "Group"

Agent count: Number of agents to simulate, any int greater than 0 is allowed

Dist/Scale: X size of maze region, y scales with at half, any int greater than 99 is allowed

Density: Density distance to place collision objects from eachother's origin, any int greater than 29 is allowed

## Output

Benchmarker outputs directly to sysout

