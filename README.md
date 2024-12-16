# tethered-fly
Tethered Fly repository for ME 556 final project

## description
This project simulates the kinematics of a tethered, yaw-free *Drosophila melanogaster* (fruit fly) aligning itself with a visual simulus. In many fruit fly experiments, it is common to restrict the fly to a single degree of freedom: yaw, i.e. side-to-side, angular movement. The fly can be baited into rotating by flashing a light on a display within the fly's visual range. In this implementation, you can choose which direction the desired angle is.

## getting started
1. Clone the repository or download and extract the zip file under the "Code" tab

2. Open `tethered_fly.mlx` in MATLAB

3. Optional: Use the "Hide Code" view option to get an uncluttered view of the controls and plots 

4. Run the Initialization section. This can either be done by 
    - Selecting Live Editor > Run > Run
    - Selecting Live Editor > Section > Run Section
    - Using the slider bars to select a starting position and/or a desired position

5. Tune the sliders to change the parameters of each simulation. The plots will automatically update

6. Optional: Run the Save Data section. This will save the data displayed in both simulations. By default, the PD controller is saved as `PDTrajectory.csv` and the LQR is saved as `OptimalTrajectory.csv`

## notes and limitations
- The program does not prevent you from creating a situation where the desired position is outside of the fly's field of view

- The implementation of Elzinga's control model (see `generateDynamics.m`) models sensor delays as a first-order Pade approximation and that one wingbeat = 5 milliseconds

- The program does not prevent you creating a control policy that corresponds to unrealistical wing asymmetry angles that either break the physics model or are beyond the fly's physical limit. This is simple enough to control for the LQR simulation: just increase the $R$ parameter so that the maximum control corresponds to a physically valid condition.