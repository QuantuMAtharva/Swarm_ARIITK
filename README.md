# Obastacle Avoidance using Swarm Robotics

> ## Intra (General) Repulsion among drones when they are close to each other (when the formation is disturbed):

<!-- ![General Repulsion](https://media.discordapp.net/attachments/892774933107707904/892775008294826054/general_repulsion.gif) -->

<p align="center">
    <img src="https://media.discordapp.net/attachments/892774933107707904/892775008294826054/general_repulsion.gif" />
    <i>When one of the drones is displaced a bit, it comes back to its original position in the formation while avoiding other drones that come along its way</i>
</p>

<br>

> ## Vertical Repulsion of drone formation to avoid collision when one fast moving external drone approaches them:

<!-- ![Vertical Repulsion](https://media.discordapp.net/attachments/892774933107707904/892775015731314778/vertical_repulsion.gif) -->

<p align="center">
    <img src="https://media.discordapp.net/attachments/892774933107707904/892775015731314778/vertical_repulsion.gif" />
    <i>The drones in formation rise up when another fast moving drone approaches them, to avoid a collision, and finally come back to their proper positions</i>
</p>

# Basic Setup
## Setting up [rotors_simulator](https://github.com/ethz-asl/rotors_simulator) and [mav_control_rw](https://github.com/ethz-asl/mav_control_rw) in your system

Comment out the lines for PX-4 Autopilot from the .bashrc file if you have it installed already

