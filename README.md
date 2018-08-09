# ankle_project_sra

3_dof_a is Daniel's original design. I need to clean up the multibody model and also implement the kinematic control. Right now the motion input to the actuators is arbitrary.

3_dof_b is my first redesign on the original. It puts the side 2 legs facing each other. The idea was to decouple the roll and pitch, but that didn't quite happen. The motion output is not the 3 desired degrees of freedom, but rather some 3 degrees of freedom applied to some constraint manifold.

2_dof_a is an attempt at a simpler design with only 2 degrees of freedom and 2 motors. The problem is that there is a center support connecting a ball joint to the bottom of the platform. This constrains the platform to rotate about that point rather than about the ankle.

2_dof_b is the same as the previous design but has 4 motors, 2 for each direction, in order to reduce the load on each motor. This design still has the same problem with the point of rotation not at the ankle.

2_dof_c is the same as the previous device, but the platform has been modified so that the foot is lowered and the point of rotation is at the ankle.

