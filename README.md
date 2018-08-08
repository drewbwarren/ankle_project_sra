# ankle_project_sra

3_dof_a is Daniel's original design. I need to clean up the multibody model and also implement the kinematic control. Right now the motion input to the actuators is arbitrary.

3_dof_b is my first redesign on the original. It puts the side 2 legs facing each other. The idea was to decouple the roll and pitch, but that didn't quite happen. The motion output is not the 3 desired degrees of freedom, but rather some 3 degrees of freedom applied to some constraint manifold.

