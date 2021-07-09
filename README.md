# Inverted Pendulum on a Cart Control Strategies
## About
This project compares different linear control strategies for controlling the full state of an inverted pendulum on a cart.  The cart has an ideal position sensor which gives the position of the cart and an angle sensor which gives the angle of the pendulum.  The cart is controlled via a horizontal force applied to the side of the cart.  The figure below shows a free body diagram of the cart.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/inverted_pendulum_fbd.png)

## Equations of Motion (EOM)
The EOM for this system were developed using the Lagrangian Energy Method, which balances the kinectic and potential energy of the system.  Using this method, the non-linear dynamics are derived as  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/Second_Order_Nonlinear_EOM.png)  
  
These equations can be rewritten as a series of first order differential equations, shown below, for use in non-linear simulations.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/First_Order_Nonlinear_EOM.png)  
  
To linearize the model, the small angles assumption (stated below) is used to derive a linear second order system and associated state space model.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/Small_Angles_Assumption.png)  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/Second_Order_Linear_EOM.png)  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/State_Space_Equations.png)  

The free response of the cart (with no input actuation) is simulated to verify the EOM were developed correctly.
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/inverted-pendulum-free-response.gif)

## Control Strategies

### Pole Placement Controller

### Linear Quadratic Regulator (LQR) Controller

## Simulations
The following simulations were run on both control strategy implementations:  
  
step position input  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Position_Step_Response.gif)  
  
step velocity input  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Velocity_Step_Response.gif)  
  
ramp position input   
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Position_Ramp_Response.gif)  
  
cart velocity disturbance  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Cart_Velocity_Disturbance.gif)  
  
pendulum velocity disturbanc     
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Pendulum_Velocity_Disturbance.gif)  
