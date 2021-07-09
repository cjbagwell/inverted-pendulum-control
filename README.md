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
The two control strategies implemented in this project, pole placement and linear quadratic regulator, both use the full state feedback architecture.
A full state feedback system, shown below, calculates the required input actuation based on the entire state of the system.  This control structure is tuned by changing the values of the gain matrix K.  This affects the response of the cart by changing the poles of the system with input actuation.  The only difference between the two controllers in this project is how this gain matrix is calculated.
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/Simulink_Model.png)  

### Pole Placement Controller  
The first controller designed for the system uses the pole placement method and the following ITAE (Integral of Time times Absolute value of Error) optimization equation.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/ITAE_Optimization_Equation.png)  
The natural frequency is obtained through the poles of the open loop system.  Solving this equation then gives the desired poles of the controlled system.  The gain of the K matrix is then found using the "place()" function provided in MATLAB.

### Linear Quadratic Regulator (LQR) Controller
The second control method explored in the current project implements a linear-quadratic regulator (LQR). LQR controllers are formed by minimizing a cost function, 𝐽 (𝑥), by iteratively solving the Riccati equation (shown below) for 𝑃, given matrices 𝑄 and 𝑅.
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/QR_Cost_Function.png)  
Here, 𝑄 represents the weight of each state in the cost function and 𝑅 is the weights for the system input. The gains for the controller are then found by solving for 𝐾 using the equation below.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/QR_Gain_Equation.png)  

## Simulations
The following simulations were run on both control strategy implementations:  
  
### step position input  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Position_Step_Response.gif)  
  
### step velocity input  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Velocity_Step_Response.gif)  
  
### ramp position input   
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Position_Ramp_Response.gif)  
  
### cart velocity disturbance  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Cart_Velocity_Disturbance.gif)  
  
### pendulum velocity disturbanc     
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/PP_vs_QR_Pendulum_Velocity_Disturbance.gif)  
