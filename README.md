# Inverted Pendulum on a Cart Control Strategies
## About
This project compares different linear control strategies for controlling the full state of an inverted pendulum on a cart.  The cart has an ideal position sensor which gives the position of the cart and an angle sensor which gives the angle of the pendulum.  The cart is controlled via a horizontal force applied to the side of the cart.  The figure below shows a free body diagram of the cart.  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/inverted_pendulum_fbd.png)

## Equations of Motion (EOM)
The EOM for this system were developed using the Lagrangian Energy Method, which balances the kinectic and potential energy of the system.  Using this method, the non-linear dynamics are derived as  
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/Second_Order_Nonlinear_EOM.png)

The free response of the cart (with no input actuation) is simulated to verify the EOM were developed correctly.
![alt text](https://github.com/cjbagwell/inverted-pendulum-control/blob/main/media/inverted-pendulum-free-response.gif)

### Pole Placement Controller

### Linear Quadratic Regulator (LQR) Controller

## Simulations
