## CarND-Term2 Extended Kalman Filter
In this project we apply an Extended Kalman Filter to estimate the position and velocity of a moving vehicle in 2 dimensions. 

The sensor readings are composed of radar and lidar data, the radar measurements give us a nonlinear sensor model when estimating position and speed, while the lidar gives us a linear sensor estimate for position. 

The system model is assumed to be a linear cartesian speed and position model and the acceleration is lumped with the process noise. 

Based on which type of measurement is received, either a linear Kalman Filter is applied for a radar measurement or a nonlinear Extended Kalman Filter is for a lidar measurement.
 
 The below figure shows the measurement model as provided in the course material. 
 
![Sensor Fusion FLow](png/sensorfusionflow.png)

$$ equation $$
