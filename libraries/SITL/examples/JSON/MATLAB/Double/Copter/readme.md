Compared to the original MATLAB code, this MATLAB backend for the JSON SITL interface connects to two UDP streams simultaneously and simulates a very simplified vehicle and a more detailed quadcopter. This code is to be used in conjuction with the **follow-copter-autotest.sh** launcher (which runs the **run_auto_follow.py** script) to simulate a quadcopter following and landing on a moving vehicle.

In **run_double_simulation.m**, the UDP ports of the both incoming AP streams are specified (9002 for the 1st SITL instance (quadcopter/drone/follower) and 9012 for the 2nd SITL instance (vehicle/target/leader)). The starting heading can also be specified, as it is not included in the incoming streams. The specified headings don't need to match the ones specified in **follow-copter-autotest.sh** / **run_auto_follow.py**.

Instead of the original **SITL_connector.m** script twice in parallel, the custom **double_SITL_connector.m** connects to both UDP streams in order to keep both simulations synchronized.

The vehicle model is heavily simplified. It receives a velocity target as an incoming servo command from the SITL instance, a simple PD controller provides an acceleration target and a simple PD controller then provides a jerk target. There are then acceleration and jerk limits (for both acceleration and deceleration), resulting in the actual jerk. Acceleration, velocity and position are then obtained through basic timestep intergration. The code for ground interaction from the original code is used.

The quadcopter model **Quad3D_Algebraic.m** is generated with Motion Genesis (code included). It includes a dynamic motor model. The forces applied to the quadcopter body are:
- Gravity,
- 4x rotor thrust,
- 4x motor torque (a combination of the rotor aero torque and the torque required to accelerate the motor/prop),
- Drag.

Other considerations include:
- Wind can be specified for each timestep (direction and velocity),
- The surface area of the quadcopter is dependant on its tilt,
- Rotor thrust is reduced as the quadcopter increases airspeed.

The quadcopter model's physical parameters match those of our 450mm quadcopter, equiped with BH Tornado T5 Pro 3115 1050kv motors, 9x5x3 HQprops and 4Ah 6S battery. The high moments of inertia are due to the custom landing installed on the quad. The motor parameters are obtained through testing with an RCBenchmark thrust stand. The surface area (equivalent flat plate area) of the drone and the thrust loss were measured at different high speeds of flight (40-120 km/h).

The AP control parameters (ATC and PSC) used in the SITL and specified by **follow-copter-autotest.sh** match the parameters used on our quadcopter.


