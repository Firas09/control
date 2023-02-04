# PID control system
This project aims to control the two wheeled balancing robot with a PID controller
## Mathematical Model
The concept is the same as the inverted pendulum Which being showed in the next figure:
![image](https://user-images.githubusercontent.com/108911160/216750073-073a9244-a1a4-4014-82b2-9d0f1ed47a40.png)


The mathematical equations are as follows:


![image](https://user-images.githubusercontent.com/108911160/216750349-6a8ee1c2-05d2-4262-ba3d-e64090c19d50.png)

We only going to use the equation for the x-axis the first one and the equation of the moment the third one

From the previous equations we get:

![image](https://user-images.githubusercontent.com/108911160/216750735-3bd4f355-b89d-452b-b33b-05b0d7849211.png)


We will reach the equilibrium point of the pendulum when the angle is 90 degrees
The equilibrium point is when θ=π. Assume that θ=π+φ(small angle). So, the equations become:


![image](https://user-images.githubusercontent.com/108911160/216750784-09c7bed0-1261-4a38-af15-dccda161b3eb.png)


And now we will need to find the Laplace transform for the previous equations,So we found it to be:

![image](https://user-images.githubusercontent.com/108911160/216750940-afec0cdb-c973-426c-9899-2a06e4941274.png)




For the next step all we need to do is substitute the following values:


<img width="470" alt="image" src="https://user-images.githubusercontent.com/108911160/216751267-a18990a3-b89e-40a5-983a-bebbefb6356c.png">

After substituting the parameters, the transfer function becomes:

![image](https://user-images.githubusercontent.com/108911160/216751341-b3fa4aa4-0463-487b-94a5-235c6a7cf1ef.png)

### The code:

```
%% Open Loop
clear;
clc;
Num = [14.286 0];
Den = [1 0.286 -70 -0.0294];
step(Num,Den);
grid on;
%% P Control 8
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 8;
sys = 90*feedback(sys*(kp),1);
step(sys);
grid on;
%% P Control 80
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 80;
sys = 90*feedback(sys*(kp),1);
step(sys);
grid on;
%% PI Control 8,8
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 8;
ki = 8;
sys = 90*feedback(sys*(kp+ki/s),1);
step(sys);
grid on;
%% PI Control 50,25
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 50;
ki = 25;
sys = 90*feedback(sys*(kp+ki/s),1);
step(sys);
grid on;
%% PD Control 8,2
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 8;
kd = 2;
sys = 90*feedback(sys*(kp+kd*s),1);
step(sys);
grid on;
%% PD Control 80,20
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 80;
kd = 20;
sys = 90*feedback(sys*(kp+kd*s),1);
step(sys);
grid on;
%% PID Control 8,8,2
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 8;
ki = 8;
kd = 2;
sys = 90*feedback(sys*(kp+ki/s+kd*s),1);
step(sys);
grid on;
%% PID Control 80,80,20
clear;
clc;
s = tf('s');
sys = 14.286*s/(s^3 + 0.286*s^2 - 70*s -0.0294);
kp = 80;
ki = 80;
kd = 20;
sys = 90*feedback(sys*(kp+ki/s+kd*s),1);
step(sys,10);
grid on;
```

The previous code consist of five parts:

the first part is when the system is an open loop situation

the second one if you want to use the P control : only Kp

the third part if we want to use the PI control:  Kp+Ki/s

the fourth part if you want to use the PD control :  Kp,Kd*s

the the last part is when we use the PID control: Kp+Ki/s+Kd*s

And we multiply the system feedback by 90 because as I mentioned earlier, the equilibrium point is at 90 degrees

[A YouTube reference](https://youtu.be/fv6dLTEvl74)











