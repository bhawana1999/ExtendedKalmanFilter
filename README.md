# ExtendedKalmanFilter

<b>Why EKF? </b>

The two assumptions of KF are-:
1. Kalman Filter will always work with Gaussian Distribution.
2. Kalman Filter will always work with Linear Functions.

For example - an equation of a straight line is a linear function while a cos function is a non linear function.

Most real world problems involve non linear functions. In most cases, the system is looking into some direction and taking measurement in another direction. This involves angles and sine, cosine functions which are non linear functions which then lead to problems.

<b>Why non-linear functions create problem? </b>

 If you feed a Gaussian with a linear function then the output is also a Gaussian. If you feed a Gaussian with a Non linear function then the output is not a Gaussian. Non Linear functions lead to Non Gaussian Distributions. So if we apply a non linear function it will not end up as a Gaussian Distribution on which we can’t apply Kalman Filter anymore. Non linearity destroys the Gaussian and it does not makes sense to compute the mean and variances.
 
 <b> Solution </b>

They are non linear but we will make them Linear by approximation. Here, we will take help of a powerful tool called Taylor Series, which will help us to get a Linear Approximation of the Non Linear function. After applying the approximation what we get is an Extended Kalman Filter.We take a point and perform a bunch of derivatives on that point. In case of an EKF, we take mean of the Gaussian on the Non Linear Curve and perform a number of derivatives to approximate it.

<img src= "https://github.com/sona-19/ExtendedKalmanFilter/blob/master/images/taylor.png">

Suppose we want to approximate sin(x) at x=0.
Lets assume that we want to find a polynomial function P(x) = c_0 + c_1 * x + c_2*x² + c_3*x³ to approximate sin(x). So we need to find out the values for c_0, c_1, c_2 and c_3

At x=0, sin(x) = 0 , P(x) = c_0 + 0 + 0
If our approximation has to be even a little bit near to sin(x), then the value of sin(x) must be equal to value of P(x) at x=0. So, c_0 = 0

It will also be good if our approximation has the same tangent slope as that of sin(x) at x=0 
At x=0, d sin(x)/dx = cos(x) cos(0) = 1 
d P(x)/dx = c_1 + 2*c_2*x + 3*c_3*x² = c_1 + 0 + 0
If our approximation has to be precise then the value of derivative of P(x) must be equal to derivative of x at x=0. So c_1 = 1

Going on.. we can find that the approximation of sin(x) = x − x³/ 3! + x⁵/5! − x⁷/7! + x⁹/9! …

We are interested in linearizing, so we are just interested in the first derivative of Taylor series. For every non linear function, we just draw a tangent around the mean and try to approximate the function linearly.

EXAMPLE

Suppose we have two sensors LIDAR and RADAR. A LIDAR provides us the distance in the form of Cartesian coordinate system. On the other hand, a RADAR provides us the distance and velocity in Polar coordinate system.

Lidar => {px, py}
Radar =>{ ρ, Φ , ρ_dot}

px, py -> Coordinates of object in Cartesian System
ρ -> is the distance to the object
Φ -> is the counter clockwise angle between ρ and x- axis
ρ_dot -> is the change of ρ
The x-axis is always in the direction where the car is heading.

Taking data from different sensors and combining them together is called Sensor Fusion.

<b>Mathematical Model</b>

Prediction Step

x′ = F.x + B.μ + ν
P′ = FPFᵀ + Q
The prediction step is exactly the same as that of Kalman Filter. It does not matters whether the data is coming from LIDAR or RADAR the prediction step is exactly the same.
Update Step (Only in case of EKF i.e. Non Linear Measurements coming from RADAR)

    Equation 1:

    y= z - h(x′)

    z -> actual measurement in polar coordinates
    h -> function that specifies how our speed and position are mapped to polar coordinates
    x′ -> Predicted Value
    y -> Difference between Measured Value and Actual Value

h(x′)

This is a function that specifies the mapping between our predicted values in Cartesian coordinates and Polar coordinates. This mapping is required because we are predicting in Cartesian coordinates but our measurement (z) that is coming from the sensor is in Polar Coordinates.

<img src= "https://github.com/sona-19/ExtendedKalmanFilter/blob/master/images/cp.png">

    Equation 2:

    S= HⱼP′Hⱼᵀ + R
    K= P′HⱼᵀS⁻¹

    R -> Measurement Noise
    K -> Kalman Gain
    S-> Total Error
    S⁻¹ -> The inverse of S
    Hⱼ -> The Jacobian Matrix

Hⱼ

Hⱼ is the Jacobian Matrix. The Jacobian matrix is the first order derivative that we just discussed in Taylor Series. Since here we are dealing with matrices, we need to find differential in the form of a matrix.

    J_kl = d F_k / dX_l

    J_kl is the k,l element of the Jacobian matrix, F_k is the kth element of the vector function F, and X_l is the lth element of the vector variable X.

Here F_k = { ρ, Φ , ρ_dot}
X_l = {px,py,vx,vy}

Since in case of RADAR we have 4 measurements, 2 for distance and 2 for velocity.

<img src = "https://github.com/sona-19/ExtendedKalmanFilter/blob/master/images/Jacobian.png">


<img src = "https://github.com/sona-19/ExtendedKalmanFilter/blob/master/images/Jacobian2.jpeg">


    Equation 3:

    x = x′ + K.y
    P = (I- KHⱼ)P′
    
In case of a LIDAR we will apply a Kalman Filter because the measurements from the sensor are Linear. But in case of a Radar we need to apply Extended Kalman Filter because it includes angles that are non linear, hence we do an approximation of the non linear function using first derivative of Taylor series called Jacobian Matrix (Hⱼ) . Then we convert our cartesian space to polar space using h(x’) and finally we replace H with Hⱼ in all further equations of a KF.
