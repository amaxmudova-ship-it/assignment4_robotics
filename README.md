## **1. Controller**

The goal of Lab 4 was to modify the Lab 3 line-following controller so that the robot could follow the track at full speed. The main difference from Lab 3 is that the robot is no longer allowed to use a reduced base_speed. At every control cycle, one wheel must operate at 100% PWM, while the controller can only slow down the other wheel to steer the robot.

The controller uses the two existing line sensors:

A4 — left sensor
A3 — right sensor

The controller is based on a PID controller.

## **2. PID Controller**

The robot uses a PID controller to calculate the steering correction from the line-following error. The proportional term responds to the current error, the integral term accumulates persistent error, and the derivative term responds to how quickly the error changes.

The proportional component is:

$$ P=K_p e $$

The derivative component is:

$$ D=K_d\frac{e-e_{previous}}{\Delta t} $$

The integral component is:

$$ I=K_i​{\Sigma}e{\Delta}t $$

The complete controller is:

$$ \boxed{ u=K_p e+ K_d\frac{e-e_{previous}}{\Delta t} + K_i​{\Sigma}e{\Delta}t} $$

The initial gains were \(K_p=1.5\), \(K_i=0.005\), and \(K_d=0.8\), with a fixed control period of \(dt=0.03s\). The integral accumulator was limited to prevent integral wind-up, and the final controller output was clamped to the range \([-200,200]\).

The integral term was retained from the Lab 3 PID controller because it helps compensate for persistent small errors caused by sensor differences or mechanical imbalance. However, the integral contribution was kept small because excessive accumulation at full speed could cause overshoot and make sharp turns less stable.

## **3. Controller Output Limitation**

The controller output is limited to:

$$ -200\leq u\leq200 $$

This is different from the previous Lab 3 controller, where the steering output was limited to approximately ±50.

The larger range is necessary because the Lab 4 motor model allows much stronger steering corrections.

For example:

$$ u=0 $$

means that no steering correction is needed.

At:

$$ u=100 $$

one wheel can be stopped.

At:

$$ u=200 $$

the inner wheel can be driven at -100%, allowing the robot to make an extremely sharp correction.

## **4.Full-Speed Motor Control**

The most important change from Lab 3 is the motor mixing.

In Lab 3, the controller used a base speed and added/subtracted the steering value:

$$ M_3=v+u $$ $$ M_4=v-u $$

We should change this approach in Lab 4 because the robot must always have at least one wheel at 100% PWM. As we removed base_speed and speed_decay for making velocity greater, we put motors its maximum speed and values of them change to:

$$ M_3=100+u $$ $$ M_4=100-u $$

For positive u:

$$ M_3=100 $$ $$ M_4=100-u $$

For negative u:

$$ M_3=100+u $$ $$ M_4=100 $$

Therefore, one wheel is always at 100% PWM.

For example:

|  `u` |   M3 |   M4 |
| ---: | ---: | ---: |
|    0 |  100 |  100 |
|   20 |  100 |   80 |
|   50 |  100 |   50 |
|  100 |  100 |    0 |
|  200 |  100 | -100 |
|  -20 |   80 |  100 |
| -100 |    0 |  100 |
| -200 | -100 |  100 |


## **5. Control Loop**
BEGIN

        Read A4 and A3
                ↓
        Normalize sensor values
                ↓
        Calculate error
                ↓
        Calculate PID controller
                ↓
        Clamp u to [-200, 200]
                ↓
        Calculate M3 and M4
                ↓
        Set motor powers
                ↓
        Wait 30 ms
                ↓
        Repeat

END

## **6. What changed?**

In Lab 3, the robot used a PID controller with a reduced base speed and changed both motor speeds around that base value. For Lab 4, I kept the PID structure but removed the base speed because one wheel must always operate at 100% PWM, so the controller now steers by braking only the inner wheel. I increased the controller output range to ±200. The main limitation on lap time was balancing aggressive PID corrections on sharp curves with stability, because excessive integral accumulation or derivative corrections can slow the robot or cause it to leave the track.

## **7. Complete pseudocode for this projec**
BEGIN

    ref_L = sensor A4
    ref_R = sensor A3

    Kp = 1
    Ki = 0.005
    Kd = 8

    i_sum = 0
    prev_err = 0


    LOOP FOREVER

        err = (sensor A4 - sensor A3) - (ref_L - ref_R)

        i_sum = max(-200, min(200, i_sum + err * 0.01))

        p = Kp * err

        i = Ki * i_sum

        d = Kd * (err - prev_err) / 10

        u = p + i + d

        prev_err = err

        u = max(-200, min(200, u))


        M3 power = 100 - u

        M4 power = 100 + u


        WAIT 30 ms

    END LOOP

END
