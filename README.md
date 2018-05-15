# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

## Observations
The project was implemented with a simple PID controller using Total_Error = (-Kp * p_error) - (Kd * d_error) - (Ki * i_error) to direct the steering with a constant throttle of .71 resulting in speeds up to 75 mph.
During implementation the P, I, and D values were manually adjusted with some help from a 'twiddle' function (bad :( implementation). I initially started with values given in the course which were P=0.2 I=0.004 and D=3.0 then a few twiddle runs resulted in values around P=.19 I=0.008 and D=10, before it started to mess up too badly. From there I started to adjust and test the variable by making small adjustments to them and running the simulator without twiddle.

### P-roportional
While adjusting, I realized that the 'P' value increase would make the car shoot back into the middle to correct the CTE more aggresively and result in the car overshooting the sides, a lower value would still allow it to adjust but with a more moderate change. So I ended up with P=0.0797. More accurately, the P value determines how hard to steer based on the size of the CTE. A higher value will cause faster corrections but can cause the vehicle to lose control if the CTE gets large enough. Using only the P controller, the car will always be at an angle while crossing the CTE and will keep switching sides.

### I-ntegral
The 'I' or integral causes the vehicle to 'slide' left and right and helps to move the car toward the center, it oberves whether the car is spending too much time on one side and based on the value would slide the car towards the center. It was however very sensitive to any changes so the I value needed to be a small number, so I set it to 0.0013. Large value of I resulted in very large steering values.

### D-erivative
The 'D' or derivative value applies a resistance to the P value steering. The larger the derivative, the more resistance. Applying the resistance helps control the cars steering, by slowing/lowering it, as it gets closer to the center and helps to avoid the fast steering from the Proportional term. I settled with the value of 5.5.

Extra runs in the Output folder:
"Test-0" -> P=0.2, I=0.004, D=3.0
"Test-1" -> P=0.09, I=0, D=11.7
"Test-2" -> P=.105 I=0, D=7.9
"Test-3" -> P=0.079, I=0.0013, D=7.2
Test runs can be downloaded at https://bit.ly/2rEXVOw

Final Variables Results -> P=0.0797, I=0.0013, D=5.5

![FinalVariablesRunGIF](https://github.com/o-ali/CarND-PID-Control-Project/blob/master/Outputs/Final-PID-GIF.gif)

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
