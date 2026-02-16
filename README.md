# Balancing Robot Design and Implementation

This repository is a structured set of lab notes and implementation records for developing a two-wheeled self-balancing robot. It documents the workflows from physical modeling and parameter measurement through firmware design, control implementation, and experimental validation.

The focus is on reproducibility and engineering rigor: measured motor parameters, deterministic control-loop behavior, system identification, control design tradeoffs, and verification testing are all captured alongside working code and experiments.

Topics covered include:
- Control design and validation workflows for balancing robots
- Motor parameter measurement (phase resistance, d/q inductance, damping)
- Deterministic embedded firmware structure for real-time control
- Encoder integration and signal integrity checks
- CAN communication and timing behavior
- Failure modes and debugging notes
- Practical test methods and instrumentation results

These notes are intended to make the development and validation process transparent and repeatable, not just to present final code.

## My balancing robot journey, specs and testing

* A subset of steps required to get a robot to stand [[LINK](DOCS/balancing_checklist.md)]
* Common issues that can break balancing in the brain board project [[LINK](DOCS/balancing_failures.md)]
* Firmware design choices that keep the balancing control loop deterministic [[LINK](DOCS/determinism_design.md)]
* Measuring phase resistance, d-axis, and q-axis inductance of a BLDC Motor [[LINK](DOCS/BLDC_inductance_measurement.md)]
* Prompt ChatGPT so it is less of a exhuberant cheerleader and more of a skeptical lab partner: [[LINK](DOCS/chatGPT_tips.md)]
* Development of the robot control board [[LINK](brainboardV1.2/README.md)]

## Not totally coherent discussions:
* [LQR](DOCS/learn_LQR.md). 
* Using the MT6701 [[LINK](DOCS/MT6701.md)] → works great but required some minor modifications to MESC position and [velocity measurement](DOCS/pos_vel_CAN.md)
* Measuring jitter on the MESC [[LINK](DOCS/jitter_testing.md)] → none found at current operating speeds
* Implementing CAN [[LINK](DOCS/CAN.md)]
* Notes about MESC [[LINK](DOCS/MESC_control_variables.md)]
* The inadequacies of [PID](DOCS/PID_position.md). 
* How the brain board firmware works: [teensy code](DOCS/teensy_firmware_walkthrough.md). 
* Brain board firmware [[LINK](DOCS/software_specs.md)]

## Laboratory notes, 2025

* [Position hold](DOCS/sept12_position_hold.md)
* [Controller issues](DOCS/sept18_controller_issues.md)
* [Torque nonlinearity](DOCS/sept19_torque_nonlinearity.md)
* [PID notes](DOCS/sept23_PID_notes.md)
* [Least squares](DOCS/sept30_least_squares.md)
* [Torque response](DOCS/oct03_torque_response.md)
* [LQR modeling](DOCS/oct11_LQR_modeling.md)
* [LQR height](DOCS/oct12_LQR_height.md)
* [IMU testing](DOCS/nov16_IMU.md)

## Let's be real

**You probably will not want to hear this.** If you came here for the code to make a balancing robot, it is here, but it probably wont help. 

You might assume building a balancing robot is mostly a matter of assembling parts and flashing someone else’s code onto it. I used to think that too — until I finally worked out how to create on one. Balancing robots are not just bolted together -- they are tuned and integrated. Everything about them depends on tight interactions between hardware, sensors, motors, and feedback loops, and those interactions are different for every single robot.

A balancing robot is a dynamic control system, not a static device. It’s constantly trying to predict its own motion, measure its own tilt, compensate for delays, cancel vibration, and stabilize itself against gravity — hundreds of times per second. That means even small differences in hardware completely change the way the controller behaves. For example:

- Motors & ESCs: Different torque curves, different dead zones, different current limits, different FOC tuning.
- Wheels & tires: Diameter, traction, inertia, compliance — all change how the robot responds to torque.
- IMU: Sensitivity, noise, mounting vibration, sample timing, bias drift — all affect measured tilt.
- Microcontroller timing: Loop rate, jitter, scheduling delays, filter performance — all shift the controller behavior.
- Physical build: Height of the center of mass, frame stiffness, mass distribution — all change the robot’s dynamics.

If you change any of these, even slightly, the controller behaves differently. That’s why tuning is so time-consuming: you’re adjusting a control law to match the physics and imperfections of your specific robot.

_You are welcome to take my code and run it on your bot._ But unfortunately just like musical instruments or high-performance cars, balancing robots need to be customized, adjusted, and dialed in until the system “feels right.” The code is just the beginning; the tuning is the craft.

Why? Well:
- Your robot will not balance without a lot of tuning.
- What's most important to use this project as a prompt for making your own system
- Tuning is the real work — not a flaw, but part of the fun
- What really makes a robot stand is eliminating all the problems like jitter, timing issues, CAN communications, and modeling

**Finally, dearest reader**... I care about you, youre adorable, but quite unfortunately one the PCB for one of the ESCs that I use is closed source. Nothing would delight me more to make it open and I will try. For a while, I was using the Flipsky Mini FSESC6.7 pro 70A but I got tired of blowing them up because of cheap parts. Some day I will make a balancing bot that runs on [moteus](https://mjbots.com/products/moteus-r4-11) or [odrive](https://odriverobotics.com) but right now that is not a priority. 

For these reasons and others you cant expect to use this repo to build a balancing robot like some sort of kit.

## The really great news is there is a new source of help. 

I want to be transparent about this: building a robot is not about writing code. I've written code for 20 plus years, in my case I worked through the entire project with ChatGPT as a kind of engineering partner. ChatG helps with asking questions, testing ideas, refining approaches, debugging weird behavior, and iterating the design over and over.

A balancing robot is a complex closed-loop control problem. As far as I know I never would have been able to use Google to address how robots behaves. ChatGPT creates the ability to explore those problems conversationally:

- When I wasn’t sure whether the IMU filtering was right, I asked it to analyze my data.
- It is incredible at helping with plotting results
- When I had motor vibration I couldn’t explain, I used chat to brainstorm possible causes and experiments.
- It taught me everything about LQR, interpretted equations, supplied python code to perform most of the calculations. It involved a lot of debugging -- when the LQR gains seemed wrong, I walked through the math with it.
- When the ESC behavior looked inconsistent, we debugged line by line until we understood it.
- When I needed to test an assumption, it helped me design the test.
- When something behaved strangely, I could describe it and get three possible hypotheses to try.
- ChatG is uncanny at image analysis. It reads part numbers off of chips and will take you to the spec sheet. Take a picture of a square wave of your scope - - the damn thing will tell you the frequency, amplitude of the wave. It also taught me how to set up [one-shot measurements](DOCS/BLDC_inductance_measurement.md). 

The project was far less like copying code from the internet and more like having a very patient senior engineer building one-off code bundles to eventually get this thing to stand. 

**We live in a world of prompt-driven engineering:** I shaped my thinking through prompts, refined those prompts based on the robot’s behavior, and used those conversations to build understanding that would have taken months on my own. ChatGPT didn’t “solve the problem” — it helped me reason through it, step by step.

So if you see this robot standing and balancing in a video, know that behind the scenes was not just hardware and code — it was hundreds of micro-conversations, experiments, adjustments, and iterations. ChatGPT was the scaffolding that helped me structure that process.

## ACKNOWLEDGEMENTS
This project would be completely impossible without the help, support and guidance of David Molony, developer of [MESC firmware](https://github.com/davidmolony/MESC_Firmware.git). Use of the MESC firmware is described throughout this repo. 
