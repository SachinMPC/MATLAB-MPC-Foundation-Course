# MATLAB-MPC-Foundation-Course
Foundation Course developed in MATLAB on Model Predictive Control 
Engineered systems, such as process plants, refineries, power plants, aircraft or automotive vehicles, consist of numerous interconnected units forming complex networks.  From the viewpoint of control system design, a common feature of such systems is that they have multiple inputs that must be simultaneously manipulated to achieve desired output behavior, often while operating in uncertain environments, i.e. these are multiple-input, multiple-output (MIMO) systems.  This course focuses on design, analysis and implementation of a widely used multi-variable optimal control approach referred to as Model Predictive Control (MPC) for optimal operation of MIMO dynamic systems. MPC uses an internal dynamic model for on-line forecasting  over a moving time window (horizon) and optimizes the inputs to be applied to the plant, in order to achieve the desired outputs. Major strengths of MPC are abilities to handle multi-variable Interactions  and  operating constraints in a systematic manner. Moreover, it is possible to simultaneously guarantee nominal performance and nominal stability under the ideal conditions.  
The course begins with the development of discrete time linear perturbation models for solving the classical optimal control problem in the real time using a moving time window framework. In the initial part of the course, control relevant local linear dynamic models are developed starting from nonlinear mechanistic dynamic models and are further used for the controller synthesis. Such mechanistic models, however, may not be available in many situations. Thus, development of MPC relevant linear state space model using system identification techniques and using operating perturbation data collected from the system is introduced in the later part of the course. 
The discrete linear model based MPC is re-formulated as a quadratic programming problem and shown to result in a state feedback controller formulation. The classical Linear Quadratic Optimal Control (LQOC) is developed using Bellman's dynamic programming and its connection with unconstrained finite horizon MPC and quasi-infinite horizon MPC are illustrated. The nominal stability of MPC is established using Lyapuov's second method. Time varying target following framework is introduced to achieve offset free control under model plant mismatch, sustained changes in unmeasured disturbances and setpoint changes. The time varying and stationary Kalman filters are introduced to deal with stochastic input disturbances and measurement noise systematically in the MPC framework. 
 The various elements of the course are designed to provide exposure to theoretical foundations of MPC along with practical implementation. Development of MPC relevant discrete dynamic model is viewed as an integral part of formulating an MPC scheme. Thus, adequate exposures are given to state estimation and system identification techniques and the relationship between these two complementary areas. The benchmark Quadruple Tank Process system is used as a theme simulation example to illustrate all the  concepts introduced in this course. An excursion to using reinforcement learning for control is also provided at the end of the course to connect MPC with learning methods thus highlighting connections between MPC and Reinforcement Learning, both being the powerful techniques for optimizing control systems. 
This teaching package introduces the Model Predictive Control Curriculum using MATLAB Live Scripts, Simulink Models and PDFs, and self practice assignments. 
# About the Course
## Who can take this course?
 This course is meant for graduate students in engineering and industrial practitioners of automation who intend to get exposure to advanced multi-variable control concepts. Senior undergraduate students can also take this course if they are interested in systems and control. It is assumed that the learner has studied the first undergraduate course in control systems taught in any engineering department. Background in linear algebra, linear ordinary differential equations, and, to some extent optimization will help in steering through the course. No specific background of any engineering discipline is assumed. 
Course Application Areas
Process Control (Chemical and Metallurgical Engineering), Robotics (Mechanical Engineering), Aeronautical Engineering, Systems and Control Engineering, Electrical Engineering
## Pre-requisite Knowledge
Linear Algebra: Matrix algebra, positive definite and semi-definite matrices, eigenvalues, and eigen-vectors.
Optimization: Familiarity with basic concepts of optimization (necessary and sufficient conditions for unconstrained and constrained optimization), quadratic programming
Fundamentals of Feedback Control Systems: Familiarity with classical control methods (transfer function representation of dynamic models, stability analysis of SISO systems, PID controller design) and the concept of feed-forward and feedback control, 
Fundamentals of Digital Control: Sampling, quantization, Analog-to-Digital Converters (ADCs), Digital-to-Analog Converters (DACs)
## MATLAB and Simulink Toolboxes Required
Control System Toolbox
Simulink Control Design
Optimization Toolbox
Statistics Toolbox 
System Identification Toolbox
Reinforcement Learning Toolbox
## Suggested MATLAB/Simulink Background Courses for beginners 
 MATLAB Onramp
Simulink Onramp
## Course Objectives
          The overall objective of this course is to introduce design, analysis and implementation of multi-variable discrete time state feedback optimal controllers for managing transient behavior of a system. Learning objectives can be summarized as follows:
Model Development: Develop computer control relevant linear dynamic models either from mechanistic dynamic models or from operating transient data
Controller Design: Design state estimators (model based estimators of unmeasured variables) and multivariable state feedback based controllers (Linear Quadratic Optimal Controller or LQOC) and Model Predictive Control or MPC) using the control relevant models
Stability Analysis: Analyse dynamic behavior of open loop system using controllability, observability and the controlled system using  Lyapunov’s first and second methods 
Performance Evaluation: Simulate the closed loop System behavior together with the designed state estimators an multi-variable controllers and benchmark systems
## Learning Outcomes
Students will be able to
Understand the different aspects of computer based control
Understand fundamental concepts of state feedback control including MPC
Synthesize and implement MPC control strategies using mechanistic, gray-box and black-box dynamic models and basis MATLAB toolboxes such as Control Systems, Statistics, Optimization and System Identification 
Analyze advantages of MPC over traditional control methods
Apply MPC techniques to solves various industrial and other real-world problems.

# Course Contents
This course is organized in 9 Modules.
## Module 1. Introduction to Model Predictive Control 
This module provides and overview of the course and introduces the theme example used in the course: the Quadruple Tank System. 
Lesson 1: Introduction to Advanced Control (Notes)
Lesson 2: Dynamic Models and Quadruple Tank Process 
## Module 2: Fundamentals of Moving Horizon Control
This module begins with nonlinear mechanistic model based optimal control formulation, and, using concepts of linearization and discretization, proceeds to introduce discrete linear model based feed-forward MPC formulations under the perfect model assumption and perfect state measurement 
### Lesson 1:  Moving Horizon Optimal Control
### Lesson 2: Local Linearization of a Nonlinear Dynamical Model
### Lesson 3: Discretization of Linear Models
### Lesson 4: Dynamic Simulation of Linearized Model
### Lesson 5: Reformulation of MPC using Linear Model
## Module 3: Quadratic Optimal Control and Stability Analysis
This module introduces classical quadratic optimal control (LQOC) theory and connects it with MPC. The nominal stability analysis of unconstrained and constrained MPC is discussed in this module. 
### Lesson 1: Linear Quadratic Optimal Controller: Formulation 
### Lesson 2: Stability Analysis of Discrete Linear Systems
## Module 4: Output Feedback and Servo Control 
This module presents different variants of MPC by relaxing the perfect model assumption. MPC variants that can deal with model-plant mismatch, setpoint (reference signal) tracking and output feedback are introduced using the concept of target setting. 
### Lesson 1: From LQOC to Quasi-Infinite Horizon MPC
### Lesson 2: MPC Target Tracking for Linear Process
### Lesson 3: Reference Tracking and Disturbance Rejection using MPC: Quadruple Tank Case Study
### Lesson 4: L4_Vanilla_MPC_Variants
### Lesson 5: Summary of Vanilla Linear MPC
## Module 5: State Estimator Design for MPC
This module introduces basics of state estimation. This includes design of Luenberger observers using pole placement method and Kalman filtering that give optimal state estimates in the presence of stochastic disturbances and measurement noise. 
### Lesson 1: Introduction to State Estimation
### Lesson 2: Luenberger (Pole Placement) Observer Design
### Lesson 3: Stochastic Disturbances and Kalman Filtering 
### Lesson 4: Stationary Kalman Filter and Drifting Disturbance Estimation
## Module 6: MPC using State Estimators
Implementation of LQOC and MPC using state estimators is introduced in this module. 
### Lesson 1: State Feedback Control Using State Estimator 
### Lesson 2: Offset-free State Space MPC using Innovation Bias Approach 
### Lesson 3: Output Tracking MPC using Innovation Bias Approach
## Module 7: Identification of Black-box Models from Data 
This module introduces fundamentals of grey-box and black-box model identification starting from experimental data obtained by perturbing the system under consideration. 
### Lesson 1: Introduction to System Identification
### Lesson 2: Output Error State Space Models
### Lesson 3: Innovation Form of State Space Models
## Module 8: MPC using Black-box Models
In this module, we present the state feedback control using estimators that are based on models directly identified from experimental data. We norther demonstrate offset free MPC formulations using such models. 
### Lesson 1: State Feedback Control Using State Estimator: Black Box Models
### Lesson 2: Black-box Model based MPC using Innovation Bias Approach
## Module 9: Connections between MPC and Reinforcement Learning - A brief excursion
This module introduces the fundamentals of Reinforcement Learning (RL) and the use of Bellman’s principle for solving RL-based optimization problems. It provides a comparative overview of Model Predictive Control (MPC) and RL approaches, followed by the implementation of an RL-based controller for the Q-Tank process. The module emphasizes both theoretical insights and practical performance evaluation of the RL controller.
### Lesson 1: MPC and Reinforcement Learning (RL)
### Lesson 2: Bellman's Principle and RL
### Lesson 3: DDPG and Actor Critic Network
### Lesson 4: RL: Q Tank Example
