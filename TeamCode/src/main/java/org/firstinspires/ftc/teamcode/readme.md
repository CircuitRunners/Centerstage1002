Welcome.

So, we have this organized into controllers and procedures. 

Typically you only want to have a two-nest folder structure under a main

For example Controllers --> Commands (1) --> Intake Commands (2) --> files only from here on out

Controllers contains:
- Assemblies (root dir): these are pieces of code that for example contain the Robot class, encapsulating all the mechanism initializations and imports and commands and stuff
- Auto: contains all the libraries and shared files for example roadrunner, its trajectorysequence extension, and pedropathing
- Commands: contains all the commands that simplify programming for an action, specifically presets. read: docs.ftclib.org/ftclib/v/v2.0.0/command-base/command-system
- Common: contains util and utilities (util is builtin, utilities are things I/other programmers created to simplify processes)
- Subsystems: contains all the robot subsystems or physical modules. examples are intake, arm, claw, a launcher, etc.
- Vision: contains/encapsulates all the code for vision and smart detection for the robot

Procedures contains:
- Assemblies (root dir): pieces of encapsulating code for stuff
- Auto: the autonomous procedures (autos) that are run during matches
  - Red/Blue
    - Audience/Backstage (stage) then 0,1,2plus0,1,2,3,4,5,6,7 represents all the above, 2 = purple + yellow drop, + white pixels
- TeleOp: the tele-operated period where drivers do cool stuff
- **Tests**: a giant folder full of tests that can be activated and deactivated based on something cool (in progress, basically all based off tester.java, which can all be disabled MAYBE MAYBE PLS)
- 