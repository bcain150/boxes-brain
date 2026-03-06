Walker's Ghost Robot Boxes:

- we should add a PID tuning state which allows the user to tune pids with the bumpers?
    - alternatively it could auto tune using a commonly know PID tuning algorithm. Can't remember which one this is

- Need to figure out the mapping of buttons here is current brain dump:
    Tank Control: left stick drives left wheel, right stick drives right wheel
    Hold Middle Button: -> Connect/Disconnect, include haptic feedback
    RB + LB: -> Upright mode, When fallen over, haptic feedback to signal entering "standup-mode", A to accept
    RT + LT (hold) -> Retune mode: egages PID tuning
    DPad - up and down -> Speed Trim

- Haptic Feedback:
    Fallen Over State: Long rumble
    Engaging Reorientation: triple quick rumble
    Finished Reorientation: Single quick rumble (this could also just be a general "I am ready")
    ON: single quick rumble
    SLEEP_MODE: single quick rumble
        - note in this case, if PID moves too much, then assume someone is handling it so we should just turn off any motor control temporarily

- Sound Feedback ?:
    currently don't have hardware for this but buying a beeper probably isn't expensive
    - best use cases would be for connection, disconnection, low battery etc.

- State Managment:
    How should we do this? potentially with ROS parameters?
    - PID tuning mode: Tunes pid loop parameters autonomously
    - ACTIVE: upright and responding to motion control
    - FALLEN: fallen over, most likely at startup (someone lays it down)
    - SLEEP_MODE: Controller is off (lost connection), holding it's position
    - DISABLED_MODE: Can only get in this mode if a catestrophic failure occurs or one of the following:
        - fell over in a weird position
        - a node died
        - in sleep mode and a distruption is detected
        NOTE: can only get out of disabled mode by being connected 