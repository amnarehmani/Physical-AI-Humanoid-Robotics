---
id: m1-ch5-fsm
title: "Lesson 2: State Machine Logic"
sidebar_label: "Lesson 2: State Machine"
description: "Managing the Patrol-Scan loop."
keywords:
  - fsm
  - logic
  - python
  - state machine
---

# Lesson 2: State Machine Logic

## The Chaos of Asynchrony

If you just write `go_to(A); scan(); go_to(B);`, your code will fail.
`go_to(A)` is non-blocking. The code will immediately execute `scan()` while the robot is still moving (or not even started).

We need a **State Machine**.

## States

1.  **IDLE**: Waiting for start.
2.  **NAVIGATING**: Moving to a waypoint.
3.  **SCANNING**: Rotating 360 degrees.
4.  **COMPLETED**: Patrol finished.

## The Loop

We use a `timer_callback` running at 10Hz to check the state.

```python
class PatrolNode(Node):
    def __init__(self):
        self.state = "IDLE"
        self.waypoints = [(2.0, 0.0), (0.0, 2.0), (-1.0, 0.0)]
        self.current_wp_index = 0
        self.timer = self.create_timer(0.1, self.control_loop)
    
    def control_loop(self):
        if self.state == "IDLE":
            # Start first waypoint
            self.navigator.go_to(*self.waypoints[self.current_wp_index])
            self.state = "NAVIGATING"

        elif self.state == "NAVIGATING":
            if self.navigator.is_task_complete():
                self.state = "SCANNING"
                self.scanner.start_scan() # Call Service

        elif self.state == "SCANNING":
            if self.scanner.is_scan_complete():
                self.current_wp_index += 1
                if self.current_wp_index >= len(self.waypoints):
                    self.state = "COMPLETED"
                else:
                    self.state = "IDLE" # Loop back
```

## Integrating the Scanner

The `ScannerNode` (which you will write) should offer a Service `StartScan`.
When called, it publishes `cmd_vel` to rotate `z=0.5` for 12 seconds (approx 360 degrees), then returns `success=True`.

## End-of-Lesson Checklist

- [ ] I have defined the states of my application.
- [ ] I use a timer loop to manage transitions, not `time.sleep()`.
- [ ] I handle the condition where the robot fails to reach a waypoint (Nav2 returns Aborted).
- [ ] I can trace the flow: Idle -> Nav -> Scan -> Idle.
