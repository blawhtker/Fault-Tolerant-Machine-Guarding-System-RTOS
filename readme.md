
**ADJUSTED SENSOR DELAY FROM 150MS TO 50MS AS SUGGESTED BY STUDENTS ON YELLOWDIG**

## Engineering Analysis 
## Scheduler Fit
My task priorities guarantee every Hard Real-Time (H) task’s deadline by assigning 
the highest priority (3) to the ride_control_handler_task. Because FreeRTOS uses a
preemptive scheduler, this critical task can interrupt any lower-priority task the 
moment it's signaled by the E-Stop ISR or a sensor event. This ensures that safety 
logic for the Universal Creative ride system is always processed immediately, well 
within its deadline. The timestamped output proves this, as an event occurring just 
after tick [5000] results in an updated STATUS message by the next output cycle at 
[5250], showing the high-priority handler ran and changed the state in between.
[5000] Proximity = 95cm  STATUS: All Clear
[5250] Proximity = 15cm   STATUS: Obstruction - Ride Halted

## Race‑Proofing
A race condition could occur with the shared global state variables (ride_status, 
is_train_in_zone, current_proximity_cm), which are written by one task or ISR and 
read by others. These are protected by making them volatile, which is a C language 
primitive that prevents the compiler from making incorrect optimizations and ensures 
every read gets the absolute latest value from memory. For the ESP32's 32-bit 
architecture, reads and writes to these simple integer types are also atomic, meaning 
they cannot be interrupted halfway through. This combination prevents data tearing and 
ensures state consistency without the overhead of a mutex.

C // This volatile keyword is the primitive that protects against the race condition.
volatile RideStatus ride_status = RIDE_ALL_CLEAR; 

## Worst‑Case Spike
The heaviest load thrown at the prototype is a simultaneous E-Stop interrupt and a 
proximity sensor fault. The RTOS scheduler handles this by immediately running the 
E-Stop ISR, which signals the highest-priority ride_control_handler_task. This hard 
real-time task has an effective deadline of 10ms, set by its polling delay. The actual 
processing time for the state change logic is trivial, estimated at under 20 microseconds. 
This leaves a massive timing margin of over 9.98ms before the deadline would slip, proving 
the system is extremely robust and capable of handling worst-case event spikes safely.

## Design Trade‑off
One feature I chose not to add to keep the system's timing predictable was a dynamic 
audio system that would play sound effects from an SD card based on the ride's location. 
For a company like Universal Creative, rider safety depends on deterministic, guaranteed 
response times. File I/O from an SD card is a notoriously non-deterministic operation with 
highly variable delays, which would make it impossible to certify the controller as 
safety-critical. The right call was to simplify the system to its core safety functions, 
ensuring the hard real-time deadlines are always met and offloading non-essential features 
like audio to a separate, non-safety-critical processor.

## Task & Deadline Analysis
Task                       ISR Name	Period   Deadline	Type	  Consequence of Failure
gpio_isr_handler            Aperiodic / ~10µs	     Hard	      Catastrophic. 
-Failure to execute would render the physical E-Stop button useless.

ride_control_handler_task	         10ms	         Hard	        Catastrophic. 
-Failure to process a halt signal could lead to vehicle collision or injury.

train_sensor_monitor_task	         150ms	        Hard	      Catastrophic. 
-Missing a vehicle detection could defeat the anti-collision system.

status_output_task	               250ms	        Soft	      Minor. 
-A status message on the operator's console would be delayed or skipped. No impact on 
rider safety.

system_power_monitor_task	          2000ms	      Soft	      Trivial. 
-The power indicator light on the control panel might stutter, with no effect on ride safety.



==================================================================================
                  Concurrency Diagram 
==================================================================================

KEY:
  [ Hardware ]        --> Signal (Semaphore)     -->
  +----------+        ... Data (Shared Var)      ...>
  | Task/ISR |
  +----------+

----------------------------------------------------------------------------------

                                                                   +-----------------------+
                                                                   |  system_power_monitor |
[ E-Stop Button ]                                                  |  (Soft, 2000ms)       |------> [ Yellow LED ]
                                                                   +-----------------------+
      |
      | Hardware Interrupt
      V
+--------------------------+
|   gpio_isr_handler       |
|   (ISR / Hard)           |---- sem_emergency_stop_button --->+---------------------------+
|   (Aperiodic / ~10us)    |                                   |  ride_control_handler   |
+--------------------------+                                   |  (Hard, 10ms)             |
                                                               |                           |------> [ Red/Green LEDs ]
                                                               +-------------+-------------+
                                                                             |
                                                                             |... ride_status ...>+-----------------------+
                                                                             |                   |   status_output_task  |
                                                                             |                   |   (Soft, 250ms)       |
                                                                             |                   +-----------------------+
                                                                             |                               |
                                                                             |                               |
[ Proximity Sensor ]                                                         |<... is_train_in_zone ...       |
                                                                             |                               |
      |                                                                      |                               V
      | Sensor Reading                                                       |                           [ Serial Monitor ]
      V                                                                      |
+--------------------------+                                                 |
| train_sensor_monitor_task|---- sem_train_proximity_event -->+               |
| (Hard, 150ms)            |                                                 |
|                          |... current_proximity_cm ......................>+
+--------------------------+
