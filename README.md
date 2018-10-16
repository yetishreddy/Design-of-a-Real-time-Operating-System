# Design-of-a-Real-time-Operating-System
The goal of this project is write an RTOS solutions for an M4F controller that implements that implements a preemptive RTOS solution with support for semaphores, yielding, sleep, priority scheduling, priority inheritance, and a shell interface.
EE 6314 Advanced Embedded Microcontroller System Design
Project 1 – Design of a Real-time Operating System
Fall 2018

1 Overview	

The goal of this project is write two RTOS solutions for an M4F controller that implements that implement a cooperative and preemptive RTOS solution with support for semaphores, yielding, sleep, priority scheduling, priority inheritance, and a shell interface.
. 
A simple framework for building the RTOS is included in the rtos.c file.

2 Requirements

Scheduler:
Modify the rtos.c functions to support system timers, yielding, and semaphores.

Each time the scheduler is called, it will look at all ready processes and will choose the next process.  

Modify the scheduler to add prioritization to 16 levels.

Note: The method used to prioritize the relative importance of the tasks is implicit in the assignment of the prioritization when createThread() is called.  

Kernel Functions:
function yield() that will yield execution back to the kernel that will store the return address and save the context necessary for the resuming the process later.  

function sleep(time_ms) and supporting kernel code that will store the return address and save the context necessary for the resuming the process later.  The process is then delayed until a kernel determines that a period of time_ms has expired.  Once the time has expired, the process that called sleep(time_ms) will be marked as ready so that the scheduler can resume the process later.

function wait(semaphore) that causes a process to block until a resource or resources is available.  The waiting process will be recorded in the semaphore process queue.  

function post(semaphore) and supporting kernel code as discussed in the lectures.  

function createThread() to store the process name and initialize the process stack as needed.

function destroyThread() that removes a process from the TCB and cleans up all semaphore entries for the process that is deleted.

function systickIsr() that handles the sleep timing and handles the preemptive task switching.

function svsCallIsr() to handle all task switching for preemptive mode..  Modify the createThread(), deleteThread(), rtosStart(), setThreadPriority(), yield(), sleep(), wait(), and post() to use this ISRs.

function pendSvIsr() to handle all task switching in both cooperative and preemptive modes.

Add a shell process that hosts a command line interface to the PC.  The command-line interface should support the following commands (borrowing from UNIX):

ps: The PID id, process name, and % of CPU time should be stored at a minimum.

ipcs: At a minimum, the semaphore usage should be displayed.

kill <PID>:This command allows a task to be killed, by referencing the process ID.

reboot: The command restarted the processor.

pidof <Process_Name> returns the PID of a process

<Process_Name> & starts a process running in the background if not already running.  Only one instance of a named process is allowed.

set priority on|off turns priority scheduling on and off (on by default)

set preemptive on|off turns preemption on and off (on by default)
