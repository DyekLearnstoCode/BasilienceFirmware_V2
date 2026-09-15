# Automation Timing Audit (2026-09-15)

This document explains a check that was done on every wait time, timer, and countdown used by the automation system, the parts of the firmware that dose pH and EC, refill the reservoir, cool the water, and run the fogger. The goal was to make sure all these timers agree with each other and that nothing in the code disagrees with what the project's own documentation says.

## The problem found

The firmware's guide, `FIRMWARE_GUIDE.md`, had two sentences describing pH and EC correction that no longer matched the actual code.

The guide used to say the system waits "one minute" after a dose before checking the reading again, and that it gives up after "3 tries."

The real firmware works differently now:

- The wait before the first check is 90 seconds, not one minute.
- There is no longer a fixed number of tries. Instead, each correction gets a 4 minute time budget. Within that time, the system keeps redosing if the reading has settled but is still outside the safe range, or right away if the reading is moving in the wrong direction. If the target still has not been reached once the 4 minutes runs out, the system locks that part (pH or EC) and asks for manual attention.
- The old settings that used to control the "3 tries" limit do not exist in the code anymore. The guide was still naming them even though they were removed a while ago.

Both sentences in the guide have been rewritten to match what the code actually does today.

While checking this, one more leftover was found. The guide mentioned a setting called `MIXING_DURATION` as if it were part of how dosing works. It is not. That setting still exists in the configuration file but nothing in the firmware ever reads it. It has no effect on anything. This has been noted in the guide so nobody assumes it is doing something. It was not deleted, since removing it is a separate decision.

## Everything else that was checked, and found correct

- **Refilling the reservoir.** Each attempt runs the water valve for 30 seconds, then waits 10 seconds for the water sensor to settle before checking the depth again. Up to 3 attempts are allowed. Even in the worst case, all 3 attempts together take about 2 minutes, well inside the system's overall 5 minute safety cutoff for any single operation.

- **Backup safety timers on pumps and valves.** Every pump and valve has a second, independent timer running in the background that will force it off if the main program ever gets stuck or delayed. In every case checked (the pH pumps, the EC pumps, the water valve during refilling and dilution, and the water cooler), this backup timer is always set longer than the part's normal run time. That means it can never accidentally cut off a dose or a refill early during normal operation. It only ever acts as a true backup.

- **Fogging schedule.** After each misting burst, the fan keeps running a little longer to push the mist toward the plant roots before the whole system rests. That extra fan time (30 seconds) is always shorter than the rest period that follows it, in every temperature mode (normal, hot, and cold weather settings), so it can never run into the start of the next misting burst.

- **Water cooling.** The cooling cycle has its own stall detector that locks the system if a step does not finish within 30 seconds. This detector correctly skips itself during the one step where a longer wait is normal and expected, which is handing control of the water pump over to an in-progress pH or EC correction. That handoff can take up to 90 seconds on its own, and the code already accounts for that instead of mistaking it for a stall.

- **The pH and EC 4 minute time budget.** The math behind this budget was checked step by step. The 90 second initial wait, the 25 second steady-reading check, and the 30 second recheck interval all fit properly inside the 4 minute total, giving the system room for roughly two redosing attempts per correction, which matches what the redesign was meant to do.

## One thing worth knowing about, but not a problem

After a refill, the water level sensor has its own short delay before it fully trusts a new reading, to avoid being fooled by splashing water. In rare cases, that delay can take almost as long as the 10 second pause the refill process itself waits before checking the water level.

In theory, this means a single refill attempt could look like it added no water, even though it actually did, simply because the sensor had not caught up yet at the moment it was checked.

This is already handled safely. The system only warns that a refill "did not work" after two attempts in a row show no rise, not just one. By the second attempt, the sensor will have caught up, so a real refill will always show up as a real rise before any warning is given. No change was needed here. It is only mentioned so the reasoning behind that "two in a row" rule is on record.

## Summary

One real problem was found: outdated wording in the firmware guide describing pH and EC correction timing. That has been corrected. One unused setting was flagged as dead but left in place. Every other timer, wait, and countdown in the automation pipeline was checked against how it is actually used in the code and found to agree with itself and with the rest of the system.
