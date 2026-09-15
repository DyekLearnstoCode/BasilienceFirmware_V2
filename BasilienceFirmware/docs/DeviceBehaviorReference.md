# Device Behavior Reference

This document answers one question over and over: **if this happens, what does the device actually do, and what would you see in the app?**

It is meant for two situations. First, testing: pick a row, cause that exact condition on the bench, and check the device does what the row says. Second, explaining the system to someone without a technical background: each row can be read on its own, in plain terms, without needing to understand how the firmware is built inside.

Everything in this document describes the main grow-chamber controller. The Harvest Scale is a separate device and is not covered here.

One rule applies everywhere in this document and is worth stating once at the top: **all of the automatic control described below keeps working even if the device has no internet connection.** Losing Wi-Fi or the cloud connection only affects reporting and alerts, covered in its own section further down. It never pauses dosing, refilling, cooling, or fogging.

## 1. Automatic pH and EC Correction

| If this happens | What the device does | What you'd see | Roughly how long |
|---|---|---|---|
| pH or EC drifts outside the safe range, and the reading has been steady long enough to trust | Runs the correct pump for 5 seconds (pH Up, pH Down, Nutrients, or opens the water valve to dilute EC), then stops and watches quietly | Status changes to Dosing, then Stabilizing | Dose itself: 5 seconds |
| A correction has just finished dosing | Circulates the water and watches without dosing again yet, so the just-added chemical has time to mix in | Status stays on Stabilizing, live reading hidden behind a "Stabilizing" loader | 90 seconds before the first check |
| After that first quiet period, the reading is still outside the safe range and has settled into a steady, unmoving value | Doses again in the same direction, or reverses direction if it went past the opposite target | A new Dosing cycle starts | Roughly 25 seconds after settling, on top of the 90 seconds above |
| While waiting, the reading is clearly moving in the wrong direction (getting worse, not just holding steady) | Doses again right away instead of waiting for it to fully settle first | A new Dosing cycle starts sooner than the steady-state case above | Checked every 30 seconds |
| A correction has been running for a while and still hasn't reached the target | Keeps trying (redosing as above) until 4 minutes total have passed since the correction started | Status stays on Dosing/Stabilizing throughout | 4 minutes is the hard cutoff |
| 4 minutes pass and the target still hasn't been reached | Stops trying and locks that one system (pH or EC) until a person clears it | An alert appears, and that system needs "Reset Safety" (Admin only) before it will try again | Locks at exactly 4 minutes |
| The target is reached before the 4-minute limit | Stops the correction and goes back to normal | Status returns to Normal, live reading reappears | As soon as the target is confirmed |
| A dose just finished, even if the reading already looks fine | Refuses to start another correction for a short cooldown, since a probe can look settled before the dosed chemical has actually mixed in | Nothing new happens for a moment even if the reading still looks off | 1 minute minimum between doses |
| EC needs to be diluted (water added) but the reservoir is already at its full working capacity | Refuses to add more water and locks the EC system for manual attention, rather than overflowing the tank | An alert appears, and EC dosing needs "Reset Safety" before it will try again | Immediate, no waiting period |
| Water level drops too low while a pH or EC correction would otherwise be needed | Refuses to dose at all until the water level recovers or a refill happens | The correction simply doesn't start, no new alert beyond the existing low-water one | Ongoing until water level recovers |

## 2. Automatic Refill

| If this happens | What the device does | What you'd see | Roughly how long |
|---|---|---|---|
| Water level drops to the refill threshold | Opens the water valve | Status changes to Refilling | Starts immediately |
| The valve has been open for a while | Closes the valve, then waits and lets the water settle before checking the level (a valve running water gives a bad reading if checked mid-flow) | Status stays on Refilling with a countdown | 30 seconds open, then 10 seconds settling |
| After settling, the water level reached the stop threshold | Refill is done | Status returns to Normal | Right after the 10-second settle |
| After settling, the water level did not reach the stop threshold yet | Repeats the open-then-settle cycle, up to 3 times total | Status stays on Refilling | Up to 3 rounds of the 30+10 second cycle above |
| 3 rounds pass and the reservoir still hasn't reached the stop level | Stops trying and locks the refill system for manual attention | An alert appears, and refill needs "Reset Safety" before it will try again | Locks after the 3rd round |
| The water level genuinely doesn't rise during a refill round, even though the valve ran (like a disconnected hose or an empty water source) | Counts it, but doesn't raise an alarm on just one round, since the water sensor itself can lag briefly right after a fill | Nothing visible yet after just one round | — |
| The water level fails to rise for 2 rounds in a row | Raises a "refill isn't actually working" alert, while still continuing its normal attempts | A new alert appears, separate from the normal low-water alert | After 2 consecutive no-rise rounds |
| An Admin starts a refill by hand from the app | Runs the valve continuously instead of the bounded 30/10 cycle above, since a person is watching and can stop it manually if something looks wrong | Status changes to Refilling, no fixed countdown shown | Continues until stopped or the level target is reached |

## 3. Water Cooling

| If this happens | What the device does | What you'd see | Roughly how long |
|---|---|---|---|
| Water temperature rises above the cooling trigger point | Circulates water, fills a short window, then runs the cooler while circulation is paused | Cooling equipment turns on | Fill: 5 seconds, then cooling: 30 seconds |
| The cooling window finishes | Circulates again briefly to flush the now-cooled water through, then checks the temperature again | Cooling equipment briefly runs circulation again | 5 seconds |
| After flushing, the water is still above the release temperature | Repeats the fill-cool-flush cycle again | Cycle repeats | Each full cycle: about 40 seconds |
| After flushing, the water has dropped to the release temperature | Cooling stops until needed again | Cooling equipment turns off | — |
| A pH or EC correction needs water circulation at the same time cooling wants to pause it | Cooling steps aside immediately so the chemical correction gets the water pump, and picks back up automatically afterward if still needed | Cooling briefly pauses, no alert, this is normal handoff, not a fault | Resumes as soon as circulation is free again |
| A commanded step (like turning the cooler or circulation pump on or off) doesn't confirm within a reasonable window | Locks the cooling system, since something isn't responding as commanded | An alert appears, and cooling needs "Reset Safety" before it will try again | 30 seconds without confirmation |
| The water temperature reading itself becomes invalid mid-cycle | Locks the cooling system rather than continuing to run equipment based on a reading it can't trust | An alert appears, and cooling needs "Reset Safety" before it will try again | Immediate once the reading is confirmed invalid |

## 4. Fogging

| If this happens | What the device does | What you'd see | Roughly how long |
|---|---|---|---|
| The device has just started up | Runs a one-time fog and airflow burst, then rests, before switching to its normal ongoing schedule | Fogger and fan run once, then pause | On: 90 seconds, off: 60 seconds, once |
| Normal operation, air temperature in the everyday range | Mists on, then rests, on a repeating schedule | Fogger cycles on and off | On: 5 minutes, off: 5 minutes, repeating |
| Air temperature climbs into the hot range | Switches to longer misting bursts with shorter rests | Fogger cycles on and off at a different pace | On: 8 minutes, off: 4 minutes, repeating |
| Air temperature drops into the cold range | Switches to shorter misting bursts with longer rests | Fogger cycles on and off at a different pace | On: 3 minutes, off: 5 minutes, repeating |
| A misting burst just ended | Keeps the fan running a little longer at full strength right after, to push the mist toward the plants before resting | Fan briefly stays on after the fogger turns off | 30 seconds |
| Water level drops too low | Fogging stops until the water level recovers | Fogger and fan turn off | Resumes automatically once water level is safe again |
| The air temperature/humidity sensor becomes unavailable | Falls back to the everyday (not hot, not cold) schedule instead of guessing, since the sensor can't currently confirm hot or cold conditions | No alert just for this, fogging continues on the fallback schedule | Ongoing until the sensor recovers |

## 5. Connectivity Loss (Wi-Fi / Cloud)

| If this happens | What the device does | What you'd see in the app | Roughly how long |
|---|---|---|---|
| The device briefly stops sending updates (a short Wi-Fi hiccup) | Keeps automating locally, tries to reconnect on its own | App shows Reconnecting | About 10 seconds of silence before this shows |
| The device stays unreachable for longer | Keeps automating locally, keeps trying to reconnect | App shows Offline | About 30 seconds of silence before this shows |
| The device stays connected to the home Wi-Fi network, but cannot reach the cloud specifically | Keeps automating locally | An alert reaches the app as normal (see the alerts section), and a "Device Unreachable" text message is also sent, since the app itself can't be trusted to reach the farmer in this state | After about 2 minutes of no cloud connection |
| The device cannot reach the home Wi-Fi network at all, even after retrying | Gives up on the saved network and opens its own setup hotspot, waiting for someone to reconfigure it | Nothing shows in the app anymore, since the app can't reach it, but a text message is sent (if the device has texted anyone before) | About 20 seconds of failed retrying |
| The connection comes back on its own | Resumes reporting to the app automatically | App returns to showing live readings and Online | No action needed |

## 6. Alerts and Notifications

Every alert below always tries to reach the app first, with an instant push notification and a popup. The "text message" column only applies while the device currently has no cloud connection, since a text on top of a push notification the farmer already got would just be noise.

| Alert | What it means | Reaches the app | Also texted, but only if offline |
|---|---|---|---|
| Low Reservoir | Water level dropped below the refill threshold | Yes | Yes |
| High Water Temperature | Reservoir water is too hot | Yes | Yes |
| High Air Temperature | Grow chamber air is too hot | Yes | Yes |
| Sensor Fault | One or more sensors are reporting invalid readings | Yes | Yes |
| Refill Ineffective | Refill ran but the water level didn't actually rise for 2 rounds in a row | Yes | No, not currently wired to text |
| EC Dilution Ineffective | Dilution ran but EC didn't actually drop for 2 attempts in a row | Yes | No, not currently wired to text |
| Harvest Due | A scheduled harvest date has arrived | Yes, checked hourly | Yes, only if the hourly check couldn't reach the app |
| Device Unreachable | The device is connected to Wi-Fi but can't reach the cloud | No, by definition the app can't be reached right now | Yes, always in this case |
| Setup Mode / Reconnect Failed | The device gave up on its saved Wi-Fi and is waiting to be reconfigured | No, same reason as above | Yes, if it has ever successfully texted anyone before |

## 7. Sensor Faults

Every sensor below follows the same basic shape: a single bad reading is treated as noise and ignored, but the same problem repeating several times in a row is treated as a real fault, and recovering also requires several good readings in a row, not just one, so a flickering connection doesn't rapidly flip back and forth.

| Sensor | If it disconnects or gives an impossible reading | What the device does | What you'd see |
|---|---|---|---|
| pH probe | Signal gets stuck at an electrically impossible level (a classic sign of a dead or disconnected probe) | Stops trusting the reading, blocks pH-based dosing until it recovers | pH card shows No Data, confirmed after about 8 seconds of a stuck signal |
| EC probe | Same as pH, an electrically impossible signal | Stops trusting the reading, blocks EC-based dosing until it recovers | EC card shows No Data, confirmed after about 8 seconds of a stuck signal |
| pH or EC probe | Reading never settles into a steady, trustworthy value for a long stretch, even without an outright electrical fault | Falls back to treating it the same as a sensor fault, rather than silently acting on a stale number forever | Card shows No Data | 
| Water temperature probe | Disconnected, or reporting the sensor's own "just powered on" placeholder value instead of a real reading | Ignores it, keeps the last good reading briefly, then gives up if it doesn't recover | Water Temperature card shows No Data after 3 bad readings in a row |
| Air temperature / humidity sensor | Disconnected, or reporting a value outside what's physically possible for this sensor | Ignores it, keeps the last good reading briefly, then gives up if it doesn't recover | Air Temperature/Humidity card shows No Data after 3 bad readings in a row |
| Water level sensor | Disconnected, or the distance reading times out | Ignores it, keeps the last good reading briefly, then gives up if it doesn't recover | Water Level card shows No Data after 3 bad readings in a row |
| Water level sensor | Reading jumps by an implausibly large amount in a single check (a classic sign of a bad echo, not real water movement) | Holds the last accepted value and waits for several checks in a row to agree on the new value before trusting it | Water Level card keeps showing the old value for a few seconds while this confirms |

## A note on testing with this document

Several of the timings above (the 90-second pH/EC settle, the 4-minute correction limit, the 3-attempt refill limit, and others) can be sped up during testing using the developer mock sensor tools already built into the app, without needing to physically wait out the real clock. See the Developer Options section of the in-app Mobile App Guide for how to feed a test value in directly.
