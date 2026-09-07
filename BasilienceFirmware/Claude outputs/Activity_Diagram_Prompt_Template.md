# Reusable Prompt Template — Basilience IoT Activity Diagrams

Use this template to generate additional activity diagrams (Figure 8, Figure 9, or new ones) in the same visual style as Figure 7 (IoT Device Initialization). Fill in the bracketed sections, then paste the whole prompt into your diagramming/AI tool of choice.

---

## The Prompt

```
Create a UML-style activity diagram for [SUBSYSTEM NAME / PROCESS TITLE], 
matching this exact visual style:

STYLE REQUIREMENTS:
- Black-and-white line diagram, no fill colors
- Process/action steps: rounded rectangles with black outline, centered text
- Decision points: diamond shapes with a yes/no branching question inside
- Start node: solid black filled circle, labeled "Start"
- End/continuation node: solid black filled circle with a letter label 
  (e.g., "A") if the flow continues to another diagram, or a bordered 
  circle with a black center if it's a true end point
- Branch labels ("Yes" / "No") placed directly on the arrows leading out 
  of each decision diamond
- Left-to-right primary flow with vertical drops into loops/sub-processes 
  where needed, matching a clean left-to-right, top-to-bottom reading order
- Arrows are thin black lines with solid arrowheads
- No color, no shadows, no icons — pure flowchart/swimlane-free structure
- Font: simple serif or sans-serif, consistent size throughout

CONTENT / LOGIC FLOW:
1. Start: [initial trigger or entry condition]
2. [First check or initialization step]
3. Decision: [condition to evaluate] 
   - If No: [loop-back or corrective action, then return to step 2/3]
   - If Yes: [proceed to next step]
4. [Continue mapping each monitored parameter or sub-process as its own 
   decision branch, e.g.: 
   - Measure [parameter] → is it within range? 
     - No → trigger [corrective actuator action] → re-check
     - Yes → continue to next parameter/step]
5. [Describe any timed loops, e.g., "run for X minutes, then rest for Y 
   minutes, repeat"]
6. [Safety interrupts: e.g., low water level stops all actions and 
   notifies user]
7. End: [final state, or connector node "A" if this flow continues into 
   another diagram]

LABELS TO INCLUDE ON EACH SHAPE:
- Use short, imperative phrasing inside boxes (e.g., "Measure Water Level," 
  "Activate Fogger," "Notify user to refill")
- Decision diamonds phrased as yes/no questions (e.g., "is fan running?", 
  "low water level?")

OUTPUT FORMAT:
- Provide as [PlantUML activity diagram code / Mermaid flowchart syntax / 
  draw.io XML / clean vector image] so it can be edited afterward.
```

---

## Example — Filled In for Figure 8 (Nutrient Reservoir Subsystem)

```
Create a UML-style activity diagram for the IoT Nutrient Reservoir 
Subsystem, matching this exact visual style:

[...same STYLE REQUIREMENTS block as above...]

CONTENT / LOGIC FLOW:
1. Start: continuous sensor acquisition begins
2. Measure pH level
3. Decision: is pH within 5.5–6.5?
   - No, above 6.5: activate pH Down pump (5 sec) → run circulation pump 
     (3 min, checking stability every 30 sec) → re-check
   - No, below 5.5: activate pH Up pump (5 sec) → same stabilization loop
   - Yes: continue to EC monitoring
4. Measure EC level
5. Decision: is EC within 1.2–2.0 mS/cm?
   - No, above range: activate dilution water (5 sec) → stabilize → re-check
   - No, below range: activate nutrient dosing (5 sec) → stabilize → re-check
   - Yes: continue to water temperature monitoring
6. Measure nutrient solution temperature (DS18B20)
7. Decision: is temperature above 25°C?
   - Yes: activate Peltier cooling → stays on until temperature drops to 23°C
   - No: continue
8. Check water level continuously in parallel — if below safe threshold, 
   interrupt and halt pH/EC/cooling actions, raise alert
9. All valid sensor branches loop back to step 2 continuously (no single 
   synchronization point — each parameter runs its own independent cycle)
10. End: connector to next diagram, or loop indefinitely

OUTPUT FORMAT: [your choice]
```

---

### Tips for reuse
- Swap the CONTENT / LOGIC FLOW section for each new subsystem (Figure 9 
  canopy, Admin workflow, User workflow, etc.) using the corresponding 
  paragraph text from Chapter 3 as your source material.
- Keep the STYLE REQUIREMENTS block identical across all diagrams so 
  Figures 7–11 look like one consistent set.
- If you want the actual diagram (not just the prompt) built directly as 
  editable code, let me know which tool you're using (PlantUML, Mermaid, 
  draw.io, Lucidchart) and I can generate the source for you.
