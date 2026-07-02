# Earth Rover Challenge — registration answer sheet

Form: **https://forms.gle/S4qWszRZpeNaDuZHA** (Google Form — exact fields not scrapeable;
answers below cover every field these forms typically ask. Paste as you go.)

## Identity
- **Name:** Aiden Fox
- **Email:** aidenfox2013@gmail.com
- **Affiliation / institution:** University of Southern California (B.S. Computer Engineering
  & Computer Science, expected Dec 2027)
- **Country / location:** United States (Los Angeles / San Diego, CA)
- **GitHub:** github.com/Ace2932
- **Team name:** "Aiden Fox" (solo) — or pick a handle; team-size rules unstated, entering as
  an individual. If it asks team size: **1**.
- **Role:** Sole developer (autonomy + software).

## Track interest (if asked)
- **Primary: Urban** (GPS-waypoint sidewalk navigation) — best fit for my nav background.
- **Secondary: Off-road** (image-goal). Open to Marathon if solo entries are viable.

## Background / experience (paste-ready)
> I'm a USC Computer Engineering & CS senior (Dec 2027) who builds autonomous robots
> end-to-end. My main project, NOVA, is an autonomous quadruped: 200 Hz Cortex-M7 motor
> firmware, a from-scratch servo-bus driver, a custom KiCad power board with INA226 rail
> telemetry, and ROS2 + LiDAR SLAM on a Jetson. I currently intern at Motif (AI-agents
> platform). For this challenge I'm running a GPS-waypoint follower against the Remote
> Access SDK (bearing controller over /data + /control), then layering camera-based
> sidewalk-keeping trained on the FrodoBots-2K / Berkeley-FrodoBots-7K datasets.

## Short "why participate" (if asked)
> Real-world outdoor autonomy on a standardized fleet is exactly the systems-integration
> problem I care about, and it lets me benchmark my nav stack against a shared course. I
> also build in public (@outofofficefox), so I'll document the run.

## Logistics answers (common fields)
- **Hardware access:** Will use the challenge testing allocation; may purchase an Earth
  Rover Mini+ if needed for practice.
- **Compute:** Off-board, my own machine (macOS / can spin up cloud GPU for the perception model).
- **Availability for pre-event testing:** Yes — up to the 20 hrs/week allocation.
- **Attend in person in Pittsburgh Sept 30–Oct 1?** Yes (pending travel; confirm).
- **How did you hear about us?** Competition listing / IROS 2026.

## After submitting
1. Watch email for the testing-allocation + SDK-token instructions.
2. Get token → run the SDK server → `python3 waypoint_follower.py` (see README).
3. Email michael.cho@frodobots.com if allocation/booking details don't arrive.
