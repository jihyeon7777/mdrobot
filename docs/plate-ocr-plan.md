# Licence plate OCR — working plan

Bring-up notes for [`mdrobot_plate_ocr`](../src/mdrobot_plate_ocr). Every step
was verified on the real camera before the next one started, and each **Result**
block records what actually happened — including the three designs that were
built, measured, and thrown away.

The package is documented in its own
[README](../src/mdrobot_plate_ocr/README.md); this file is the record of *why*
the defaults are what they are.

## Target

Read the plate taped to a box about 1 m in front of the robot's USB camera and
publish the text, on a Raspberry Pi 5 reached **only over SSH**. Streaming
frames to a laptop RViz had already been tried and was too slow to see anything,
so nothing here publishes an image: the text goes on a topic and framing is
checked by opening annotated JPEGs written to disk.

The plate turned out to read **`123가4568`** — three digits, one Hangul
syllable, four digits, the post-2019 Korean format.

**No motor is involved.** The [hardware safety protocol](../CLAUDE.md) does not
apply to this work.

## Progress

- [x] **Step 0** — OCR chain on a synthetic plate · *SW only*
- [x] **Step 1** — camera controls and one captured frame · *HW*
- [x] **Step 2** — settings matrix, and the split strategy it forced · *HW*
- [x] **Step 3** — live loop, debounce, debug JPEGs · *HW*
- [x] **Step 4** — ROS 2 package, node, topics, tests · *HW*
- [x] **Step 5** — adaptive exposure and plate-centre offset · *HW*

## Environment as found

| Item | Value | Consequence |
|---|---|---|
| `cap.read()` @720p and @480p | 64 ms (15.6 fps) both | Resolution is free; the bottleneck is exposure |
| `auto_exposure` | `3`, `exposure_dynamic_framerate=1` | Camera trades frame rate for light → blur |
| `power_line_frequency` | `1` (50 Hz) | Wrong for Korea; bands under LED light |
| Focus controls | **none** | "AF" webcam exposes no focus knob at all |
| `libtesseract5` 5.3.4 | already installed | Pulled in by `python3-opencv` |
| `.traineddata` | none | No OCR possible until `apt install` |
| Korean fonts | `fonts-nanum`, `fonts-noto-cjk` | Enables Step 0 and template matching |
| PEP 668 | `EXTERNALLY-MANAGED` | `pip` blocked, apt is not |
| rosdep `tesseract-ocr-kor`, `python3-tesserocr` | no keys | README apt line, not `package.xml` |

---

## Step 0 · OCR chain, no camera

`probe.py --synthetic` renders `12가3456` with NanumGothicBold at plate
proportions and OCRs it under every page segmentation mode.

**Pass**: the digits are right.

### Result — PASS, 2026-08-14

All four modes returned `12가3456` exactly, Hangul included, at 14–33 ms with
`tesserocr`. The CLI would have paid 150–400 ms of engine init per call, so
`python3-tesserocr` earned its place immediately.

---

## Step 1 · Camera controls and one frame

Apply 60 Hz anti-flicker and manual exposure via `v4l2-ctl`, capture one frame,
report focus and glyph height.

**Pass**: the plate is in frame, sharp, and its glyphs are near Tesseract's
30–40 px working range.

### Result — PASS after two corrections, 2026-08-14

The first frame was **badly overexposed** — 20 ms was far too long for this
room, and the plate's thin print washed out entirely. Sweeping the exposure over
the plate region:

| exposure | 1.0 ms | 2.0 | 4.0 | 6.0 | 8.0 | 12.0 | 15.6 | 20.0 |
|---|---|---|---|---|---|---|---|---|
| contrast (std) | 20.3 | **30.5** | 27.5 | **28.4** | 28.2 | 26.8 | 21.6 | 15.3 |
| clipped % | 0.0 | 0.0 | 0.0 | 0.0 | 0.1 | 0.1 | 1.6 | 2.0 |

Contrast peaks at 2–8 ms and collapses at the camera's own 15.6 ms default. 6 ms
became the starting value — and Step 5 later replaced the fixed number entirely.

Second correction: at 720p the glyphs were only ~20 px. **1080p is a different,
tighter field of view on this camera** and put the same plate at ~55 px, for the
same 64 ms per frame. 1080p became the default.

Only then was the plate legible enough to read what it says: `123가4568`, not
the digits-only string it appeared to be at 720p.

---

## Step 2 · Settings matrix

Rank every region × preprocess × psm × language combination over one saved frame.

**Pass**: some combination produces a valid plate.

### Result — FAIL as designed, then PASS after a redesign, 2026-08-14

**No single-pass combination read the plate.** The failure was consistent and
informative: the digits landed and the Hangul did not. `12374568`, `12314568`,
`12324568` — seven of eight characters, with `가` read as a digit. The one
combination that did recover `가` (`adaptive`, 2x, psm 13 → `123가2568`)
corrupted a digit instead.

That is a structural limit, not a tuning problem: `tessedit_char_whitelist` is
**global**, so a single pass cannot be told "digits here, one syllable there".

So the crop is cut into characters first, and each group is read under its own
rule. Three things had to be got right, each after the obvious version failed:

1. **Segmentation by ink projection, not connected components.** `가` is `ㄱ`
   plus `ㅏ` with a gap; components return two characters. Runs closer than a
   tenth of the crop height are merged — the gap inside `가` measured 3 px, the
   smallest gap between characters 28 px.
2. **The printed border has to go first.** It touches every character, so the
   projection saw one run and reported a single character. Morphological line
   removal failed — the plate is photographed at a slight angle, so no column
   holds a long enough vertical run. Removing connected components that *both*
   touch the crop edge and span most of it worked, and `trim_to_length` drops
   whatever fragments survive, by ink mass rather than width so the narrow `1`
   is not the one discarded.
3. **The digit runs need `-l eng`.** With `-l kor` and a digit whitelist
   Tesseract returned **nothing at all** — the Korean LSTM's preferred path is
   Hangul and whitelisting prunes it away. Under `eng` the same crop read `123`
   and `4568` at confidence 96.

The syllable still failed: `-l kor --psm 10` on the isolated glyph returned
`기`, which is not a syllable any Korean plate carries. **Template matching**
fixed it — only 40 syllables are legal, so rendering all of them in the installed
fonts and taking the best normalised cross-correlation picked `가` with a 0.058
margin over `거`.

A **text-band detector** (blackhat → Sobel → close) then removed the need for a
hand-set ROI: on the full frame it returned exactly one candidate, enclosing the
plate, in 56 ms.

Final: `123가4568` at confidence 96, no ROI, ~100 ms.

---

## Step 3 · Live loop

Run continuously, debounce, write debug JPEGs.

**Pass**: the plate reads live, and removing it stops the output.

### Result — PASS after backing out a "fix", 2026-08-14

First run: 76 frames, ~75% read `123가4568` exactly, ~5.5 Hz achievable. Two
defects — a digit run coming back empty (~20%), and the band's padding pulling
in a neighbour so three digits read as four.

The obvious fix, re-reading each character separately with `--psm 10`, made
things **worse**: per-character OCR always returns *something*, so it converted
rejections into plausible-but-wrong plates and published `125가4568` six times
in 145 frames. It was reverted. A wrong plate is worse than no plate.

What shipped instead: retry the whole band under a different psm, and take the
retry **only if its digit count matches the segmentation**. A mismatch is left
to fail validation. That raised correct reads to 92% with no wrong publishes
from that path.

The remaining errors could not be filtered by confidence — a wrong read scored
96 and a correct one 86 — so the debouncer was raised to **3 of 5**:

| run | frames | correct | wrong reads | **published** |
|---|---|---|---|---|
| confirm 2-of-5 | 143 | 131 (92%) | 12 | `123가4568` ×12, **`125가4568` ×2** |
| confirm 3-of-5 | 219 | 197 (90%) | 22 | `123가4568` ×18, **nothing else** |

Wrong reads are sporadic and disagree with each other; the correct one dominates.
Three-in-five separates them where confidence cannot.

Template match margins over the same run: correct `가` median 0.106, wrong
syllables 0.029–0.053. A threshold is available (`min_syllable_margin`) but is
off by default — the debouncer already did the job, and a second filter tuned on
one scene is a good way to overfit.

---

## Step 4 · ROS 2 package

**Pass**: `colcon build`, the topics carry the plate, `pytest` is green.

### Result — PASS, 2026-08-14

Built and launched. Over 25 s the node published `123가4568` eight times and
nothing else. `ros2 topic echo --once` attached *after* all the reads and
received the plate immediately, which is the whole point of latching `~/plate`:
"no message" has to mean "nothing recognised", not "the node is dead".

`ocr_rate` was lowered from 4.0 to 3.0 Hz — 4.0 overran its 250 ms budget on the
node's slower frames. 235 unit tests pass; `normalize` and `debounce` need no
third-party package at all, and the rest fake the OCR engine.

---

## Step 5 · Adaptive exposure and plate-centre offset

Two requirements raised during Step 4.

**Fixed exposure is a trap.** 6 ms is right for this room and produces a black
frame in a darker one. Handing metering back to the camera is not the answer
either — that is what caused the Step 1 overexposure, because it averages a
scene of white wall and white paper and exposes for the average.

**Pass**: starting from a deliberately wrong exposure, the loop converges and
resumes reading; the offset matches the picture.

### Result — PASS, 2026-08-14

`ExposureController` meters **the region the OCR is about to read** and steps
towards a target, with a saturation guard (a plate can be blown out while the
frame's mean looks fine) and a settle delay (a UVC exposure change takes a few
frames to appear, and reacting to stale frames oscillates).

Started at 400 (40 ms, unreadable), it converged in seven steps —
400 → 308 → 237 → 182 → 140 → 108 → 83 → **64** — and went straight back to
reading at confidence 96. The bench optimum was 60.

`offset_from_centre` reports the plate's centre against the frame's: pixels,
normalised [-1, 1], an estimated bearing, and the plate's width as a fraction of
the frame's. Signs follow the image, which is also what a robot wants — `+x` is
right of centre, `+y` is below. Published on every accepted read rather than on
debounced confirmation, because a control loop wants it at frame rate.
Measured: `(-0.07, -0.86)`, bearing −3.0°, width 24%.

The debug overlay draws the frame centre, the plate centre and the line between
them, and moved its header to whichever edge hides less of the plate — a plate
taped high on a box sits exactly where a header would go.

---

# Reference

## What each unusual choice is defending against

| Choice | The obvious alternative, and how it failed |
|---|---|
| `detector: textband` | Quad search: white paper on a light wall gives Canny no closed contour, tape and bow break `approxPolyDP`, and by area a monitor beats the plate |
| `strategy: split` | One Tesseract pass: reads 7 of 8 characters, because a global whitelist cannot express the layout |
| ink projection | Connected components: `가` is two components, so 8 characters become 9 |
| `drop_border_components` | Morphological line removal: the plate is at an angle, so no column holds a long vertical run |
| `trim_to_length` by ink mass | By width: the digit `1` is the narrowest real character and gets discarded |
| `digit_lang: eng` | `kor` + digit whitelist: returns nothing at all |
| `syllable_source: template` | `-l kor --psm 10`: returned `기` for `가` |
| retry, never per-character | Per-character fallback: always returns something, so it publishes wrong plates |
| `confirm_count: 3` | Confidence threshold: a wrong read scored 96, a correct one 86 |
| `exposure_mode: adaptive` | Fixed value (dies in a dark room) or camera auto (blows out white paper) |
| 1080p | 720p: a tighter crop on this camera, glyphs at 20 px instead of 55 px |
| `tesserocr` | The CLI: reloads the model every call, 150–400 ms wasted per frame |

## Open questions

- Only one plate, one distance, one room have been tested. The template match
  margin and the segmentation thresholds are the parts most likely to need
  revisiting when any of those change.
- `bearing_deg` assumes a pinhole camera and a nominal 70° horizontal field of
  view. Fine as a readout, not calibrated.
- The plate typeface is not among the template fonts. Template matching works
  because the answer set is 40, not because the shapes match.

## Out of scope

- Vehicle detection, multiple plates, reading while moving.
- Older regional plates (`서울12가3456`).
- Driving the robot from the offset. If that is wanted, it talks over topics —
  this package deliberately depends on nothing else in the workspace.
