# mdrobot_plate_ocr

Reads a **Korean licence plate** from a USB camera and publishes the text as a
ROS 2 topic — plus where the plate sits relative to the centre of the frame.

**No image topic, on purpose.** This robot is reached over SSH. Streaming camera
frames to a remote RViz was too slow to see anything, which is why this package
exists: the recognised *text* goes on a topic, and framing is checked by opening
annotated JPEGs that the node writes to disk.

Depends on nothing else in this workspace. It shares the robot, not the motor
bus, and no motor is involved — the repository's hardware safety protocol does
not apply here.

## Install

```bash
sudo apt install tesseract-ocr tesseract-ocr-kor python3-tesserocr v4l-utils
```

`libtesseract5` usually arrives already, pulled in by `python3-opencv`, so the
real download is small. `tesseract-ocr-kor` and `python3-tesserocr` have no
rosdep keys, which is why they are here rather than in `package.xml`.

`python3-tesserocr` is optional but wanted: it loads the language model once,
while the `tesseract` CLI reloads it on every call and throws away 150–400 ms
per frame on a Raspberry Pi 5. Without it the package falls back to the CLI.

## Bring-up, without ROS 2

Run the probe from a bare clone. Each mode removes one variable, so work them in
order — if a step fails, the ones after it cannot succeed.

```bash
# 1. Is the OCR chain even working? Renders a perfect plate; no camera involved.
python3 examples/plate_ocr_probe.py --synthetic

# 2. Is the plate in frame and in focus? Writes one JPEG to open in your editor.
python3 examples/plate_ocr_probe.py --shot debug/shot.jpg

# 3. Which settings actually win on this scene? Ranks every combination.
python3 examples/plate_ocr_probe.py --matrix debug/shot.jpg

# 4. Live: reads, debounces, prints, and writes debug images.
python3 examples/plate_ocr_probe.py --live -v
```

Two numbers in `--shot` and in the debug overlay are worth more than the
confidence score:

- **glyph height** — Tesseract's LSTM wants roughly 30–40 px. Lower means "move
  the plate closer" or "raise the resolution".
- **focus** — variance of the Laplacian. Only comparable against itself on the
  same scene; slide the paper until it peaks. This webcam exposes *no* focus
  controls despite the "AF" in its name, so a human has to do it.

## ROS 2

```bash
colcon build --packages-select mdrobot_plate_ocr && source install/setup.bash
ros2 launch mdrobot_plate_ocr plate_ocr.launch.py
```

| Topic | Type | When |
|---|---|---|
| `~/plate` | `std_msgs/String` | A confirmed plate, and nothing else. **Latched** — a subscriber that attaches later still gets the last one, so an empty `echo` really does mean "nothing was read", not "the node is dead". |
| `~/plate_offset` | `geometry_msgs/Point` | Every accepted read. `x`, `y` normalised to [-1, 1] — `x` positive is **right** of centre, `y` positive is **below**; `z` is the plate's width as a fraction of the frame's. Usable directly as a control error. |
| `~/plate_detail` | `std_msgs/String` (JSON) | Every attempt, rejects included: raw text, confidence, region, segments, template match, focus, glyph height, exposure, offset, reject reason. |

```bash
ros2 topic echo /mdrobot_plate_ocr/plate           # just the plates
ros2 topic echo /mdrobot_plate_ocr/plate_detail    # is it alive, what does it see
```

All options live in [`config/plate_ocr.yaml`](config/plate_ocr.yaml), annotated
with what each value was measured to be and why.

## How it reads a plate

Six of the choices below are the opposite of the obvious one. Each is the result
of the obvious one failing on real frames.

**Find text, not rectangles.** Hunting for the plate's quadrilateral fails in
this exact scene — white paper on a light wall gives Canny no closed contour,
tape and a slight bow make `approxPolyDP` return five or six vertices, and by
area a monitor or a door frame beats the plate. A blackhat/Sobel/close pass looks
for *a wide run of vertical strokes* instead and returned exactly one candidate,
enclosing the plate, in 56 ms. No hand-set ROI needed.

**Split the line into characters before recognising it.** A single Tesseract pass
cannot express the constraint that matters: `tessedit_char_whitelist` is global,
so there is no way to say "digits here, one Hangul syllable there". One pass
either reads the digits and turns `가` into a digit (`12374568`), or reads `가`
and corrupts a digit.

**Cut by ink projection, not connected components.** A Hangul syllable is *made*
of disconnected parts — `가` is `ㄱ` plus `ㅏ` with a visible gap — so components
return two characters. Runs closer than a tenth of the crop height are merged;
measured here the gap inside `가` was 3 px and the smallest gap between
characters was 28 px.

**Read the digits with `-l eng`.** With `-l kor` and a digit whitelist Tesseract
returns *nothing at all*: the Korean LSTM's preferred path is Hangul and
whitelisting prunes it away. The same crop reads at confidence 96 under `eng`.

**Identify the syllable by template matching.** `-l kor --psm 10` on the isolated
glyph returned `기` for a `가` — not merely wrong but not a syllable any Korean
plate carries. Only **40 syllables** are legal, so rendering all of them in the
installed fonts and taking the best normalised cross-correlation starts with an
enormous advantage over choosing from ~2400. Tesseract's answer is still recorded
in `~/plate_detail` for comparison.

**Confirm before publishing, and never guess.** Over 219 live frames the correct
plate read 131–197 times and four different wrong plates read once or twice each,
never three times inside any window of five — so `~/plate` saw only the correct
one. Confidence cannot do this job: a wrong read scored 96 while a correct one
scored 86. And when a digit run reads back the wrong number of characters it is
left to fail validation rather than re-read character by character — that was
tried, and because per-character OCR always returns *something*, it converted
rejections into plausible-but-wrong plates six times in 145 frames.

## Exposure

The camera's own metering is what produced the first unreadable frames. It
averages the whole scene, and a scene of white wall plus white paper reads as
"bright", so it exposes for the average and pushes the plate's thin print into
saturation.

So the node meters **the region it is about to OCR** and moves the exposure a
step at a time (`exposure_mode: adaptive`, the default). Starting from a badly
overexposed 40 ms it converged to 6.4 ms in seven steps and went straight back
to reading — which is the point: a value measured on the bench stops working in
a darker room, and this does not. `manual` pins `exposure_start`; `camera` hands
metering back to the sensor.

## Testing

```bash
pytest        # from the repository root
```

`normalize` and `debounce` are pure standard library and fully covered.
`segment`, `reader`, `exposure` and the offset geometry need OpenCV but no
camera and no Tesseract — the OCR engine is faked.

## Known limits

- Tuned for one plate on a fixed target at ~1 m. No vehicle detection, no
  multiple plates, no reading while moving.
- The plate pattern covers `12가3456` and `123가4567`. Older regional plates
  (`서울12가3456`) are out of scope.
- `bearing_deg` assumes a pinhole camera and a nominal 70° field of view. Use
  the normalised offset for control and treat the angle as a readout.
- The template fonts are Nanum and Noto, neither of which is the plate typeface.
  It works because the answer set is 40, not because the shapes match exactly.
