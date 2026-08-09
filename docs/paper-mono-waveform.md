# Paper Mono three-level waveform (SSD1677)

This note separates facts from hypotheses. Controller behavior comes from the
SSD1677 data sheet, electrophoretic behavior comes from E Ink publications and
peer-reviewed work, and panel-specific numbers come from `src/PanelLab.cpp`.

"Shipped waveform" below is the configuration that was confirmed acceptable on
the connected unit. Everything from "Experiment journal" onward is the path that
led there, kept because several of its rejected candidates document failure
modes that are easy to re-introduce. Those sections are history, not the
current design.

## Product refresh modes

Paper Mono deliberately separates navigation UI from book rendering:

| Surface | Default output | Waveform |
|---|---|---|
| Menus, dialogs, status UI, boot and sleep screens | two levels | panel OTP non-flashing B/W fast refresh |
| Reader `Fast` | two levels | the same panel OTP non-flashing B/W fast refresh |
| Reader `Balanced` | three levels | one target-coded W/G/B activation |
| Gray calibration | diagnostic three-level output | custom corrective waveform |

`Balanced` is the reader default. The setting is persisted as `Fast=0` and
`Balanced=1`; these numeric values must not be reordered. TXT, EPUB, and XTC
all use the same policy. `Fast` does not disable the saved text-antialiasing
preference, so switching back to `Balanced` restores it. EPUB images in `Fast`
use a midpoint B/W threshold but do not build gray selector planes.

The OTP path reloads the module's own LUT, temperature selection, and analog
calibration whenever it follows a custom grayscale update. If the recorded
glass state is gray, the host supplies an opposite binary OLD value so the OTP
update actively reaches the requested black or white endpoint. This avoids
using another panel's LUT for ordinary UI updates.

## What the panel is doing

The film contains oppositely charged black and white pigment particles in a
fluid. An electric field moves one pigment toward the viewing surface and the
other away. The display is bistable, but switching is not a linear
voltage-to-gray conversion: particle mobility, wall adhesion, dwell time,
temperature, previous pulses, and remnant electric field all affect the result.

Consequences for a waveform:

- A pulse sequence has optical state, electrical impulse, and history. Matching
  only the first visible result is insufficient.
- Repeated transitions must close their impulse loop. A waveform which looks
  correct on its first page can still accumulate remnant charge and make the
  nominal white background relax to gray.
- Pulse order matters even when the nominal source-rail sum is zero. A cleanup
  tail should move away from an endpoint first and finish toward its target.
- A waveform must be calibrated for this film lot and temperature. Bytes copied
  from another panel are useful evidence, not a usable preset.

## SSD1677 constraints

The file and class are historically named `Ssd1683Driver`; the Paper Mono
silicon is SSD1677.

The controller has two one-bit RAM planes, written by commands `0x24` and
`0x26`. They select one of four source waveform entries:

```text
entry = (RAM_0x26_bit << 1) | RAM_0x24_bit
```

One activation therefore gives a pixel only four possible schedules. A direct
three-state transition matrix needs at least five:

```text
hold, lighten one step, darken one step,
lighten two steps, darken two steps
```

An exact source-aware all-pairs W/G/B update cannot fit in one SSD1677
activation while retaining a hold class. The current candidate instead assigns
entries 1/2/3 to target W/G/B and conditions every pixel on every Balanced
page. Its reset/develop paths are local to each target class; there is no
whole-screen white or black reference image. The cost is a longer first
activation and the need to validate repeated electrical history on this panel.

The SSD1677 LUT provides ten groups, four phases per group, phase lengths up to
255 frames, and group repetition. Source codes are VSS, VSH1, VSL, and VSH2.
Code zero is VSS, not high impedance. With DCVCOM enabled, simple `V * frames`
arithmetic is only a nominal first-order check; it is not a measurement of
pixel charge or remnant voltage.

Controller details corrected in this change:

- `0x3C = 0x80` selects a VCOM border. SSD1677 HiZ would be `0xC0`.
- command `0x21` takes one data byte, not two.
- command `0x10` enters SSD1677 deep sleep with `0x03`, not `0x01`.

## Why the previous fast path grayed the panel

At its defaults, the old target-coded fast LUT used six repetitions of one
away frame and three target frames:

```text
target white:  +15*1, -15*3, repeated 6 = -180 nominal V*frames
target gray:   -15*16, +5*48             =    0 nominal V*frames
target black:  -15*1, +15*3, repeated 6 = +180 nominal V*frames
```

White and black cancel only for a particular alternating endpoint sequence.
Arbitrary three-level loops do not:

```text
W -> G -> W = 0 + (-180) = -180
G -> B -> G = +180 + 0   = +180
```

A later zero-net full waveform can re-anchor pigment optically, but cannot
repay an unknown per-pixel impulse history. This matches the observed symptom:
the first refresh looks good, repeated pages acquire a gray cast, and a flashing
full refresh temporarily repairs it.

The old maintenance pass added two more problems. It drove white first and
finished with the black-going weak rail, and its `-15*1 + +5*3 == 0` argument
ignored DCVCOM and mobility asymmetry. It has been removed from the background
maintenance path.

The full waveform also accepted `tGray` values not divisible by three while
using `tGray / 3` for compensation. Values such as 40, 44, and 52 silently left
a class-specific remainder. Full-waveform calibration is now quantized to a
multiple of three.

## Shipped waveform

One activation. Ordinary pages run 56 frames / 280 ms at the 5 ms frame rate;
corrective sweeps run the longer 64-frame profile. No second stage.

### Frame budget

`TriParams` holds the three lengths the balance equations are derived from.
`makeTriLut(out, corrective)` generates both profiles from them:

```text
              page   corrective
preUp    k =    12       16   activation kick, shared by all driven classes
tGray    =      24       24   weak-rail (+5 V) frames that develop the middle tone
tBlack   =      24       32   strong-rail (+15 V) frames that develop black
total    =      56       64   frames
```

Each driven class spends its own number of white frames so that its net
`V * frames` is exactly zero, and the trajectories are right-aligned into the
tail so every class finishes moving toward its own target. For the page
profile, into a 44-frame tail:

```text
white: +15*12  +15*12     -15*24       = 0     (packed erase, see below)
gray:  +15*12  +5*24      -15*(12+8)   = 0
black: -15*12  +15*24     -15*(24-12)  = 0
```

The black class takes its kick on the opposite rail (`makeTriLut` assigns LUT
entry 3 `VS_WHITE` for the whole kick group). Kicking a pixel that is about to
be driven black *toward* black would saturate it before the drive phase starts
and cost the tail its headroom, so the kick pushes it white first and the
+15 V drive that follows pays that back along with the class's white dose.

That reverse swing — target black sitting on the white rail before it develops —
is what the eye reads as the page "flashing", and it is the reason the page
profile is 12/24 rather than 16/32. Shortening the kick and the black develop
cuts the reverse-swing interval by a quarter while leaving `tGray` alone, so the
middle tone still lands at ~40 % of the weak-rail swing. `tGray` remains
settings-driven (`lightFrames`); both profiles share it.

### The erase class packs before it wipes

The white class could simply idle through the tail and wipe for `k` frames at
the end. It does not. `packedErase` gives it a second `k`-frame charge toward
black first, then wipes for `2k`. Net impulse is unchanged (still zero) and the
timeline is the same length, but the wipe now carries twice the dose and
arrives after an away-then-toward excursion — which is what clears the residual
ghost that solid black areas leave when they are erased. The corrective profile
does not use it; a corrective sweep is a full-swing reset already.

`tGray` is quantized to a multiple of three because the gray class pays its
weak-rail dose back at one third the rail voltage; a remainder would leave a
class-specific debt.

### Why gray uses the weak rail

Measured on this glass: full optical swing takes about 10 frames at ±15 V but
about 60 frames at +5 V. Driving the middle tone on VSH2 therefore buys more
than ten resolvable steps instead of one or two, and the longer, gentler
excursion does not show the left/right gate-droop shading that a strong-rail
partial pulse does. Black and white stay on the strong rails so the total stays
inside the frame budget.

### DC balance must be per class, not amortised

Each class's net impulse has to vanish on its own. Consider the three closed
optical loops:

```text
W -> B -> W  sums netB + netW
W -> G -> W  sums netG + netW
G -> B -> G  sums netG + netB
```

Requiring all three to vanish forces `netW = netG = netB = 0` individually.
This is why a periodic corrective refresh cannot substitute for balanced
classes: the accumulated debt is per pixel and depends on that pixel's own
transition history, which the host does not track.

### The analog registers are not optional

Every timing constant above was measured under one specific analog set:

```text
VGH  0x17 (0x03)          VSH1 0x41  +15 V strong black rail
VSH2 0xA8  +5 V weak rail VSL  0x32  -15 V strong white rail   (0x04)
VCOM 0x30 (0x2C)
```

`loadCustomLut()` re-asserts all five on every LUT load. They are volatile
across reset and deep sleep, and each OTP trigger (`0x22 = 0xFC`) reloads the
module's own bank over them, so a Balanced page that follows any UI page would
otherwise run with foreign rails. The observed failure is specific and
recognisable: an unspecified VSH2 leaves the weak-rail develop doing nothing, so
antialiased edges stay white instead of gray, while an unspecified VSL/VCOM
tints the nominal white background gray. If the panel ever shows "gray
background, black glyph cores, white glyph outlines", check these writes first —
it is not a plane or entry swap.

### Entry 0 is the hold entry, and it is idle

On an ordinary page the driven set is `changed | target-gray`. Everything else —
unchanged white background *and* unchanged black — selects entry 0, which is
VSS for the whole waveform.

Holding unchanged black is the fix for the "text vanishes then re-inks"
whiteout. Per-class DC balance forces every driven black pixel to pay its dose
in white frames, so a glyph pixel that both pages share used to be driven fully
white for ~160 ms and then re-inked, purely to be re-anchored at a value it
already had. No reshaping of the LUT can remove that transit; only not driving
the pixel can. Unchanged gray stays driven, because the middle tone is the
drift-fragile one.

This is also why entry 0 no longer carries the white-biased background top-up
(a 1-frame +15 V / 5-frame -15 V pulse folded into the kick group) that earlier
builds used to shave residue off unchanged white on every page. Once entry 0
also holds black, that dose bleaches held glyphs within a few pages. Residue is
now aimed where it can be aimed: the packed erase wipe above, and the corrective
sweep, which drives every pixel and therefore never selects entry 0 at all.

The cost is that background and held-glyph re-anchoring is deferred to the
corrective cadence (`refreshFrequency`, default every 10 pages). Judge graying
of static glyphs across a full cadence interval, not one page turn.

## Experiment journal

Superseded. Retained for the failure modes it documents.

### Two-stage candidate

Stage 1 is one target-coded activation. The RAM selectors map every target
white, gray, and black pixel to LUT entries 1, 2, and 3 respectively, regardless
of its recorded source. The right-aligned trajectories finish toward their own
target and take 80 frames / 400 ms at the 5 ms frame rate. This is the same
trajectory which produced the best first-page endpoints in the earlier B/C
tests. Reusing it on every page is the E experiment: stable black is actively
re-anchored instead of receiving only entry-zero time from page two onward.

E used a separate 24-frame white-field cleanup as stage 2. On hardware its
middle tone was no longer distinguishable from white and antialiased text
became visibly thinner than Fast text. F is a one-variable diagnostic: it keeps
the stage-1 LUT and selector planes byte-identical while setting cleanup to zero.
The log confirmed `gray=1`, but the lower white field became deep gray as soon
as the reader opened. This proves the gray planes were present, stage 1 alone
does not land white uniformly, and E's second stage was simultaneously repairing
white while exposing gray and black to entry-zero/DCVCOM.

G restores a 24-frame second stage but reuses target-coded W/G/B selectors. Per
four-frame repeat W uses B,W,B,W; G uses B,W,W,B; and B uses W,W,B,B. Every
class therefore sees two VSH1 and two VSL frames and no class falls through to
VSS. White finishes white-going; gray and black finish black-going.

G hardware logs exposed a separate atomicity bug. Normal pages reported
`stages=2 frames=80+24`, but input arriving during stage 1 produced
`stages=1 frames=80+0` and left the whole background at the visibly gray
intermediate state. H makes stage 2 mandatory after stage 1 starts. It also
raises black development from 32 to 40 frames while increasing that class's
white reset from 16 to 24 frames, preserving its nominal zero source impulse
and the overall 80-frame stage-1 duration. The middle-tone trajectory is
unchanged.

I addresses the H observations that the page still flashed too much, black was
washed out, and one completed update could optically remain on a gray field.
For an ordinary page, unchanged white now selects entry 0 in stage 1, while all
changed pixels and every target gray/black pixel keep the W/G/B target-coded
drive. Stage 2 assigns entry 0 the same white schedule as entry 1, so this is not
an unpolished VSS hold. The polish is shortened from 24 to 12 frames. White
alternates B/W for six cycles; black sees the same six white and six black
frames but groups all release frames first and finishes with six contiguous
black-going frames. This preserves equal source-rail counts while avoiding the
repeated black release that made H text light. Gray keeps the B/W/W/B closed
excursion. Corrective/first-page updates still actively drive every pixel.

### Stage 2 post-clean

Every target-white pixel is cleaned, including unchanged background. This keeps
an erased glyph and the surrounding white field on the same waveform history.
Gray and black use entry zero during this activation. Hardware testing rejected
the nominally symmetric black sequence because it bleached stable text from
about 80% to about 40% black on the second update.

The cleanup candidate is the symmetric sequence generated and timed by the lab:

```text
+15 V for 1 frame  (move away from the white endpoint)
-15 V for 1 frame  (return toward white; final active frame)
```

The E white sequence was 12 cycles, or 24 frames / 120 ms. F temporarily used
zero cycles to isolate its optical effect. G uses six four-frame repeats, also
24 frames / 120 ms, but explicitly drives all three targets. D appended a
six-frame black-only group; it was removed after the whole white/gray field
shifted gray. A timing group is global, so white and gray spent those six frames
at entry-zero VSS even though only black had an explicit source pulse. This is
not evidence of pixel-relative DC balance: entry zero is VSS,
LUT4 code zero is DCVCOM, and TFT feed-through and particle mobility are not in
the source-rail model. The old background scrub is not scheduled afterward, so
an obsolete mask cannot run after a newer page.

The cleanup count is deliberately exposed as `cleanupPasses` in production and
`postclean` in PanelLab. More cycles may remove more residue, but must be judged
against flicker, relaxed L*, and VCOM-related drift rather than assumed safe.

### Earlier one-shot parameters

An 80-frame variant preceded the shipped 64-frame one. Its gray class split the
kick into a self-cancelling `+15*8 -15*8` pair, which wasted half the activation
and, combined with a 12-frame saturation floor on the white reset, forced
`tGray >= 36`. That is more than half the weak-rail swing, so the "middle" tone
always landed near black and thin CJK strokes read as two levels:

```text
white: +15*16 -15*16 = 0
gray:  (+15*8 -15*8) -15*16 +5*48 = 0
black: -15*16 -15*24 +15*40 = 0
```

Giving gray the full kick toward black decouples the tone from the saturation
floor and is what made `tGray = 24` reachable.

These equations omit DCVCOM. SSD1677 source code 00 is VSS and LUT4 code 00 is
DCVCOM, so source-rail zero is not proof of zero pixel-voltage impulse. The E
test first isolates the known D selector/tail problems; VCOM compensation must
be calibrated separately if repeated E pages still walk in one direction.

### PanelLab validation

Build and flash the dedicated lab environment:

```sh
pio run -e paper_mono_lab -t upload
scripts/panel_lab.py
```

Relevant commands:

```text
gtg3                         run the production one-shot + cleanup transition
abclean [n]                  leave an A | B | A cleanup comparison after n loops
tri                          run the 80-frame balanced reference waveform
set postclean N              white cleanup cycles
set lutdump 1                decode LUT and compare predicted/BUSY time
img a / img b, ab N          upload and repeatedly flip real pages
```

`abclean 3` first performs one common two-cycle deep clear, then runs identical
W/G/B-to-white histories across the whole panel three times. It finally applies
the cleanup candidate only to the center third. The final white screen is:

```text
A: no cleanup | B: 12-cycle cleanup | A: no cleanup
```

The setup contains a visible full black/white clear. It must not be run without
the observer's confirmation. After it finishes, ask which region is whiter and
has less retained stripe/text structure, and whether the center cleanup flicker
was acceptable. Controller completion does not answer any of those questions.

Hardware timing observed on the connected unit for the current defaults:

- two-cycle deep clear: about 2470 ms / 120 frames;
- one-shot W/G/B stage: 80 frames / about 400 ms of waveform time;
- 12-cycle white cleanup: 24 frames / about 120 ms;
- a cold stage also includes roughly 80 ms of rail/startup overhead.

These measurements only confirm that the controller executed the requested
timeline. They are not optical validation, and no waveform is accepted until
the physical A/B result is confirmed by the observer.

Do not judge only one A-to-B flip. The minimum useful test set is:

1. Prepare a trusted W, G, or B source and run all nine transitions.
2. Measure or photograph L* after about 100 ms, 1 s, and 10 s of dwell.
3. Repeat W-G-W, G-B-G, and W-B-W loops at least 50 times.
4. Repeat with text, large flat fields, and left/center/right patches.
5. Repeat at cold, room, and warm panel temperatures.
6. Sweep `postclean` around 0, 6, 12, and 24. Change one variable at a time.

Acceptance is not merely "looks correct now." Endpoint and middle L* must stay
within tolerance after dwell, loop endpoints must not walk, unchanged areas
must not tint, and no transition may show a white/black reference flash.

## Interactive waveform editor

`tools/waveform-editor.html` is a single-file editor for the production
pipeline above. Open it in desktop Chrome (directly from disk; Web Serial needs
no server), connect to the running CrossPoint firmware's USB CDC port, and edit
the three host-authored LUT slots as a drag-and-drop sequencer: the shared
TP/RP timing is one ruler (SSD1677 timing is global across entries), each
entry is a lane painted with one of the four source rails, and the page shows
per-class net V·frames balance, a first-order optical prediction (±15 V ≈ 10
frames full swing, +5 V ≈ 60 frames, as measured on this glass), and the
driver's real `SSD1677 update:` timings parsed live from the log.

The firmware side is `src/util/WaveformLab.cpp` plus override hooks in
`Ssd1683Driver`, compiled only with `-DFREEINK_WAVEFORM_LAB=1` (on in the
`paper_mono` dev env, absent everywhere else). It is runtime-inert by default:
until a `CMD:WAVE SET/LOAD` installs an override — or the user has explicitly
saved `/waveforms/boot.txt` — `runUpdate()` takes its original `makeTriLut()`
branch and the SPI byte stream is identical to a build without the flag.
`CMD:WAVE OFF` returns to stock behavior at any time.

Slots map onto the real stages: `A` replaces the per-page tri activation, `B`
adds a second activation (the mandatory-stage-2 structure the H experiment
validated), `C` replaces the LUT used by corrective sweeps and falls back to
`A`, then to the built-in waveform. The corrective cadence is the existing
`refreshFrequency` setting; `CMD:WAVE CADENCE` changes it RAM-only.

Serial protocol (over the existing `CMD:` channel, replies prefixed `WAVE:`):

```text
CMD:WAVE STATUS              mode/cadence/active-mask/baked report
CMD:WAVE GET A|B|C           dump the slot's next-run LUT as 222 hex chars
CMD:WAVE SET A|B|C <hex222>  install a RAM override for the slot
CMD:WAVE CLEAR A|B|C         drop one slot's override
CMD:WAVE OFF                 drop all overrides — stock behavior
CMD:WAVE TEST                corrective redraw of the current screen
CMD:WAVE CADENCE 1|5|10|15|30
CMD:WAVE HOLD 0|1            unchanged black holds on entry 0 (default 1)
CMD:WAVE SAVE|LOAD <name>    persist/restore /waveforms/<name>.wfm on SD
CMD:WAVE LIST                list saved sets
CMD:WAVE BOOT <name>|OFF     write/remove the boot marker
```

`HOLD` is on by default — it is production behaviour, described under "Entry 0
is the hold entry" above. The command exists so an experiment can turn it *off*:
a PAGE override whose entry 0 is not idle (the pre-bake stock LUT, for instance,
which carried the background top-up) will bleach held black, so send `HOLD 0`
before installing one. `WAVE OFF` restores the default along with the slots.

A runtime override costs nothing per page: the LUT already travels host RAM →
SPI on every custom activation, so the override only changes which 111 bytes
are copied. For a compile-time default, the editor exports
`PaperMonoWaveformBaked.h`; dropped next to `Ssd1683Driver.cpp` (gitignored) it
replaces the generated waveform via `__has_include`, and deleting it restores
stock. The editor's optical model is a first-order sanity check only — every
acceptance rule in the PanelLab section (nine transitions, dwell, loops,
temperatures) still applies before shipping a waveform.

## Source comparison

- [E Ink: how electrophoretic ink works](https://www.eink.com/tech/detail/How_it_works)
  describes the oppositely charged pigment particles and field-driven optical
  state.
- [SSD1677 data sheet](https://www.e-paper-display.com/SSD1677Specification.pdf)
  defines the two RAM planes, four LUT entries, 40 programmable phases, voltage
  codes, border setting, and deep-sleep command.
- [E Ink US7193625B2](https://patents.google.com/patent/US7193625B2/en)
  gives a three-state transition table and explains that alternating pulses can
  converge particles toward a middle state; importantly, its gray transitions
  depend on the source state.
- [E Ink US20090195568A1](https://patents.google.com/patent/US20090195568A1/en)
  discusses history, dwell, temperature, and errors in direct gray-level drive.
- [E Ink US20140340430A1](https://patents.google.com/patent/US20140340430A1/en)
  formalizes impulse-potential transitions as target potential minus source
  potential, the model used by `gtg3`.
- [Optimized activation waveform study](https://www.mdpi.com/2072-666X/11/5/498)
  compares erase/activation/write structure and the non-linear response of
  electrophoretic particles.
- [M5GFX SSD1677 implementation](https://github.com/m5stack/M5GFX/blob/master/src/lgfx/v1/panel/Panel_SSD1677.cpp)
  is useful controller evidence: its quality and differential modes keep source
  history and use multiple transition steps. Its literal LUT values are not
  used here because they target a different film and optical encoding.
