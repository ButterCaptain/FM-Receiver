# RTL-SDR Software-Defined FM Radio

---

## What This Is

A working FM radio receiver built entirely in software. You plug in a $25 RTL-SDR USB dongle, point it at a station, and audio comes out of your speakers — no FM chip, no dedicated hardware decoder. Just math running on the CPU turning raw IQ samples into audio in real time.

About 100 lines of Python once you strip the comments. Getting it to run without glitches took considerably longer.

<img width="4284" height="5712" alt="IMG_2078" src="https://github.com/user-attachments/assets/12924a47-b4ac-4a0f-9b93-e529f77c6e3c" />

---

## Why I Built This

I took signals and systems last year and somewhere in the middle of it I realized that software-defined radio existed, that you could replace a rack of analog hardware with a cheap USB stick and some Python. That felt worth exploring.

I used the [RTL-SDR.com](https://www.rtl-sdr.com) blog and the [pyrtlsdr docs](https://pyrtlsdr.readthedocs.io) heavily. I didn't come into this knowing DSP. I learned each piece as I needed it to fix something that was broken. The write-up below reflects that.

---

## Signal Chain

The RTL-SDR outputs raw **IQ samples**, complex numbers representing the baseband signal. The pipeline to turn those into audio:

```
[RTL-SDR hardware]
        |
   IQ samples @ 2.4 MHz
        |
[Frequency shift] — corrects for the offset tuning trick
        |
[FIR lowpass + decimate ×10] — 2.4 MHz → 240 kHz
        |
[Polar discriminator] — FM demodulation
        |
[Butterworth lowpass + decimate ×5] — 240 kHz → 48 kHz
        |
[De-emphasis IIR] — undoes the transmitter's pre-emphasis
        |
[sounddevice callback] — audio output
```

### Offset Tuning

RTL-SDR chips produce a DC spike at the center of the tuned frequency, it's a hardware artifact of zero-IF receivers. If you tune directly to a station, that spike lands on your signal. The fix is to tune the hardware **250 kHz off-target** and shift the signal back down in software after sampling. One line of math sidesteps a hardware limitation entirely, which is a good illustration of what SDR actually is.

### FM Demodulation — Polar Discriminator

FM encodes audio as instantaneous frequency deviation. To recover it, you measure the phase change between consecutive samples:

```python
demodulated = np.angle(combined[1:] * np.conj(combined[:-1]))
```

This gives instantaneous frequency deviation directly, which is proportional to the original audio. I found it while reading through SDR community resources and it's the right tool for this — no need for a PLL or a hardware discriminator circuit.

### De-emphasis

US FM broadcasts apply **pre-emphasis** before transmitting, high frequencies are intentionally boosted to improve SNR over the air. Receivers are supposed to apply the inverse filter on the other end. The standard time constant is **75 µs**, which sets the filter's corner frequency. I didn't know this existed until the audio sounded wrong. Once I added it, the difference was immediately obvious.

---

## Problems I Actually Had to Solve

### 1. Sample Drops and Audio Lag

The naive single-threaded approach was: read samples → run DSP math → output audio → repeat. The RTL-SDR has an internal hardware buffer that overflows if you don't drain it fast enough, and the DSP math is slow enough that it couldn't keep up.

The fix was splitting the work into a three-stage pipeline:

1. **`sdr_worker` thread** — dedicated to reading hardware. Does nothing else, never waits on math.
2. **`process_dsp` (main thread)** — pulls raw chunks from a queue, runs the full signal chain, pushes audio to a second queue.
3. **`audio_callback`** — sounddevice background callback, pulls from the audio queue in real time.

The queues decouple the stages so hardware I/O, CPU-heavy DSP, and audio output all run independently without blocking each other.

### 2. Periodic Pops in the Audio

After fixing the threading, there were loud pops at consistent intervals — exactly at chunk boundaries. This one took a while to track down.

The problem was in the **polar discriminator**. It computes phase differences between adjacent samples:

```python
demodulated = np.angle(combined[1:] * np.conj(combined[:-1]))
```

At the start of each new chunk, there's no previous sample from the prior chunk, I was defaulting it to zero. A complex zero causes a maximal phase jump, which is exactly what a pop sounds like.

The fix is to carry the last sample across chunk boundaries and prepend it to the next one:

```python
combined = np.insert(decimated_rf, 0, last_sample)
last_sample = decimated_rf[-1]
```

The IIR and FIR filter states (`zi`) have the same continuity requirement and are also carried across chunks, but the discriminator's missing sample was the source of the audible artifact.

---

## What I'd Do Differently

- **Wrap state in a class.** Using `global` for `zi_rf`, `zi_audio`, and `zi_deemph` works but is messy. A `DSPProcessor` class would be cleaner.
- **Stereo decoding.** FM stereo uses a 19 kHz pilot tone and a difference signal at 38 kHz. It's a meaningful extension I haven't built yet.
- **Configurable parameters.** Station frequency, gain, and volume are hardcoded. They should be CLI arguments.
- **Waterfall display.** A live FFT visualization would make this more useful for actually exploring the spectrum.

---

## Setup

**Dependencies:**
```
numpy
scipy
sounddevice
pyrtlsdr
```

**System library** (required by pyrtlsdr):
- macOS: `brew install librtlsdr`
- Linux: `sudo apt install librtlsdr-dev`

**Run:**
```bash
pip install numpy scipy sounddevice pyrtlsdr
python fm_radio.py
```

Set `center_freq` to a strong local FM station before running.

---

## C++ Rewrite — Personal Challenge

After getting the Python version working, I rewrote the whole thing in C++ as a personal challenge. The signal chain is identical, but everything had to be built manually: the filter classes, thread-safe queue, IIR difference equation, all of it. Python's `scipy.signal` does a lot of that for you and you don't realize how much until it's gone.

The main things that were different in C++:

**Manual filter state.** In Python, `scipy.signal.lfilter` accepts and returns a state vector (`zi`) that you just pass back in next iteration. In C++, each filter class owns its own internal delay line and history buffers. It's more code, but it made the statefulness explicit in a way that actually helped me understand what the filter is doing between chunks.

**No `np.angle` shortcut.** The Python polar discriminator is one line. In C++ it expands to the full atan2 formula with the real and imaginary parts of the cross-product computed explicitly. Writing it out made it clearer why it works.

**PortAudio vs sounddevice.** Python's `sounddevice` wraps PortAudio and handles the callback threading for you. In C++ I used PortAudio directly with blocking writes (`Pa_WriteStream`) instead of a callback, which simplified the threading model, the DSP loop just writes audio at the end of each iteration and PortAudio blocks until the soundcard is ready.

**Queue design.** Python's `queue.Queue` has `put_nowait` and `get` built in. In C++ I wrote a small `BoundedThreadSafeQueue` using a mutex and `condition_variable`. The bounded drop behavior (drop oldest instead of blocking) was something I had to think through explicitly.

The C++ version is more verbose, but writing it out manually gave me a better understanding of what the Python version was doing under the hood.

**Build (macOS with Homebrew):**
```bash
brew install librtlsdr portaudio
g++ -std=c++17 -O2 fm_receiver.cpp \
    -I/opt/homebrew/include -L/opt/homebrew/lib \
    -lrtlsdr -lportaudio -o fm_receiver
./fm_receiver
```
