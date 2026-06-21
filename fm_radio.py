import numpy as np
import scipy.signal as signal
import sounddevice as sd
from rtlsdr import RtlSdr
import queue
import threading

# Initial Configurations
fs = 2.4e6
center_freq = 98.5e6   
offset_hz = 250e3       
chunk_size = 256000     

# Init the rtl-SDR
sdr = RtlSdr()
sdr.sample_rate = fs
sdr.center_freq = center_freq - offset_hz
sdr.gain = 'auto'

# Filter Configuration
nyquist_rf = fs / 2.0
taps_rf = signal.firwin(64, 100e3 / nyquist_rf)
zi_rf = signal.lfilter_zi(taps_rf, 1.0) * 0.0

fs_demod = fs / 10 
nyquist_audio = fs_demod / 2.0
b_audio, a_audio = signal.butter(2, 15e3 / nyquist_audio)
zi_audio = signal.lfilter_zi(b_audio, a_audio) * 0.0

# 75us de-emphasis filter (standard for us fm broadcasts)   
fs_audio = 48000
tau = 75e-6
alpha = np.exp(-1 / (fs_audio * tau))
b_deemph = [1 - alpha]
a_deemph = [1, -alpha]
zi_deemph = signal.lfilter_zi(b_deemph, a_deemph) * 0.0

# Threading Queues
rf_queue = queue.Queue(maxsize=10)    # Bucket for raw radio data
audio_queue = queue.Queue(maxsize=50) # Bucket for finished audio
running = True

def audio_callback(outdata, frames, time, status):
    # Pulls from the processed audio queue to feed the sound card
    try:
        chunk = audio_queue.get_nowait()
        outdata[:] = np.float32(chunk).reshape(-1, 1)
    except queue.Empty:
        outdata[:] = np.zeros((frames, 1), dtype=np.float32)

stream = sd.OutputStream(
    samplerate=fs_audio, 
    channels=1, 
    blocksize=int(chunk_size / 50),
    callback=audio_callback
)

def sdr_worker():
    # Just reads raw samples and tosses them into th queue
    # kept in its own thread to avoid blocking the dropping samples
    while running:
        try:
            samples = sdr.read_samples(chunk_size)
            # Put data in queue; timeout allows thread to check 'running' flag and exit
            rf_queue.put(samples, timeout=1) 
        except queue.Full:
            print("Warning: DSP math is lagging, dropping an RF chunk.")
        except Exception as e:
            if running: print(f"SDR Reader Error: {e}")
            break

# Processing Thread
def process_dsp():
    
    global zi_rf, zi_audio, zi_deemph
    current_phase = 0.0
    last_sample = np.complex128(0)
    
    while running:
        try:
            samples = rf_queue.get(timeout=0.5)
        except queue.Empty:
            continue # Wait for the SDR worker to fetch more data

        # shifts frequency down to undo the offset tuning from earlier 
        t = np.arange(len(samples)) / fs
        phase_array = 2 * np.pi * offset_hz * t + current_phase
        current_phase = (phase_array[-1] + (2 * np.pi * offset_hz / fs)) % (2 * np.pi) 
        shifted_samples = samples * np.exp(-1j * phase_array)

        # rf filter and decimate down to 240kHz
        filtered_rf, zi_rf = signal.lfilter(taps_rf, 1.0, shifted_samples, zi=zi_rf)
        decimated_rf = filtered_rf[::10]

        # polar discriminator fm demodulation 
        # we keep the last sample from the previous chunk to avoid a pop at the boundary
        combined = np.insert(decimated_rf, 0, last_sample)
        last_sample = decimated_rf[-1]
        demodulated = np.angle(combined[1:] * np.conj(combined[:-1]))

        # Audio lowpass and decimate to 48kHz
        audio_filtered, zi_audio = signal.lfilter(b_audio, a_audio, demodulated, zi=zi_audio)
        audio_signal = audio_filtered[::5] 

        # De-emphasis
        de_emphasized, zi_deemph = signal.lfilter(b_deemph, a_deemph, audio_signal, zi=zi_deemph)

        # volume scale 
        audio_normalized = de_emphasized * 0.5 

        try:
            audio_queue.put_nowait(audio_normalized)
        except queue.Full:
            pass 


stream.start()

reader_thread = threading.Thread(target=sdr_worker, daemon=True)
reader_thread.start()

print("Streaming FM radio... Press Ctrl+C to stop.")

try:
    process_dsp()
except KeyboardInterrupt:
    print("\nCaught Ctrl+C! Stopping system safely...")
finally:
    # Clean up 
    running = False
    reader_thread.join(timeout=1.0)
    sdr.close()
    stream.stop()
    stream.close()
    print("Hardware released.")
